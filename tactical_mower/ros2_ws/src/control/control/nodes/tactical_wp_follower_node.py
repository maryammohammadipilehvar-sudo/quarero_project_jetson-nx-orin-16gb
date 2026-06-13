#!/usr/bin/env python3
"""ROS2 node for tactical waypoint following.

This node orchestrates navigation components to follow waypoints,
avoid obstacles, and handle charging station docking.
"""

from dataclasses import asdict
import math
import yaml
import json
import rclpy
from rclpy.node import Node
from pathlib import Path
from typing import Optional, Tuple, List
from time import time

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3Stamped
from nav_msgs.msg import Odometry, OccupancyGrid
from interfaces.msg import CommandDrive, GeoPath, ObstacleSectors
from interfaces.srv import CommandControl, WaypointService, GetScheduleAction
from std_msgs.msg import Bool, Float32, String
from fixposition_driver_msgs.msg import FusionEpoch

from ..navigation.coordinate_transformer import CoordinateTransformer
from ..navigation.transform_manager import TransformManager
from ..navigation.waypoint_follower import WaypointFollower, WaypointMode
from ..navigation.waypoint_follower_nav2 import WaypointFollowerNav2
from ..navigation.speed_estimator import SpeedEstimator
from ..obstacle_avoidance import ObstacleAvoidanceController
from ..gnss.rtk_status_monitor import RTKStatusMonitor, RTKStatus
from ..robot_state_machine import RobotStateMachine, RobotState, RobotStateContext
from ..routes.route_manager import RouteManager
from ..home_return.home_return_controller import HomeReturnController


class TacticalWpFollowerNode(Node):
    """ROS2 node for waypoint following with obstacle avoidance and docking."""

    def __init__(self):
        """Initialize the TacticalWpFollowerNode."""
        super().__init__('tactical_wp_follower')

        # Parameters
        self.declare_parameter('settings_file', '/routen/settings/settings.yaml')
        self.declare_parameter('routes_dir', '/routen/routes')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_steering', 100.0)
        self.declare_parameter('replan_check_interval', 2.0)  # Interval in seconds between path collision checks (<=0 for continuous)

        # State persistence handled by centralized StateManager (singleton)
        
        # Obstacle avoidance parameters
        self.declare_parameter('obstacle_avoidance.speed_reduction_slow', 0.6)
        self.declare_parameter('obstacle_avoidance.initial_stop_duration', 3.0)
        self.declare_parameter('obstacle_avoidance.resume_clear_duration', 3.0)
        self.declare_parameter('obstacle_avoidance.deadlock_timeout', 30.0)
        self.declare_parameter('obstacle_avoidance.lidar_timeout', 2.0)

        # Structured navigation telemetry ([NAVTEL]) for evidence-based troubleshooting.
        # Always-on, additive, fail-soft — does NOT influence any motion decision.
        self.declare_parameter('telemetry.enabled', True)
        self.declare_parameter('telemetry.heartbeat_period', 2.0)  # seconds between heartbeat lines when nothing changes

        # Load parameters
        self._settings_file = Path(self.get_parameter('settings_file').value).expanduser()
        configured_routes_dir = Path(self.get_parameter('routes_dir').value).expanduser()
        control_freq = self.get_parameter('control_frequency').value
        max_steer = self.get_parameter('max_steering').value
        replan_check_interval = self.get_parameter('replan_check_interval').value
        
        # Prefer an existing routes directory to avoid mismatches between containers
        candidate_dirs = [
            configured_routes_dir,
            Path("/routen/routes"),
            Path("/data/routes"),
        ]
        routes_dir = next((p for p in candidate_dirs if p.exists()), configured_routes_dir)
        self.get_logger().info(f"Using routes directory: {routes_dir}")
        
        # Initialize settings-related attributes before loading
        self.max_speed = 1.0
        self._settings_cache = {}
        self._home_position: Optional[Tuple[float, float, float]] = None
        self._charge_position: Optional[Tuple[float, float, float]] = None
        self._dock_home_radius = 2.0  # meters
        
        # Initialize CoordinateTransformer early (needed by HomeReturnController and TransformManager)
        self._coord_transformer = CoordinateTransformer()
        
        # Initialize TransformManager early (map origin will be set from TF+GPS)
        self._transform_manager = TransformManager(node=self)
        
        # Load settings early to get obstacle avoidance flag and waypoint tolerance
        self._load_settings()
        
        # Load max_route_distance from settings for autonomous recovery
        self._max_route_distance = float(self._settings_cache.get('max_route_distance', 3.0))
        self.get_logger().info(f"Max route distance for recovery: {self._max_route_distance}m")
        
        # Get waypoint tolerance from settings (default 0.5m if not set)
        wp_tolerance = self._settings_cache.get('waypoint_tolerance', 0.5)
        
        # Obstacle handling mode: 'off' | 'stop' | 'avoid' (live-controlled via /control/obstacle_mode).
        #   off   -> linear follower, no obstacle reaction (the LiDAR/sensor-loss watchdog still halts)
        #   stop  -> linear follower + protective stop: hold still until the obstacle is removed (never route around)
        #   avoid -> Nav2 follower: path-plan around obstacles
        self._obstacle_mode = self._resolve_obstacle_mode(self._settings_cache)
        # Derived flag: True whenever obstacle handling is active at all (stop OR avoid).
        # It arms the LiDAR-loss safety halt during navigation and informs follower
        # selection — it does NOT decide whether the protective-stop reflex runs. That
        # reflex is 'stop'-only (see _build_navigation_kwargs): in 'avoid' Nav2 owns
        # collision handling via its costmap, so layering the reflex on top would clamp
        # Nav2's output to zero and defeat path-planning.
        self._enable_obstacle_avoidance = (self._obstacle_mode != 'off')
        self.get_logger().info(f"Obstacle mode initialized from settings: {self._obstacle_mode}")

        # State persistence removed - state machine handles all state management
        
        # Load transform offsets from parameters (if available)
        # These can be set in params.yaml or via launch arguments
        vrtk_to_base_x = self.declare_parameter('transforms.vrtk_to_base.x', 0.0).value
        vrtk_to_base_y = self.declare_parameter('transforms.vrtk_to_base.y', 0.0).value
        vrtk_to_base_z = self.declare_parameter('transforms.vrtk_to_base.z', 0.0).value
        base_to_lidar_x = self.declare_parameter('transforms.base_to_lidar.x', 0.0).value
        base_to_lidar_y = self.declare_parameter('transforms.base_to_lidar.y', 0.0).value
        base_to_lidar_z = self.declare_parameter('transforms.base_to_lidar.z', 0.6).value  # Default lidar height
        
        # Load base_footprint transform (for 2D navigation)
        vrtk_to_base_footprint_x = self.declare_parameter('transforms.vrtk_to_base_footprint.x', vrtk_to_base_x).value
        vrtk_to_base_footprint_y = self.declare_parameter('transforms.vrtk_to_base_footprint.y', vrtk_to_base_y).value
        vrtk_to_base_footprint_z = self.declare_parameter('transforms.vrtk_to_base_footprint.z', 0.0).value  # base_footprint is on ground
        
        # Load TF timeout parameter
        tf_timeout = self.declare_parameter('tf_timeout', 0.01).value
        self._transform_manager.set_tf_timeout(tf_timeout)
        
        self._transform_manager.set_vrtk_to_base_transform(
            x=vrtk_to_base_x,
            y=vrtk_to_base_y,
            z=vrtk_to_base_z
        )
        self._transform_manager.set_vrtk_to_base_footprint_transform(
            x=vrtk_to_base_footprint_x,
            y=vrtk_to_base_footprint_y,
            z=vrtk_to_base_footprint_z
        )
        self._transform_manager.set_base_to_lidar_transform(
            x=base_to_lidar_x,
            y=base_to_lidar_y,
            z=base_to_lidar_z
        )
        
        # Publish static transforms
        self._transform_manager.publish_static_transforms()
        
        # Initialize RouteManager and HomeReturnController for autonomous recovery
        self._route_manager = RouteManager(routes_dir, logger=self.get_logger())
        # Load home_tolerance from settings (default 1.0m if not set)
        home_tolerance = self._settings_cache.get('home_tolerance', 0.5)
        self._home_return = HomeReturnController(
            self._coord_transformer,
            self._route_manager,
            tolerance=home_tolerance
        )
        # _max_route_distance is already loaded from settings in line 111
        
        # Charge position tolerances
        self._charge_position_tolerance_charging = 0.18  # 18cm "am I present at charge" detection — deliberately looser than the 5cm docking STOP (docking_state charge_position_tolerance) to give RTK-noise hysteresis so the robot can't bounce out of DOCKED. NOT the stop precision.
        self._charge_position_tolerance_undocking = 0.4  # 40cm for sending undock command
        
        # Initialize robot state machine with ROS node reference
        # States will have access to ROS node for publishers, subscribers, etc.
        self._robot_state_machine = RobotStateMachine(node=self)
        # Start in UNINITIALIZED, will transition to MANUAL when charge_pos is set
        # (handled in _load_settings after charge_position is loaded)
        
        # Initialize RTK status monitor for navigation safety
        self._rtk_monitor = RTKStatusMonitor(
            require_both_gnss=True,
            required_status=8,  # RTK_FIXED
            stability_window=5,
            min_stable_readings=3
        )
        self._rtk_check_enabled = True  # Can be disabled for testing

        # Initialize components
        # _coord_transformer is initialized earlier (needed by HomeReturnController)
        self._speed_estimator = SpeedEstimator(filter_alpha=0.25, max_deviation=1.5)
        
        # Initialize BOTH waypoint followers (Nav2 for obstacle avoidance, linear for direct navigation)
        # The active follower is selected based on obstacle avoidance enabled state
        
        # Nav2-based waypoint follower (used when obstacle avoidance is enabled)
        self._waypoint_follower_nav2 = WaypointFollowerNav2(
            node=self,
            transform_manager=self._transform_manager,
            waypoint_tolerance=wp_tolerance,
            max_steering=max_steer,
            replan_check_interval=replan_check_interval
        )
        replan_mode = "continuous" if replan_check_interval <= 0.0 else f"every {replan_check_interval}s"
        self.get_logger().info(
            f"Nav2-based waypoint follower initialized (replanning check: {replan_mode})"
        )
        
        # Traditional/linear waypoint follower (used when obstacle avoidance is disabled)
        self._waypoint_follower_linear = WaypointFollower(
            waypoint_tolerance=wp_tolerance,
            max_steering=max_steer
        )
        self.get_logger().info("Linear waypoint follower initialized")
        
        # Select active follower based on obstacle mode.
        # Only 'avoid' uses Nav2 (path planning around obstacles); 'stop' and 'off'
        # use the linear follower so the robot drives straight and the protective-stop
        # controller (for 'stop') halts it rather than routing around.
        if self._obstacle_mode == 'avoid':
            self._waypoint_follower = self._waypoint_follower_nav2
            self.get_logger().info("Active follower: Nav2 (obstacle mode: avoid)")
        else:
            self._waypoint_follower = self._waypoint_follower_linear
            self.get_logger().info(f"Active follower: Linear (obstacle mode: {self._obstacle_mode})")
        # Initialize state machine-based obstacle avoidance controller
        # This will be used when OccupancyGrid data is available via navigation_helper
        
        # Deadlock warning callback - sends warning to web app
        def deadlock_warning_callback(message: str):
            """Send deadlock warning to web app via logging topic."""
            warn_msg = String()
            warn_msg.data = message
            self._log_warn_pub.publish(warn_msg)
            self.get_logger().warning(f"Deadlock warning sent to web app: {message}")
        
        # Load obstacle avoidance parameters
        speed_reduction_slow = float(self.get_parameter('obstacle_avoidance.speed_reduction_slow').value)
        initial_stop_duration = float(self.get_parameter('obstacle_avoidance.initial_stop_duration').value)
        resume_clear_duration = float(self.get_parameter('obstacle_avoidance.resume_clear_duration').value)
        deadlock_timeout = float(self.get_parameter('obstacle_avoidance.deadlock_timeout').value)
        self._lidar_timeout = float(self.get_parameter('obstacle_avoidance.lidar_timeout').value)

        # Navigation telemetry ([NAVTEL]) state
        self._telemetry_enabled = bool(self.get_parameter('telemetry.enabled').value)
        self._telemetry_heartbeat = float(self.get_parameter('telemetry.heartbeat_period').value)
        self._navtel_last_sig = None       # last emitted change-signature
        self._navtel_last_emit = 0.0       # wall-clock of last emitted line
        self._last_user_event_msg = None   # last user-facing event sent to /tactical/logging/info
        self._current_state_name = 'UNINITIALIZED'  # cached each loop for the telemetry chokepoint
        
        # Initialize controller with deadlock callback and parameters
        # This controller will be used by navigation_helper when OccupancyGrid is available
        self._obstacle_avoidance = ObstacleAvoidanceController(
            logger=self.get_logger(),
            speed_reduction_slow=speed_reduction_slow,
            initial_stop_duration=initial_stop_duration,
            resume_clear_duration=resume_clear_duration,
            deadlock_timeout=deadlock_timeout,
            deadlock_callback=deadlock_warning_callback
        )

        # State
        self._last_gps: Optional[Tuple[float, float, float]] = None
        self._last_orientation: Optional[Quaternion] = None
        # Cached yaw (rad) from /fixposition/ypr.vector.x (ENU, 0=east).
        # The quaternion in /fixposition/odometry_enu is NOT a usable heading
        # source (verified 2026-05-17: yaw spans ±180° randomly while the RTK
        # moving-baseline heading varies coherently with actual motion).
        # _get_current_pose_map() overrides pose.orientation with this cached
        # yaw when available.
        self._last_yaw_rad: Optional[float] = None
        self._yaw_override_warned: bool = False
        self._obstacle_detected = False
        self._occupancy_grid: Optional[OccupancyGrid] = None  # Latest OccupancyGrid from obstacle detection
        self._obstacle_sectors_msg: Optional[ObstacleSectors] = None  # Latest obstacle sectors from sector publisher
        self._last_lidar_msg_time: Optional[float] = None  # Time of last message on /obstacles/lidar (Livox)
        self._lidar_timeout_warn_sent = False  # True after warning sent for current lidar timeout
        self._autonomous_operation_enabled = False  # Default False
        self._autonomous_operation_started_once = False  # Track if autonomous operation has been started at least once
        self._current_route: Optional[dict] = None  # Current route info (for tracking, not persistence)
        self._pending_geopath: Optional[GeoPath] = None  # Store geopath to start after undocking
        self._unified_charging_state = False  # Unified charging state from /robot/state
        self._charging_relay_enabled = False  # Track charging relay state
        self._charging_requested = False  # Track if charging was requested via topic
        self._has_active_schedule = False  # Track if there's an active schedule from scheduler (independent of route_active)
        
        # Startup recovery: track if we've done the initial charge position check
        # This is critical for proper recovery after container restart
        self._startup_recovery_done = False
        # Flag to preserve startup charge position detection until DOCKED/CHARGING transition completes
        self._startup_at_charge_pos = False
        # Track last attempt to set map origin (for rate limiting)
        self._last_map_origin_attempt_time: Optional[float] = None
        # manual_cmd removed - state machine now uses autonomous_operation_enabled flag
        self._robot_position: Optional[Tuple[float, float, float]] = None  # Current robot GPS position
        
        # GPS watchdog: track last known position for jump detection
        self._last_known_position_enu: Optional[Tuple[float, float]] = None
        self._gps_jump_detected = False  # Flag to stop robot when GPS jump detected
        self._last_published_state: Optional[RobotState] = None  # Track last published state for transition logging
        
        # Recovery timer: delay before running autonomous recovery in IDLE state
        # This gives the scheduler time to send the next route before recovery kicks in
        self._recovery_timer = None
        self._recovery_delay_seconds = 0.5  # 500ms delay before recovery

        # Publishers
        self._cmd_pub = self.create_publisher(CommandDrive, '/cmd_drive', 10)
        self._route_completed_pub = self.create_publisher(Bool, '/tactical/robot/route_completed', 10)
        self._charging_status_pub = self.create_publisher(String, '/tactical/robot/charging_status', 10)
        self._autonomous_operation_pub = self.create_publisher(Bool, '/control/autonomous_operation', 10)
        self._speed_estimate_pub = self.create_publisher(Float32, '/tactical/robot/speed_kmh', 10)
        self._robot_state_pub = self.create_publisher(String, '/tactical/robot/state', 10)
        self._log_warn_pub = self.create_publisher(String, '/tactical/logging/warn', 10)
        self._log_info_pub = self.create_publisher(String, '/tactical/logging/info', 10)
        # Publisher for clearing charging request when entering manual mode
        self._charging_requested_pub = self.create_publisher(Bool, '/tactical/control/charging/requested', 10)

        # Services
        self.create_service(CommandControl, '/control/autonomous_operation', self._autonomous_operation_service)
        self.create_service(WaypointService, '/control/waypoints', self._waypoint_service)
        
        # Service clients for scheduler queries
        self._schedule_action_client = self.create_client(
            GetScheduleAction, 
            '/control/get_schedule_action'
        )
        self._schedule_query_pending = False
        self._last_schedule_query_time = 0.0
        self._schedule_query_interval = 10.0  # Query scheduler every 5 seconds in allowed states
        
        # Allowed states for schedule queries (from state machine)
        self._allowed_schedule_query_states = [
            RobotState.IDLE,
            RobotState.DOCKED,
            RobotState.CHARGING,
            RobotState.UNDOCKED
        ]
        
        # Subscribers
        self.create_subscription(String, '/robot/state', self._robot_state_callback, 10)
        self.create_subscription(NavSatFix, '/fixposition/odometry_llh', self._gps_callback, 10)
        # Using /fixposition/odometry_enu from FP_A-ODOMENU - this is the most precise ENU coordinate source
        # Alternative: /fixposition/odometry_enu_smooth (from FP_A-ODOMSH) is smoothed but may have latency
        # The direct odometry_enu is preferred for precision and real-time control
        self.create_subscription(Odometry, '/fixposition/odometry_enu', self._odom_callback, 10)
        self.create_subscription(Vector3Stamped, '/fixposition/ypr', self._ypr_callback, 10)
        self.create_subscription(FusionEpoch, '/fixposition/fusion', self._fusion_callback, 10)
        self.create_subscription(Point, '/tactical/control/charging/dock', self._dock_callback, 10)
        self.create_subscription(Bool, '/tactical/control/charging/undock', self._undock_callback, 10)
        self.get_logger().info("Subscribed to /tactical/control/charging/undock topic")
        self.create_subscription(Bool, '/tactical/control/charging/requested', self._charging_requested_callback, 10)
        self.create_subscription(Float32, '/control/set_speed', self._set_speed_callback, 10)
        self.create_subscription(Bool, '/control/obstacle_avoidance_enabled', self._obstacle_avoidance_enabled_callback, 10)
        self.create_subscription(String, '/control/obstacle_mode', self._obstacle_mode_callback, 10)
        self.create_subscription(Bool, '/control/enable_charging', self._charging_relay_callback, 10)
        
        # Always subscribe to obstacle topics (controller is always initialized)
        self.create_subscription(Bool, '/obstacle_detected', self._obstacle_callback, 10)
        self.create_subscription(OccupancyGrid, '/obstacles/lidar', self._occupancy_grid_callback, 10)
        self.create_subscription(ObstacleSectors, '/obstacles/sectors', self._obstacle_sectors_callback, 10)

        # Control timer
        self._control_timer = None
        period = 1.0 / control_freq
        self._control_timer = self.create_timer(period, self._control_loop)

        # Settings reload timer - check for settings changes every 2 seconds
        self._settings_reload_timer = self.create_timer(2.0, self._load_settings)

        # User-facing event narrator — 1 Hz so the dashboard ticker also gets
        # status in MANUAL / IDLE where no drive commands flow.
        self._user_event_timer = self.create_timer(1.0, self._publish_user_event_tick)

        self.get_logger().info("TacticalWpFollowerNode initialized")
        
        # Publish initial autonomous operation state
        initial_state = Bool()
        initial_state.data = self._autonomous_operation_enabled
        self._autonomous_operation_pub.publish(initial_state)
        self.get_logger().info(f"Published initial autonomous operation state: {self._autonomous_operation_enabled}")

        # Register state entry callbacks for Moore State Machine pattern
        # All actions happen in entry callbacks, transitions are pure condition checks
        self._robot_state_machine.register_state_entry_callback(
            RobotState.MANUAL,
            self._on_manual_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.IDLE,
            self._on_idle_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.NAVIGATING,
            self._on_navigating_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.DOCKING,
            self._on_docking_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.UNDOCKING,
            self._on_undocking_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.DOCKED,
            self._on_docked_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.CHARGING,
            self._on_charging_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.UNDOCKED,
            self._on_undocked_state_entry
        )
        self._robot_state_machine.register_state_entry_callback(
            RobotState.RETURNING_TO_HOME,
            self._on_returning_to_home_state_entry
        )

    def _robot_state_callback(self, msg: String):
        """Handle robot state updates to get unified charging state."""
        try:
            data = json.loads(msg.data)
            self._unified_charging_state = bool(data.get("charging_state", False))
        except Exception as e:
            self.get_logger().warn(f"Failed to parse robot state: {e}")

    def _load_settings(self):
        """Load settings (home point, charge point, speed factor, waypoint tolerance)."""
        settings_file = self._settings_file
        settings = {}
        if settings_file.exists():
            try:
                with open(settings_file, 'r') as f:
                    settings = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().error(f"Failed to load settings: {e}")
        self._settings_cache = settings

        new_speed = settings.get('speed_factor')
        if new_speed is not None and new_speed != self.max_speed:
            self.max_speed = new_speed
            self.get_logger().info(f"Loaded max_speed from settings: {self.max_speed}")
        
        # Load waypoint tolerance from settings (update both followers for consistency)
        new_tolerance = settings.get('waypoint_tolerance')
        if new_tolerance is not None:
            new_tolerance = float(new_tolerance)
            # Update Nav2 follower
            if hasattr(self, '_waypoint_follower_nav2') and self._waypoint_follower_nav2 is not None:
                if new_tolerance != self._waypoint_follower_nav2.waypoint_tolerance:
                    self._waypoint_follower_nav2.waypoint_tolerance = new_tolerance
                    self.get_logger().info(f"Updated Nav2 follower waypoint_tolerance: {new_tolerance}")
            # Update linear follower
            if hasattr(self, '_waypoint_follower_linear') and self._waypoint_follower_linear is not None:
                if new_tolerance != self._waypoint_follower_linear.waypoint_tolerance:
                    self._waypoint_follower_linear.waypoint_tolerance = new_tolerance
                    self.get_logger().info(f"Updated linear follower waypoint_tolerance: {new_tolerance}")
        
        # Load max_route_distance from settings (for autonomous recovery)
        new_max_route_distance = settings.get('max_route_distance')
        if new_max_route_distance is not None and hasattr(self, '_max_route_distance'):
            new_max_route_distance = float(new_max_route_distance)
            if new_max_route_distance != self._max_route_distance:
                self._max_route_distance = new_max_route_distance
                self.get_logger().info(f"Updated max_route_distance from settings: {self._max_route_distance}m")
        elif new_max_route_distance is not None:
            # First time loading
            self._max_route_distance = float(new_max_route_distance)
            self.get_logger().info(f"Loaded max_route_distance from settings: {self._max_route_distance}m")
        
        # Load home_tolerance from settings and update HomeReturnController
        new_home_tolerance = settings.get('home_tolerance')
        if new_home_tolerance is not None and hasattr(self, '_home_return'):
            new_home_tolerance = float(new_home_tolerance)
            if new_home_tolerance != self._home_return.tolerance:
                self._home_return.tolerance = new_home_tolerance
                self.get_logger().info(f"Updated home_tolerance from settings: {self._home_return.tolerance}m")
        
        home_cfg = settings.get('home_point')
        if (home_cfg and 'latitude' in home_cfg and 'longitude' in home_cfg and
            home_cfg['latitude'] is not None and home_cfg['longitude'] is not None):
            self._home_position = (
                float(home_cfg['latitude']),
                float(home_cfg['longitude']),
                float(home_cfg.get('yaw', 0.0))
            )
        else:
            self._home_position = None
        
        charge_cfg = settings.get('charge_point')
        if (charge_cfg and 'latitude' in charge_cfg and 'longitude' in charge_cfg and 'yaw' in charge_cfg and
            charge_cfg['latitude'] is not None and charge_cfg['longitude'] is not None and charge_cfg['yaw'] is not None):
            self._charge_position = (
                float(charge_cfg['latitude']),
                float(charge_cfg['longitude']),
                float(charge_cfg['yaw'])
            )
            # Map origin will be set from TF and GPS (ensures match with FP_ENU0)
            # This will be called when GPS data is available
            # DO NOT set map origin from charge_point - Map Origin != charge position!
            # Synchronize origin with direct coordinate transformer (for HomeReturnController and distance calculations)
            self._coord_transformer.set_origin(
                self._charge_position[0],
                self._charge_position[1],
                0.0  # Altitude
            )
            # Transition from UNINITIALIZED to MANUAL when charge position is set
            if hasattr(self, '_robot_state_machine'):
                if self._robot_state_machine.get_state() == RobotState.UNINITIALIZED:
                    context = self._create_state_context()
                    # Transition to MANUAL state (pure condition check, actions in entry callback)
                    self._robot_state_machine.transition_to(RobotState.MANUAL, context)
        else:
            self._charge_position = None

    def _on_manual_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering MANUAL state.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be MANUAL)
        """
        if previous_state == RobotState.UNINITIALIZED:
            self.get_logger().info("Charge position set - transitioned to MANUAL state")
        else:
            self.get_logger().info("Entered MANUAL state")
        
        # Reset need_charge flag when entering manual mode
        # This ensures that charging requests are cleared when user takes manual control
        if self._charging_requested:
            self._charging_requested = False
            charging_request_msg = Bool()
            charging_request_msg.data = False
            self._charging_requested_pub.publish(charging_request_msg)
            self.get_logger().info("Cleared charging request (need_charge) when entering MANUAL mode")
    
    def _check_startup_charge_position_recovery(self):
        """Check if robot is at charge position on startup and transition to IDLE.
        
        This is critical for recovery after container restart. If the robot was
        physically at the charge position when the system restarted, we need to
        recognize this and transition through the proper state chain:
        MANUAL -> IDLE -> DOCKED -> CHARGING (or UNDOCKING if schedule active)
        
        This prevents the robot from trying to navigate while still docked, which
        can cause damage or unpredictable behavior.
        """
        # Mark as done even if we fail - we only try once
        self._startup_recovery_done = True
        
        # Need charge position configured
        if not self._charge_position:
            self.get_logger().info("Startup recovery: No charge position configured, skipping")
            return
        
        # Need GPS position
        if not self._last_gps:
            self.get_logger().info("Startup recovery: No GPS position yet, skipping")
            return
        
        # Check RTK status
        is_rtk_ready, rtk_reason = self._rtk_monitor.is_rtk_ready_for_docking()
        is_fusion_ok, fusion_reason = self._rtk_monitor.is_fusion_initialized()
        rtk_fix = is_rtk_ready and is_fusion_ok
        
        if not rtk_fix:
            self.get_logger().info(
                f"Startup recovery: RTK not ready yet ({rtk_reason}, {fusion_reason}), "
                "will retry in control loop"
            )
            # Reset flag to retry in control loop
            self._startup_recovery_done = False
            return
        
        # Compute distance to charge position for logging
        robot_pos = (self._last_gps[0], self._last_gps[1], self._last_gps[2])
        charge_pos = (self._charge_position[0], self._charge_position[1], 0.0)
        distance_to_charge = self._coord_transformer.compute_distance(robot_pos, charge_pos)
        
        self.get_logger().info(
            f"Startup recovery: GPS={self._last_gps[0]:.7f},{self._last_gps[1]:.7f}, "
            f"Charge={self._charge_position[0]:.7f},{self._charge_position[1]:.7f}, "
            f"Distance={distance_to_charge:.3f}m"
        )
        
        # Check if at charge position - use 0.5m tolerance for startup
        # This is more lenient than the normal 5cm tolerance
        startup_tolerance = 0.5
        is_at_charge = distance_to_charge <= startup_tolerance
        
        self.get_logger().info(
            f"Startup recovery: distance={distance_to_charge:.3f}m, tolerance={startup_tolerance}m, "
            f"at_charge={is_at_charge}, rtk_fix={rtk_fix}"
        )
        
        if is_at_charge:
            current_state = self._robot_state_machine.get_state()
            
            # Set flag to preserve charge position detection until DOCKED/CHARGING transition completes
            self._startup_at_charge_pos = True
            self.get_logger().info(
                f"🔌 Startup recovery: Set _startup_at_charge_pos=True (distance={distance_to_charge:.3f}m)"
            )
            
            # Only transition if we're in MANUAL or UNINITIALIZED
            if current_state in [RobotState.MANUAL, RobotState.UNINITIALIZED]:
                self.get_logger().info(
                    f"🔌 Startup recovery: Robot is at charge position ({distance_to_charge:.3f}m) - "
                    f"transitioning to IDLE (was in {current_state.name}). "
                    "IDLE will handle DOCKED/CHARGING transitions."
                )
                
                # Create context with correct charge position state
                context = self._create_state_context()
                context.is_at_charge_pos = True
                context.rtk_fix = True
                
                # Transition to IDLE - IDLE's determine_next_state will handle DOCKED
                success = self._robot_state_machine.transition_to(RobotState.IDLE, context)
                
                if success:
                    self.get_logger().info("Startup recovery: Successfully transitioned to IDLE")
                else:
                    self.get_logger().warn("Startup recovery: Failed to transition to IDLE")
            else:
                self.get_logger().info(
                    f"Startup recovery: Robot at charge position but already in {current_state.name}, "
                    "no transition needed"
                )
        else:
            self.get_logger().info(
                f"Startup recovery: Robot is NOT at charge position "
                f"(distance={distance_to_charge:.3f}m > {startup_tolerance}m), staying in MANUAL"
            )

    def _on_idle_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering IDLE state - queries scheduler for next action.
        
        This implements the service-based schedule query:
        1. Check if autonomous operation is enabled
        2. Query scheduler for active schedule/route
        3. Handle response (start route, undock, dock, etc.)
        
        The scheduler service decides what action to take based on:
        - Active schedules in time window
        - Robot position (at home, at charge position)
        - Battery status
        
        If no active schedule, the recovery timer is used as fallback.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be IDLE)
        """
        # Cancel any existing recovery timer
        self._cancel_recovery_timer()
        
        self.get_logger().info("Entered IDLE state")
        
        # CRITICAL: Stop robot immediately when entering IDLE
        # This prevents the robot from continuing to drive with stale commands
        # from the previous state (e.g., after route completion)
        self._publish_drive_command(0, 0)
        
        # Safety check: Stop waypoint follower if still active
        # NOTE: This should normally NOT trigger because:
        # - On normal route completion, WaypointFollower sets _mission_active=False BEFORE calling on_route_completed()
        # - This means is_active() returns False when we enter IDLE after a route ends
        # This check is only a safety net for edge cases (e.g., race conditions)
        # It does NOT affect route repetitions - those are handled by the scheduler
        # which sends the next route after we query it in IDLE state
        if self._waypoint_follower.is_active():
            self.get_logger().warn("IDLE: WaypointFollower still active - stopping (this may indicate a bug)")
            self._waypoint_follower.stop()
        
        # Check if autonomous operation is enabled - required for any autonomous action
        context = self._create_state_context()
        if not context.autonomous_operation_enabled:
            self.get_logger().info("IDLE: Autonomous operation disabled, waiting for manual activation")
            return
        
        # Check if we already have an active route - if so, no query needed
        if context.route_active:
            self.get_logger().info("IDLE: Route already active, no query needed")
            return
        
        # Check sensor information
        if not self._last_gps or not self._last_orientation:
            self.get_logger().warn("IDLE: Sensor information not available - waiting for GPS/orientation")
            return
        
        # Query scheduler for next action
        # This is the primary way to get schedules in the new architecture
        self.get_logger().info("IDLE: Querying scheduler for next action...")
        self._query_schedule_action_async()
        
        # Also schedule a delayed recovery as fallback if scheduler doesn't respond
        # or if there's no active schedule but we want local recovery
        # Recovery is NOT started if robot is at charge position (context.is_at_charge_pos blocks it)
        # Additional safety check: Don't start recovery if robot is too close to charge position
        # (e.g., during or shortly after undocking, when is_at_charge_pos might be False but still too close)
        # to prevent collision with charging station/hut
        if self._robot_position and not context.is_at_charge_pos:
            if self._is_too_close_to_charge_position():
                self.get_logger().info(
                    "IDLE: Skipping recovery - robot is too close to charge position "
                    "(safety check to prevent collision with charging area)"
                )
            else:
                self.get_logger().debug(f"IDLE: Scheduling fallback recovery in {self._recovery_delay_seconds}s...")
                self._recovery_timer = self.create_timer(
                    self._recovery_delay_seconds,
                    self._execute_delayed_recovery_once
                )
        elif context.is_at_charge_pos:
            # Robot is at charge position - recovery should not run (state machine should handle DOCKED/CHARGING)
            self.get_logger().debug("IDLE: Robot at charge position - recovery not started (will transition to DOCKED/CHARGING)")

    def _cancel_recovery_timer(self):
        """Cancel any pending recovery timer."""
        if self._recovery_timer is not None:
            self._recovery_timer.cancel()
            self._recovery_timer = None
    
    def _execute_delayed_recovery_once(self):
        """Execute the delayed recovery logic (called by timer, runs once)."""
        # Cancel the timer immediately so it only runs once
        self._cancel_recovery_timer()
        
        # Re-check conditions before executing recovery
        context = self._create_state_context()
        
        # If a route is now active (scheduler sent one), don't run recovery
        if context.route_active:
            self.get_logger().info("Recovery: Route became active, skipping recovery")
            return
        
        # If we're no longer in IDLE state, don't run recovery
        current_robot_state = self._robot_state_machine.get_state()
        if current_robot_state != RobotState.IDLE:
            self.get_logger().info(f"Recovery: No longer in IDLE state (now {current_robot_state.name}), skipping recovery")
            return
        
        # If autonomous operation was disabled, don't run recovery
        if not context.autonomous_operation_enabled:
            self.get_logger().info("Recovery: Autonomous operation disabled, skipping recovery")
            return
        
        # Safety check: Don't run recovery if robot is at/near charge position or home position
        # This prevents recovery from starting routes when robot is already at base
        # Use both is_at_* and is_near_* to handle GPS jitter (consistent with IdleState)
        if context.is_at_charge_pos or context.is_at_home_pos or context.is_near_charge_pos or context.is_near_home_pos:
            self.get_logger().info(
                f"Recovery: Robot is at/near charge/home position (at_charge={context.is_at_charge_pos}, "
                f"near_charge={context.is_near_charge_pos}, at_home={context.is_at_home_pos}, "
                f"near_home={context.is_near_home_pos}) - skipping recovery to prevent unnecessary navigation"
            )
            return
        
        # Safety check: Don't run recovery if robot is too close to charge position
        # (e.g., during or shortly after undocking) to prevent collision with charging station
        if self._is_too_close_to_charge_position():
            self.get_logger().info(
                "Recovery: Robot is too close to charge position (safety check) - skipping recovery "
                "to prevent collision with charging area"
            )
            return
        
        # Load max_route_distance from settings
        max_distance = self._max_route_distance
        
        self.get_logger().info(f"Recovery: Searching for routes within {max_distance}m...")
        
        try:
            # Use HomeReturnController to find closest route
            result = self._home_return.find_closest_route_for_home_return(
                self._robot_position,
                max_distance=max_distance,
                use_perpendicular_entry=True
            )
            
            if result:
                closest_route, distance, geopath = result
                self.get_logger().info(f"Recovery: Found closest route '{closest_route}' at {distance:.2f}m")
                
                # Validate distance is within configured maximum
                if distance <= max_distance:
                    self.get_logger().info(f"Recovery: Route '{closest_route}' is within max distance ({distance:.2f}m <= {max_distance}m) - starting navigation")
                    
                    # Start navigation directly - we're in IDLE so no undocking needed
                    self._start_navigation_directly(geopath, closest_route)
                else:
                    self.get_logger().warn(f"Recovery: Route '{closest_route}' at {distance:.2f}m exceeds max distance ({max_distance}m) - skipping")
            else:
                self.get_logger().info(f"Recovery: No route found within {max_distance}m - robot will remain in IDLE state")
        except Exception as e:
            self.get_logger().error(f"Recovery: Error finding closest route: {e}")

    def _on_navigating_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering NAVIGATING state.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be NAVIGATING)
        """
        self.get_logger().info("Entered NAVIGATING state")
        # Route tracking and waypoint setup is handled in _start_navigation_directly before transition
        # No additional actions needed here as route is already started

    def _on_docking_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering DOCKING state.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be DOCKING)
        """
        self.get_logger().info("Entered DOCKING state")
        self._publish_charging_status("positioning", "Getting into position behind charge point")
        # Clear any active routes when entering DOCKING state
        # Routes should not be active during docking
        if self._current_route is not None:
            self.get_logger().info("Clearing route when entering DOCKING state")
            self._current_route = None
        # Stop waypoint follower if active
        if self._waypoint_follower.is_active():
            self._waypoint_follower.stop()
        # Route tracking is handled in _dock_callback before transition

    def _on_undocking_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering UNDOCKING state.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be UNDOCKING)
        """
        self.get_logger().info(f"Entered UNDOCKING state (from {previous_state.name})")
        # Publish undocking status IMMEDIATELY to prevent scheduler from re-enabling charging
        # This must happen before starting undocking to avoid race conditions
        self._publish_charging_status("undocking", "Starting undocking - moving backwards from charge position")
        
        # UndockingState.on_update() will handle starting the undocking procedure
        # when it's called for the first time with the required kwargs

    def _on_docked_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering DOCKED state.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be DOCKED)
        """
        self.get_logger().info("Entered DOCKED state")
        # Stop robot when docked
        self._publish_drive_command(0, 0)
        # Clear any leftover routes to prevent premature undocking
        # Routes should only be active when explicitly started by scheduler
        if self._current_route is not None:
            self.get_logger().info("Clearing leftover route when entering DOCKED state")
            self._current_route = None
        # Stop waypoint follower if active
        if self._waypoint_follower.is_active():
            self._waypoint_follower.stop()
        
        # Query scheduler to check if there's an active schedule that should start
        # This allows scheduled routes to trigger undocking
        if self._autonomous_operation_enabled:
            self.get_logger().info("DOCKED: Querying scheduler for active schedules...")
            self._query_schedule_action_async()

    def _on_charging_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering CHARGING state.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be CHARGING)
        """
        self.get_logger().info("Entered CHARGING state")
        # Clear any leftover routes to prevent premature undocking
        # Routes should only be active when explicitly started by scheduler
        if self._current_route is not None:
            self.get_logger().info("Clearing leftover route when entering CHARGING state")
            self._current_route = None
        # Stop waypoint follower if active
        if self._waypoint_follower.is_active():
            self._waypoint_follower.stop()
        
        # Query scheduler to check if there's an active schedule that should start
        # This allows scheduled routes to trigger undocking even while charging
        if self._autonomous_operation_enabled:
            self.get_logger().info("CHARGING: Querying scheduler for active schedules...")
            self._query_schedule_action_async()

    def _on_undocked_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering UNDOCKED state.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be UNDOCKED)
        """
        self.get_logger().info("Entered UNDOCKED state")
        
        # If there's a pending geopath, start it DIRECTLY now
        if self._pending_geopath is not None:
            pending = self._pending_geopath
            self._pending_geopath = None
            self.get_logger().info("Undocking complete - starting pending route directly")
            # Ensure schedule flag is set when starting from pending route (from active schedule)
            self._has_active_schedule = True
            # Start navigation directly - we already decided to start this route before undocking
            self._start_navigation_directly(pending, "pending_route")
        else:
            # Query scheduler for next action after undocking
            self._query_schedule_action_async()
    
    def _query_schedule_action_async(self):
        """Query scheduler for next action asynchronously.
        
        This is called from state entry callbacks and periodic polling.
        Only queries if:
        1. Service client is available
        2. Not already waiting for a response
        3. In an allowed state for queries
        """
        # Check if we're already waiting for a response
        if self._schedule_query_pending:
            self.get_logger().debug("Schedule query already pending - skipping")
            return
        
        # Check if we're in an allowed state
        current_state = self._robot_state_machine.get_state()
        if current_state not in self._allowed_schedule_query_states:
            self.get_logger().debug(f"Not querying scheduler - state {current_state.name} not allowed")
            return
        
        # Check if service is available
        if not self._schedule_action_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().debug("Schedule action service not available")
            return
        
        # Check if we already have an active route - control loop decides this, not scheduler
        if self._waypoint_follower.is_active():
            self.get_logger().debug("Not querying scheduler - route already active")
            return
        
        # Create request - minimal, scheduler only needs command_id for logging
        request = GetScheduleAction.Request()
        request.command_id = f"ctrl_{time()}"
        
        self.get_logger().info(f"Querying scheduler for active schedule...")
        
        # Send async request
        self._schedule_query_pending = True
        self._last_schedule_query_time = time()
        future = self._schedule_action_client.call_async(request)
        future.add_done_callback(self._handle_schedule_action_response)
    
    def _handle_schedule_action_response(self, future):
        """Handle response from scheduler service.
        
        The scheduler ONLY provides schedule/route DATA.
        This method makes ALL decisions about what action to take based on:
        - Current robot state
        - Robot position (at charge pos, at home pos)
        - Whether a schedule is active and has route data
        
        Args:
            future: Future containing the service response
        """
        self._schedule_query_pending = False
        
        try:
            response = future.result()
            
            if not response.success:
                self.get_logger().warn(f"Schedule query failed: {response.message}")
                return
            
            self.get_logger().info(
                f"Schedule data received: has_schedule={response.has_active_schedule}, "
                f"route={response.route_name}, waypoints={len(response.waypoints)}, "
                f"msg={response.message}"
            )
            
            # No active schedule - clear flag and return
            if not response.has_active_schedule:
                self.get_logger().debug(f"No active schedule: {response.message}")
                self._has_active_schedule = False
                return
            
            # Have an active schedule - set flag and let CONTROL LOOP decide what to do
            self._has_active_schedule = True
            current_state = self._robot_state_machine.get_state()
            is_at_charge = self._is_at_charge_position()
            is_at_home = self._is_at_home_position()
            has_route_data = response.waypoints and len(response.waypoints) > 0
            
            self.get_logger().info(
                f"Decision context: state={current_state.name}, at_charge={is_at_charge}, "
                f"at_home={is_at_home}, has_route_data={has_route_data}"
            )
            
            # Decision logic - CONTROL LOOP decides!
            
            # Case 1: Robot is DOCKED or CHARGING - need to undock first
            if current_state in [RobotState.DOCKED, RobotState.CHARGING]:
                if has_route_data:
                    self.get_logger().info("Schedule active while docked - triggering undocking")
                    self._trigger_undocking_for_schedule(response)
                else:
                    self.get_logger().warn("Schedule active but no route data - waiting")
                return
            
            # Case 2: Robot is UNDOCKED - check if we can start route
            if current_state == RobotState.UNDOCKED:
                if has_route_data:
                    self.get_logger().info("Undocked with route data - starting route")
                    self._start_route_from_schedule_response(response)
                else:
                    self.get_logger().warn("Undocked but no route data available")
                return
            
            # Case 3: Robot is IDLE - decide based on position
            if current_state == RobotState.IDLE:
                if is_at_charge:
                    # At charge position but IDLE (not DOCKED) - should undock
                    if has_route_data:
                        self.get_logger().info("IDLE at charge position - triggering undocking for schedule")
                        self._trigger_undocking_for_schedule(response)
                    return
                
                if is_at_home:
                    # At home position - check if too close to charge position first
                    # If robot is still within 1m of charge position (e.g., during/after undocking),
                    # trigger undocking instead of starting navigation to avoid collision
                    if self._is_too_close_to_charge_position():
                        if has_route_data:
                            self.get_logger().info("IDLE at home but too close to charge position - triggering undocking for schedule")
                            self._trigger_undocking_for_schedule(response)
                        else:
                            self.get_logger().warn("At home but too close to charge position and no route data available")
                        return
                    
                    # At home position and safe distance from charge - can start route directly
                    if has_route_data:
                        self.get_logger().info("IDLE at home - starting scheduled route")
                        self._start_route_from_schedule_response(response)
                    else:
                        self.get_logger().warn("At home but no route data available")
                    return
                
                # Not at charge and not at home - try to start anyway (recovery)
                if has_route_data:
                    self.get_logger().info("IDLE not at home/charge - starting route for recovery")
                    self._start_route_from_schedule_response(response)
                else:
                    self.get_logger().warn("IDLE with schedule but no route data and not at base")
                return
            
            # Other states - log but don't act
            self.get_logger().debug(f"Schedule active but in state {current_state.name} - no action taken")
        
        except Exception as e:
            self.get_logger().error(f"Error handling schedule response: {e}")
    
    def _start_route_from_schedule_response(self, response):
        """Start a route DIRECTLY from scheduler response.
        
        This method starts navigation directly via _start_navigation_directly.
        The control loop has already decided that we should start - no additional
        checks needed.
        
        Args:
            response: GetScheduleAction.Response with route data
        """
        if not response.waypoints or len(response.waypoints) == 0:
            self.get_logger().warn("Cannot start route - no waypoints in response")
            return
        
        # Create GeoPath from response
        geopath = GeoPath()
        geopath.waypoints = list(response.waypoints)
        geopath.mode = response.geopath_mode
        
        self.get_logger().info(
            f"Starting route '{response.route_name}' from scheduler "
            f"({len(geopath.waypoints)} waypoints, mode={geopath.mode})"
        )
        
        # Start navigation DIRECTLY - control loop already decided this is safe
        self._start_navigation_directly(geopath, response.route_name)
    
    def _start_navigation_directly(self, geopath: GeoPath, route_name: str = ""):
        """Start navigation DIRECTLY without any state checks.
        
        This is the central method for starting navigation. It assumes the control
        loop has already verified that starting navigation is safe (not docked, etc.)
        
        Args:
            geopath: GeoPath with waypoints to follow
            route_name: Optional route name for logging
        """
        if len(geopath.waypoints) == 0:
            self.get_logger().warn("Cannot start navigation - empty GeoPath!")
            return
        
        # Cancel any pending recovery timer
        self._cancel_recovery_timer()
        
        # Ensure map origin is set before converting waypoints
        # This is critical for startup when scheduler tries to start before GPS callback sets origin
        if self._transform_manager.get_map_origin() is None:
            if self._last_gps is not None:
                self.get_logger().info(
                    f"Map origin not set yet, attempting to set from TF and GPS before starting route '{route_name}'"
                )
                if self._transform_manager.auto_set_map_origin_from_tf_and_gps(self._last_gps):
                    self.get_logger().info("Map origin successfully set from TF and GPS")
                else:
                    self.get_logger().error(
                        f"Cannot start route '{route_name}': Map origin not set and failed to set from TF and GPS. "
                        "Waiting for GPS callback to set map origin."
                    )
                    return
            else:
                self.get_logger().error(
                    f"Cannot start route '{route_name}': Map origin not set and GPS data not available. "
                    "Waiting for GPS fix."
                )
                return
        
        # Convert waypoints to map frame (using map origin from TransformManager)
        # All waypoints should be relative to the map origin (charge position), not the first waypoint
        waypoints_map = []
        for wp in geopath.waypoints:
            x, y, z = self._transform_manager.gps_to_map(wp.x, wp.y, wp.z)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            waypoints_map.append(pose)
        
        # Set mode and waypoints
        mode_map = {
            GeoPath.ONCE: WaypointMode.ONCE,
            GeoPath.LOOP: WaypointMode.LOOP,
            GeoPath.PING_PONG: WaypointMode.PING_PONG
        }
        self._waypoint_follower.set_mode(mode_map.get(geopath.mode, WaypointMode.ONCE))
        self._waypoint_follower.set_waypoints(waypoints_map)
        
        # Always start from waypoint 0 for fresh route starts (scheduler
        # repetitions, Run-Now).  find_nearest_waypoint caused instant route
        # completion when the robot was already at the last waypoint — all
        # remaining repetitions were consumed in < 1 s without movement.
        self._waypoint_follower.start(start_index=0)
        
        # Verify that waypoint follower is actually active after start()
        # This is critical to prevent immediate transition to RETURNING_TO_HOME
        if not self._waypoint_follower.is_active():
            self.get_logger().error(
                f"CRITICAL: Waypoint follower not active after start()! "
                f"This will cause immediate transition to RETURNING_TO_HOME. "
                f"Waypoints: {len(waypoints_map)}, route='{route_name}'"
            )
            return  # Don't transition to NAVIGATING if route isn't actually active
        
        self.get_logger().info(f"Navigation started: {len(waypoints_map)} waypoints, route='{route_name}'")
        
        # Track current route (for monitoring only)
        try:
            waypoints_list = []
            for wp in geopath.waypoints:
                waypoints_list.append({'lat': float(wp.x), 'lon': float(wp.y), 'alt': float(wp.z)})
            
            start_index = 0  # consistent with start(start_index=0) above

            # Convert mode to string for JSON serialization
            mode_map = {
                GeoPath.ONCE: 'once',
                GeoPath.LOOP: 'loop',
                GeoPath.PING_PONG: 'ping_pong',
                GeoPath.STOP: 'none'
            }
            mode_string = mode_map.get(geopath.mode, 'once')
            
            self._current_route = {
                'source': 'schedule',
                'route_name': route_name,
                'waypoints': waypoints_list,
                'mode': mode_string,
                'current_waypoint_index': int(start_index),
                'started_at': time()
            }
        except Exception:
            pass
        
        # Transition to NAVIGATING state
        # Create context AFTER verifying waypoint follower is active
        # This ensures route_active will be True in the context
        context = self._create_state_context()
        
        # Double-check that route_active is True in context before transitioning
        # This prevents the race condition where context shows route_active=False
        if not context.route_active:
            self.get_logger().error(
                f"CRITICAL: Context shows route_active=False after starting navigation! "
                f"waypoint_follower.is_active()={self._waypoint_follower.is_active()}. "
                f"This will cause immediate transition to RETURNING_TO_HOME. "
                f"Not transitioning to NAVIGATING to prevent loop."
            )
            return  # Don't transition if context shows route as inactive
        
        self._robot_state_machine.transition_to(RobotState.NAVIGATING, context)
    
    def _trigger_undocking_for_schedule(self, response):
        """Trigger undocking before starting a scheduled route.

        Args:
            response: GetScheduleAction.Response (may contain route to start after undock)
        """
        # User intent override: an explicit route start means the user wants
        # the robot off the dock. The sticky self._charging_requested flag
        # (which maps to need_charge=True in the state context and blocks
        # CHARGING -> UNDOCKING) is cleared here so the transition is allowed.
        # The battery cutoff is enforced upstream by the scheduler, so a route
        # actually arriving here implies the battery is sufficient.
        if self._charging_requested:
            self._charging_requested = False
            try:
                clear_msg = Bool()
                clear_msg.data = False
                self._charging_requested_pub.publish(clear_msg)
            except Exception:
                pass
            self.get_logger().info(
                "Cleared charging request (need_charge) on explicit route start"
            )
            try:
                m = String()
                m.data = "\u23ed\ufe0f Lade-Anforderung gel\u00f6scht. Roboter darf abdocken."
                self._log_info_pub.publish(m)
            except Exception:
                pass

        # Store the route to start after undocking if provided
        if response.waypoints and len(response.waypoints) > 0:
            geopath = GeoPath()
            geopath.waypoints = list(response.waypoints)
            geopath.mode = response.geopath_mode
            self._pending_geopath = geopath
            self.get_logger().info(f"Stored pending route '{response.route_name}' for after undocking")
        
        # Trigger undocking
        current_state = self._robot_state_machine.get_state()
        if current_state in [RobotState.DOCKED, RobotState.CHARGING, RobotState.IDLE]:
            context = self._create_state_context()
            # For IDLE state, if robot is too close to charge position, treat it as at charge position
            # This allows undocking to start even if robot is not exactly at charge position
            # but still within the safety distance (< 1m)
            if current_state == RobotState.IDLE:
                if self._is_too_close_to_charge_position():
                    context.is_at_charge_pos = True
                    self.get_logger().info("IDLE state: Robot too close to charge position, treating as at charge for undocking")
                else:
                    # If not too close, check if actually at charge position
                    context.is_at_charge_pos = self._is_at_charge_position()
            else:
                # For DOCKED/CHARGING, always at charge position
                context.is_at_charge_pos = True
            context.schedule_active = True  # Indicate schedule wants to start
            success = self._robot_state_machine.transition_to(RobotState.UNDOCKING, context)
            if success:
                # Publish status to notify external systems of undocking state change
                self._publish_charging_status("undocking", "Undocking for scheduled route")
            else:
                self.get_logger().error("Failed to transition to UNDOCKING for schedule")
    
    def _start_return_home_from_response(self, response):
        """Start return home navigation from scheduler response.
        
        Args:
            response: GetScheduleAction.Response with optional reverse route
        """
        # If response contains waypoints, use them for return route
        if response.waypoints and len(response.waypoints) > 0:
            geopath = GeoPath()
            geopath.waypoints = list(response.waypoints)
            geopath.mode = GeoPath.ONCE
            
            self.get_logger().info(f"Starting return home with {len(geopath.waypoints)} waypoints")
            
            # Transition to RETURNING_TO_HOME state
            context = self._create_state_context()
            success = self._robot_state_machine.transition_to(RobotState.RETURNING_TO_HOME, context)
            if success:
                # Set up waypoints for return home
                self._setup_waypoints_from_geopath(geopath)
            else:
                self.get_logger().error("Failed to transition to RETURNING_TO_HOME")
        else:
            # No route provided - use state machine's own return home logic
            self.get_logger().info("Return home requested but no route provided - using recovery logic")
            context = self._create_state_context()
            self._robot_state_machine.transition_to(RobotState.RETURNING_TO_HOME, context)
    
    def _trigger_docking(self):
        """Trigger docking procedure.
        
        Called when scheduler indicates robot should dock.
        """
        current_state = self._robot_state_machine.get_state()
        
        # Check if we're in a state that can transition to DOCKING
        if current_state in [RobotState.IDLE, RobotState.UNDOCKED]:
            context = self._create_state_context()
            success = self._robot_state_machine.transition_to(RobotState.DOCKING, context)
            if success:
                self._publish_charging_status("positioning", "Starting docking procedure")
            else:
                self.get_logger().error("Failed to transition to DOCKING")
        else:
            self.get_logger().warn(f"Cannot dock from state {current_state.name}")
    
    def _check_periodic_schedule_query(self):
        """Check if periodic schedule query should be performed.
        
        Called from control loop to periodically query scheduler.
        """
        current_time = time()
        
        # Check if enough time has passed since last query
        if current_time - self._last_schedule_query_time < self._schedule_query_interval:
            return
        
        # Check if autonomous operation is enabled
        if not self._autonomous_operation_enabled:
            return
        
        # Query scheduler
        self._query_schedule_action_async()
    
    def _setup_waypoints_from_geopath(self, geopath: GeoPath) -> bool:
        """Helper method to set up waypoint follower from GeoPath message.
        
        Args:
            geopath: GeoPath message with waypoints
            
        Returns:
            True if waypoints were set up successfully, False otherwise
        """
        if len(geopath.waypoints) == 0:
            self.get_logger().warn("Cannot set up waypoints: empty GeoPath!")
            return False
        
        # Ensure map origin is set before converting waypoints
        if self._transform_manager.get_map_origin() is None:
            if self._last_gps is not None:
                self.get_logger().info(
                    "Map origin not set yet, attempting to set from TF and GPS before setting up waypoints"
                )
                if self._transform_manager.auto_set_map_origin_from_tf_and_gps(self._last_gps):
                    self.get_logger().info("Map origin successfully set from TF and GPS")
                else:
                    self.get_logger().error(
                        "Cannot set up waypoints: Map origin not set and failed to set from TF and GPS. "
                        "Waiting for GPS callback to set map origin."
                    )
                    return False
            else:
                self.get_logger().error(
                    "Cannot set up waypoints: Map origin not set and GPS data not available. "
                    "Waiting for GPS fix."
                )
                return False
        
        # Convert waypoints to map frame (using map origin from TransformManager)
        # All waypoints should be relative to the map origin (charge position), not the first waypoint
        waypoints_map = []
        for wp in geopath.waypoints:
            x, y, z = self._transform_manager.gps_to_map(wp.x, wp.y, wp.z)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            waypoints_map.append(pose)
        
        # Set mode and waypoints
        mode_map = {
            GeoPath.ONCE: WaypointMode.ONCE,
            GeoPath.LOOP: WaypointMode.LOOP,
            GeoPath.PING_PONG: WaypointMode.PING_PONG
        }
        self._waypoint_follower.set_mode(mode_map.get(geopath.mode, WaypointMode.ONCE))
        self._waypoint_follower.set_waypoints(waypoints_map)
        
        # Start mission
        if self._last_gps:
            current_pose_map = self._get_current_pose_map()
            if current_pose_map:
                nearest_idx = self._waypoint_follower.find_nearest_waypoint(current_pose_map)
                self._waypoint_follower.start(start_index=nearest_idx)
            else:
                self._waypoint_follower.start()
        else:
            self._waypoint_follower.start()
        
        self.get_logger().info(f"Waypoints set up: {len(waypoints_map)} waypoints, mode={geopath.mode}")
        
        # Track current route (for monitoring only, not persistence)
        try:
            waypoints_list = []
            for wp in geopath.waypoints:
                waypoints_list.append({'lat': float(wp.x), 'lon': float(wp.y), 'alt': float(wp.z)})
            
            start_index = 0
            if self._last_gps:
                try:
                    current_pose_map = self._get_current_pose_map()
                    if current_pose_map:
                        nearest_idx = self._waypoint_follower.find_nearest_waypoint(current_pose_map)
                        start_index = nearest_idx
                except Exception:
                    start_index = 0
            
            self._current_route = {
                'source': 'geopath',
                'waypoints': waypoints_list,
                'current_waypoint_index': int(start_index),
                'started_at': time()
            }
        except Exception:
            pass
        
        return True
    
    def _on_returning_to_home_state_entry(self, previous_state: RobotState, current_state: RobotState):
        """Callback when entering RETURNING_TO_HOME state.
        
        Sets up the reverse route for returning to home.
        
        Args:
            previous_state: Previous state before transition
            current_state: Current state (should be RETURNING_TO_HOME)
        """
        self.get_logger().info("Entered RETURNING_TO_HOME state - setting up reverse route")
        
        # Safety check: If robot is already at/near home position, skip route setup
        # The state machine's determine_next_state will handle the transition to DOCKING
        # This prevents unnecessary route searching when robot is already at destination
        at_home = self._is_at_home_position()
        near_home = self._is_at_home_position(tolerance=0.4)  # Same as is_near_home_pos
        if at_home or near_home:
            self.get_logger().info(
                f"RETURNING_TO_HOME: Robot already at/near home position "
                f"(at_home={at_home}, near_home={near_home}) - skipping route setup, "
                f"state machine will transition to DOCKING"
            )
            return
        
        # Get current robot position
        if not self._robot_position:
            self.get_logger().warn("RETURNING_TO_HOME: Robot position not available - cannot create reverse route")
            return
        
        # Try to find closest route and create reverse path
        try:
            # Load max_route_distance from settings
            max_distance = self._max_route_distance
            
            # Use HomeReturnController to find closest route
            result = self._home_return.find_closest_route_for_home_return(
                self._robot_position,
                max_distance=max_distance,
                use_perpendicular_entry=True
            )
            
            if result:
                closest_route, distance, reverse_geopath = result
                self.get_logger().info(
                    f"RETURNING_TO_HOME: Found closest route '{closest_route}' at {distance:.2f}m"
                )
                
                # Validate distance is within configured maximum
                if distance <= max_distance:
                    # Set up waypoints from reverse geopath
                    if self._setup_waypoints_from_geopath(reverse_geopath):
                        self.get_logger().info(
                            f"RETURNING_TO_HOME: Reverse route '{closest_route}' set up successfully"
                        )
                    else:
                        self.get_logger().warn("RETURNING_TO_HOME: Failed to set up reverse route waypoints")
                else:
                    self.get_logger().warn(
                        f"RETURNING_TO_HOME: Route '{closest_route}' at {distance:.2f}m exceeds "
                        f"max distance ({max_distance}m) - cannot return home"
                    )
            else:
                self.get_logger().warn(
                    f"RETURNING_TO_HOME: No route found within {max_distance}m - cannot return home"
                )
        except Exception as e:
            self.get_logger().error(f"RETURNING_TO_HOME: Error setting up reverse route: {e}", exc_info=True)

    def _resolve_charge_target(self, msg: Optional[Point]) -> Optional[Tuple[float, float, float]]:
        """Determine the docking target pose."""
        if msg:
            has_coords = (math.isfinite(msg.x) and math.isfinite(msg.y) and
                          (abs(msg.x) > 1e-6 or abs(msg.y) > 1e-6))
            if has_coords:
                yaw = msg.z if math.isfinite(msg.z) else 0.0
                return (msg.x, msg.y, yaw)
        return self._charge_position

    def _get_home_distance(self) -> Optional[float]:
        """Compute current distance to home."""
        if not self._home_position or not self._last_gps:
            return None
        robot_pos = (self._last_gps[0], self._last_gps[1], self._last_gps[2])
        home_pos = (self._home_position[0], self._home_position[1], 0.0)
        return self._coord_transformer.compute_distance(robot_pos, home_pos)

    def _is_near_home_for_docking(self) -> Tuple[bool, Optional[float]]:
        """Check whether the robot is within docking radius of home."""
        distance = self._get_home_distance()
        if distance is None:
            return False, None
        return distance <= self._dock_home_radius, distance

    def _dock_callback(self, msg: Point):
        """Handle docking command.
        
        Args:
            msg: Point message where x=latitude, y=longitude, z=yaw (in degrees)
        """
        self.get_logger().info(f"Received docking command: lat={msg.x}, lon={msg.y}, yaw={msg.z}")
        
        # Stop waypoint following
        self._waypoint_follower.stop()
        self._load_settings()
        
        if not self._last_gps:
            self.get_logger().error("No GPS position available for docking!")
            self._publish_charging_status("error", "No GPS position available")
            return
        
        target_pose = self._resolve_charge_target(msg)
        if not target_pose:
            self.get_logger().error("Docking command ignored - no charge point configured and no coordinates provided")
            self._publish_charging_status("error", "No charge point configured")
            return
        
        near_home, distance = self._is_near_home_for_docking()
        if not near_home:
            if distance is None:
                message = "Cannot verify distance to home - missing GPS or home position"
            else:
                message = f"Too far from home ({distance:.2f} m > {self._dock_home_radius:.1f} m)"
            self.get_logger().warn(f"Dock command blocked: {message}")
            self._publish_charging_status("blocked", message)
            return
        
        # Get origin from coordinate transformer (or use current GPS if not set)
        origin_gps = self._coord_transformer.get_origin()
        if origin_gps == (0.0, 0.0, 0.0) and self._last_gps:
            # Origin not set, use current GPS position
            origin_gps = self._last_gps
        
        # Ensure we have home position
        if not self._home_position:
            self.get_logger().error("Docking command ignored - no home position configured")
            self._publish_charging_status("error", "No home position configured")
            return
        
        # Track docking command (for monitoring only, not persistence) - before transition
        self._current_route = {
            'source': 'dock_command',
            'target': {'lat': target_pose[0], 'lon': target_pose[1], 'yaw': target_pose[2]},
            'started_at': time()
        }
        
        # Transition to DOCKING state (state class will handle docking procedure)
        context = self._create_state_context()
        success = self._robot_state_machine.transition_to(RobotState.DOCKING, context)
        
        if not success:
            self.get_logger().error("Failed to transition to DOCKING state")
            self._publish_charging_status("error", "Failed to start docking - check prerequisites")
            return

    def _charging_requested_callback(self, msg: Bool):
        """Handle charging request from scheduler.
        
        This callback receives the charging request status from the scheduler.
        The State Machine will automatically react to need_charge in the context.
        """
        self._charging_requested = msg.data
        if msg.data:
            self.get_logger().info("Charging requested - State Machine will handle transition")
        else:
            self.get_logger().info("Charging request cleared")
    
    def _charging_relay_callback(self, msg: Bool):
        """Handle charging relay state from ChargingState or Scheduler.
        
        This callback tracks the actual charging relay state so the context
        reflects the true hardware state.
        """
        self._charging_relay_enabled = msg.data
        self.get_logger().debug(f"Charging relay state updated: {msg.data}")
    
    def _undock_callback(self, msg: Bool):
        """Handle undocking command."""
        if not msg.data:
            return
        
        self.get_logger().info("Undocking command received")
        
        # Check RTK status before undocking (required for both GNSS)
        is_ready, reason = self._rtk_monitor.is_rtk_ready_for_docking()
        if not is_ready:
            self.get_logger().warn(f"Undocking abgebrochen: {reason}")
            self._publish_charging_status("error", f"RTK nicht bereit: {reason}")
            return
        
        # Check fusion status (should be green/initialized)
        is_fusion_ok, fusion_reason = self._rtk_monitor.is_fusion_initialized()
        if not is_fusion_ok:
            self.get_logger().warn(f"Undocking abgebrochen: {fusion_reason}")
            self._publish_charging_status("error", f"Fusion nicht initialisiert: {fusion_reason}")
            return
        
        self.get_logger().info(f"RTK and fusion checks passed for undocking: {reason}, {fusion_reason}")
        
        # Check current robot state machine state
        current_robot_state = self._robot_state_machine.get_state()
        self.get_logger().info(f"Current robot state: {current_robot_state.name}")
        
        if not self._last_gps:
            self.get_logger().error("No GPS position available for undocking!")
            self._publish_charging_status("error", "No GPS position available")
            return
        
        self.get_logger().info(f"GPS available: {self._last_gps}")
        
        # Stop waypoint following
        self._waypoint_follower.stop()
        self._load_settings()
        
        if not self._charge_position:
            self.get_logger().error("Undocking command ignored - no charge point configured")
            self._publish_charging_status("error", "No charge point configured")
            return
        
        self.get_logger().info(f"Charge position: {self._charge_position}")
        
        if not self._home_position:
            self.get_logger().error("Undocking command ignored - no home position configured")
            self._publish_charging_status("error", "No home position configured")
            return
        
        self.get_logger().info(f"Home position: {self._home_position}")
        
        # If already undocking, just log and return
        if current_robot_state == RobotState.UNDOCKING:
            self.get_logger().info("Already in UNDOCKING state - undocking in progress")
            self._publish_charging_status("undocking", "Undocking already in progress")
            return
        
        # Clear any active routes when undocking is requested
        # This ensures route_active and schedule_active are False in context
        if self._current_route is not None:
            self.get_logger().info("Clearing route when undocking is requested")
            self._current_route = None
        if self._waypoint_follower.is_active():
            self._waypoint_follower.stop()
        
        # Transition to UNDOCKING state (state class will handle the undocking procedure)
        context = self._create_state_context()
        # If we're currently in DOCKING or DOCKED state, we're at charge position
        # This ensures the transition works even if GPS check hasn't updated
        if current_robot_state in [RobotState.DOCKING, RobotState.DOCKED, RobotState.CHARGING]:
            context.is_at_charge_pos = True
            # Set schedule_active=True to indicate scheduler wants to undock
            # This is needed because route hasn't started yet, but we need to undock first
            context.schedule_active = True
        success = self._robot_state_machine.transition_to(RobotState.UNDOCKING, context)
        
        if success:
            self.get_logger().info("Successfully transitioned to UNDOCKING state")
            self._publish_charging_status("undocking", "Starting undocking")
        else:
            self.get_logger().error("Failed to transition to UNDOCKING state")
            self._publish_charging_status("error", "Failed to start undocking - check prerequisites")
            # Transition to IDLE on failure
            self._robot_state_machine.transition_to(RobotState.IDLE, context)

    def _publish_charging_status(self, state: str, message: str, debug_info: dict = None):
        """Publish charging status with optional debug information.
        
        Args:
            state: Current state (positioning, aligning, approaching, docked, etc.)
            message: Human-readable message
            debug_info: Optional dictionary with detailed debugging information
        """
        msg = String()
        if debug_info:
            # Format: state|message|json_debug_info
            debug_json = json.dumps(debug_info)
            msg.data = f"{state}|{message}|{debug_json}"
        else:
            msg.data = f"{state}|{message}"
        self._charging_status_pub.publish(msg)

    def _obstacle_callback(self, msg: Bool):
        """Handle obstacle detection."""
        self._obstacle_detected = msg.data

    def _occupancy_grid_callback(self, msg: OccupancyGrid):
        """Store the latest OccupancyGrid message and update LiDAR heartbeat for timeout check."""
        self._occupancy_grid = msg
        self._last_lidar_msg_time = time()
        self.get_logger().debug("Received OccupancyGrid data.")
    
    def _obstacle_sectors_callback(self, msg: ObstacleSectors):
        """Store the latest ObstacleSectors message."""
        self._obstacle_sectors_msg = msg
        blocked_count = sum(1 for sector in msg.sectors if sector.blocked)
        self.get_logger().debug(f"Received ObstacleSectors with {len(msg.sectors)} sectors, {blocked_count} blocked")

    def _gps_callback(self, msg: NavSatFix):
        """Store GPS position and handle startup recovery."""
        if (msg.latitude, msg.longitude, msg.altitude) != (0, 0, 0):
            first_gps_fix = self._last_gps is None
            self._last_gps = (msg.latitude, msg.longitude, msg.altitude)
            self._robot_position = (msg.latitude, msg.longitude, msg.altitude)
            
            # Auto-set map origin from TF and GPS on first GPS fix
            # This ensures the origin matches FP_ENU0 from Fixposition Driver
            if first_gps_fix:
                robot_gps = (msg.latitude, msg.longitude, msg.altitude)
                if self._transform_manager.auto_set_map_origin_from_tf_and_gps(robot_gps):
                    self.get_logger().info(
                        "Map origin auto-set from TF and GPS (matches FP_ENU0)"
                    )
                else:
                    self.get_logger().warn(
                        "Failed to auto-set map origin from TF and GPS. Map origin must be set from TF+GPS, not from settings."
                    )
            
            # Startup recovery: check if we need to transition to DOCKED on first GPS fix
            # This is critical for proper recovery after container restart
            if first_gps_fix and not self._startup_recovery_done:
                self._check_startup_charge_position_recovery()

    def _odom_callback(self, msg: Odometry):
        """Store orientation and update speed estimate from odometry."""
        self._last_orientation = msg.pose.pose.orientation

        # Update speed estimator with GPS/fusion velocity
        velocity = msg.twist.twist.linear
        self._speed_estimator.update_from_gps(velocity.x, velocity.y, velocity.z)
        
        # GPS watchdog: check for position jumps > 2m
        current_position_enu = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        if self._last_known_position_enu is not None:
            # Calculate distance from last known position
            dx = current_position_enu[0] - self._last_known_position_enu[0]
            dy = current_position_enu[1] - self._last_known_position_enu[1]
            distance = math.sqrt(dx * dx + dy * dy)
            
            # If jump > 2m, set flag to stop robot
            if distance > 2.0:
                self.get_logger().warn(f"GPS jump detected: {distance:.2f}m from last position - stopping robot until RTK fixed")
                self._gps_jump_detected = True
                # Reset last known position to current to prevent repeated triggers
                self._last_known_position_enu = current_position_enu
            else:
                # Update last known position only if no jump detected
                self._last_known_position_enu = current_position_enu
        else:
            # First position reading - initialize
            self._last_known_position_enu = current_position_enu
    
    def _ypr_callback(self, msg: Vector3Stamped):
        """Cache RTK dual-antenna yaw (rad) from /fixposition/ypr.

        Per fixposition_driver/data_to_ros2.cpp:221, the Vector3 is filled in
        the order "yaw pitch roll" wrt the ENU frame — so:
          vector.x = yaw  (this is what we want)
          vector.y = pitch
          vector.z = roll
        Yaw is in radians, 0 = east, +π/2 = north (ENU convention).
        """
        try:
            self._last_yaw_rad = float(msg.vector.x)
        except Exception as e:
            self.get_logger().error(f"YPR callback error: {e}")

    def _fusion_callback(self, msg: FusionEpoch):
        """Handle fusion status updates for RTK monitoring."""
        try:
            if not getattr(msg, "fpa_odomstatus_avail", False):
                return
            
            status = msg.fpa_odomstatus
            fusion_data = {
                'gnss1_status': int(getattr(status, "gnss1_status", -1)),
                'gnss2_status': int(getattr(status, "gnss2_status", -1)),
                'fusion_status': int(getattr(status, "init_status", -1)),
                'imu_status': int(getattr(status, "imu_status", -1))
            }
            self._rtk_monitor.update_fusion_state(fusion_data)
        except Exception as e:
            self.get_logger().error(f"Fusion callback error: {e}")

    def _set_speed_callback(self, msg: Float32):
        """Handle speed update."""
        self.max_speed = msg.data
        self.get_logger().info(f"Max speed updated to: {self.max_speed}")

    @staticmethod
    def _resolve_obstacle_mode(settings) -> str:
        """Resolve obstacle handling mode from settings.

        Authoritative key is 'obstacle_mode' (off|stop|avoid). For backward
        compatibility an explicit legacy 'enable_obstacle_avoidance: false' maps to
        'off'. Everything else (including legacy true / unset) defaults to 'stop' —
        the protective stop, which is the safe default for a security robot.
        """
        mode = settings.get('obstacle_mode')
        if isinstance(mode, str) and mode.lower() in ('off', 'stop', 'avoid'):
            return mode.lower()
        if settings.get('enable_obstacle_avoidance') is False:
            return 'off'
        return 'stop'

    def _obstacle_mode_callback(self, msg: String):
        """Set obstacle handling mode at runtime: 'off' | 'stop' | 'avoid'."""
        mode = (msg.data or '').strip().lower()
        if mode not in ('off', 'stop', 'avoid'):
            self.get_logger().warn(f"Ignoring invalid obstacle_mode '{msg.data}' (expected off|stop|avoid)")
            return
        self._apply_obstacle_mode(mode)

    def _obstacle_avoidance_enabled_callback(self, msg: Bool):
        """Legacy boolean control: True -> 'avoid', False -> 'off'.

        Retained for backward compatibility. The precise three-way control is
        /control/obstacle_mode (String); prefer that.
        """
        self._apply_obstacle_mode('avoid' if msg.data else 'off')

    def _apply_obstacle_mode(self, new_mode: str):
        """Apply an obstacle handling mode, swapping the active waypoint follower as needed.

        'avoid' uses the Nav2 follower (routes around obstacles); 'stop' and 'off'
        use the linear follower (for 'stop', the protective-stop controller halts the
        robot). When the follower changes during active navigation, the current
        waypoints/mode/index are transferred so the route resumes seamlessly.
        """
        new_mode = new_mode if new_mode in ('off', 'stop', 'avoid') else 'stop'
        previous_mode = self._obstacle_mode
        self._obstacle_mode = new_mode
        self._enable_obstacle_avoidance = (new_mode != 'off')

        if previous_mode == new_mode:
            return

        self.get_logger().info(f"Obstacle mode changed: {previous_mode} -> {new_mode}")

        # Desired follower for the new mode
        new_follower = self._waypoint_follower_nav2 if new_mode == 'avoid' else self._waypoint_follower_linear
        follower_name = "Nav2" if new_mode == 'avoid' else "Linear"

        old_follower = self._waypoint_follower
        if new_follower is old_follower:
            # Mode changed but the follower is the same (e.g. stop <-> off) — nothing to swap.
            return

        old_follower_name = "Nav2" if old_follower is self._waypoint_follower_nav2 else "Linear"
        was_active = old_follower.is_active()
        current_waypoints = getattr(old_follower, '_waypoints', []) or getattr(old_follower, '_waypoints_map', [])
        current_mode = getattr(old_follower, 'mode', WaypointMode.ONCE)
        current_idx = getattr(old_follower, '_current_wp_idx', 0)

        if was_active:
            old_follower.stop()
            self.get_logger().info(f"Stopped {old_follower_name} follower")

        self._waypoint_follower = new_follower
        self.get_logger().info(f"Switched to {follower_name} waypoint follower")

        if was_active and current_waypoints:
            new_follower.set_mode(current_mode)
            new_follower.set_waypoints(current_waypoints)
            new_follower.start(start_index=current_idx)
            self.get_logger().info(
                f"Resumed navigation on {follower_name} follower at waypoint {current_idx}/{len(current_waypoints)}"
            )

    def _autonomous_operation_service(self, request, response):
        """Handle autonomous operation enable/disable service."""
        previous_state = self._autonomous_operation_enabled
        self._autonomous_operation_enabled = request.state
        
        # Track if autonomous operation has been started at least once
        if request.state and not previous_state:
            # Autonomous operation is being enabled for the first time (or re-enabled after being disabled)
            self._autonomous_operation_started_once = True
            self.get_logger().info("Autonomous operation enabled - checking RTK status for first start")
            
            # Check RTK status when enabling autonomous operation for the first time
            is_ready, reason = self._rtk_monitor.is_rtk_ready_for_docking()
            if not is_ready:
                # RTK not ready - reject the enable request
                self._autonomous_operation_enabled = False
                self.get_logger().warn(f"Autonomous operation enable rejected: {reason}")
                response.success = False
                response.message = f"RTK nicht bereit: {reason}"
                return response
            
            # Check fusion status (should be green/initialized)
            is_fusion_ok, fusion_reason = self._rtk_monitor.is_fusion_initialized()
            if not is_fusion_ok:
                self._autonomous_operation_enabled = False
                self.get_logger().warn(f"Autonomous operation enable rejected: {fusion_reason}")
                response.success = False
                response.message = f"Fusion nicht initialisiert: {fusion_reason}"
                return response
            
            self.get_logger().info(f"RTK and fusion checks passed: {reason}, {fusion_reason}")
        
        # Publish the state change so other nodes (like scheduler) can track it
        msg = Bool()
        msg.data = self._autonomous_operation_enabled
        self._autonomous_operation_pub.publish(msg)
        
        if not self._autonomous_operation_enabled:
            # If disabled, stop the robot immediately but keep waypoint follower state
            # This allows resuming the route when re-enabled
            self._publish_drive_command(0, 0)
            self.get_logger().info(f"Autonomous operation disabled - robot stopped (route paused) (command_id: {request.command_id})")
            response.message = "Autonomous operation disabled - robot stopped"
        else:
            self.get_logger().info(f"Autonomous operation enabled - robot can resume (command_id: {request.command_id})")
            response.message = "Autonomous operation enabled"
        
        response.success = True
        return response
    
    def _waypoint_service(self, request, response):
        """Handle waypoint service request."""
        command_id = request.command_id
        route_name = request.route_name if hasattr(request, 'route_name') else ''
        waypoints = request.waypoints
        mode = request.mode
        
        self.get_logger().info(f"Waypoint service called: '{route_name}' - {len(waypoints)} waypoints, mode={mode} (command_id: {command_id})")
        
        # Convert Point[] to GeoPath format. GeoPath has no route_name field —
        # route_name is tracked separately in _current_route below.
        geopath = GeoPath()
        geopath.mode = mode

        for point in waypoints:
            geopath.waypoints.append(point)
        
        # Process the waypoints
        if mode == GeoPath.STOP:
            self.get_logger().info("Waypoint service STOP mode → stopping mission")
            self._waypoint_follower.stop()
            self._publish_drive_command(0, 0)
            self._current_route = None  # Clear current route when stopped
            response.message = "Mission stopped"
        else:
            # Dispatch through the same path the scheduler uses. That handles
            # map-origin setup, GPS→map transform, set_mode/set_waypoints, and
            # the is_active() sanity check. Manually unrolling those steps here
            # is what caused the previous _waypoint_service crash.
            if len(waypoints) > 0:
                current_state = self._robot_state_machine.get_state()
                if current_state in [RobotState.DOCKED, RobotState.CHARGING]:
                    # Robot is on the dock — undock first, then NAVIGATING will
                    # pick up _pending_geopath. Reuse the scheduler path so the
                    # same docked-recovery + safety gates apply.
                    class _SyntheticScheduleResponse:
                        pass
                    synth = _SyntheticScheduleResponse()
                    synth.waypoints = list(geopath.waypoints)
                    synth.geopath_mode = geopath.mode
                    synth.route_name = route_name
                    self.get_logger().info(
                        f"Waypoint service: state={current_state.name}, triggering undocking before route '{route_name}'"
                    )
                    self._trigger_undocking_for_schedule(synth)
                    response.message = f"Undocking; route '{route_name}' queued ({len(waypoints)} wp)"
                else:
                    self._start_navigation_directly(geopath, route_name)
                    response.message = f"Waypoints set: {len(waypoints)} points, mode={mode}"
            else:
                self._current_route = None
                response.message = "No waypoints provided"
        
        # Track mission info when waypoints set via service (for monitoring only, not persistence)
        try:
            if mode != GeoPath.STOP and len(waypoints) > 0:
                wp_list = [{'lat': float(p.x), 'lon': float(p.y), 'alt': float(p.z)} for p in waypoints]
                # choose start index based on current position if possible
                start_index = 0
                if self._last_gps:
                    try:
                        current_pose_map = self._get_current_pose_map()
                        if current_pose_map:
                            nearest_idx = self._waypoint_follower.find_nearest_waypoint(current_pose_map)
                            start_index = nearest_idx
                    except Exception:
                        start_index = 0

                # Convert mode to string for JSON serialization
                mode_map = {
                    0: 'none',  # GeoPath.STOP
                    1: 'loop',  # GeoPath.LOOP
                    2: 'ping_pong',  # GeoPath.PING_PONG
                    3: 'once'  # GeoPath.ONCE (if exists)
                }
                mode_string = mode_map.get(mode, 'once')
                
                # Transition to NAVIGATING state will happen in control loop
                self._current_route = {
                    'source': 'waypoint_service',
                    'command_id': command_id,
                    'route_name': route_name if route_name else '',
                    'waypoints': wp_list,
                    'mode': mode_string,
                    'current_waypoint_index': int(start_index),
                    'started_at': time()
                }
        except Exception:
            pass

        response.success = True
        return response

    def _get_current_pose_map(self) -> Optional[PoseStamped]:
        """Get current robot pose in map frame.
        
        Tries TF lookup first (map → base_footprint), falls back to GPS-based calculation
        if TF is not available or lookup fails.
        
        Returns:
            PoseStamped in map frame, or None if both TF and GPS data are unavailable
        """
        pose: Optional[PoseStamped] = None

        # GPS-based pose (fast, no blocking). TF inside this node is unreliable
        # (buffer doesn't accumulate frames — see project_docking_precision_fix.md).
        # GPS + Fixposition yaw override gives equivalent accuracy for RTK-based nav.
        if self._last_gps is None or self._last_orientation is None:
            return None
        if self._transform_manager.get_map_origin() is None:
            if not self._transform_manager.auto_set_map_origin_from_tf_and_gps(
                self._last_gps, timeout=0.01
            ):
                return None
        x, y, z = self._transform_manager.gps_to_map(
            self._last_gps[0], self._last_gps[1], self._last_gps[2]
        )
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = self._last_orientation

        # Override orientation with RTK dual-antenna yaw if available.
        # The odom/TF quaternion has been observed to be unreliable (random
        # ±180° yaw) while /fixposition/ypr.z is stable; see RCA in this branch.
        if self._last_yaw_rad is not None:
            yaw = self._last_yaw_rad
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
        elif not self._yaw_override_warned:
            self.get_logger().warn(
                "No /fixposition/ypr yet — using odometry quaternion as heading "
                "(unreliable). Override will engage once first ypr message arrives."
            )
            self._yaw_override_warned = True

        return pose
    
    def _is_at_charge_position(self, tolerance: Optional[float] = None) -> bool:
        """Check if robot is at charge position.
        
        Args:
            tolerance: Distance tolerance in meters. If None, uses appropriate tolerance
                      based on current state (5cm for charging, 40cm for undocking check)
            
        Returns:
            True if at charge position within tolerance
        """
        if not self._charge_position or not self._last_gps:
            return False
        
        # Startup recovery flag: if we detected charge position during startup,
        # maintain that state until we complete the transition to DOCKED/CHARGING
        if self._startup_at_charge_pos:
            current_robot_state = self._robot_state_machine.get_state() if hasattr(self, '_robot_state_machine') else None
            if current_robot_state in [RobotState.DOCKED, RobotState.CHARGING]:
                # Clear the flag now that we're in a stable docked/charging state
                self._startup_at_charge_pos = False
            elif current_robot_state in [RobotState.UNDOCKING, RobotState.UNDOCKED]:
                # Robot is actively leaving the dock: the startup latch must not keep
                # asserting "at charge" or it deadlocks the undock transition
                # (RCA 2026-06-05). Drop the latch and fall through to the real
                # GPS-distance check below.
                self._startup_at_charge_pos = False
            else:
                return True
        
        # Get current state for decision making
        current_state = self._robot_state_machine.get_state() if hasattr(self, '_robot_state_machine') else None
        
        # IMPORTANT: During UNDOCKING, always check GPS distance first
        # This ensures accurate detection even if unified_charging_state is still True
        # (unified_charging_state can be delayed/stale during undocking)
        if current_state == RobotState.UNDOCKING:
            # For undocking, use larger tolerance (40cm) and always check GPS distance
            if tolerance is None:
                tolerance = self._charge_position_tolerance_undocking
            robot_pos = (self._last_gps[0], self._last_gps[1], self._last_gps[2])
            charge_pos = (self._charge_position[0], self._charge_position[1], 0.0)
            distance = self._coord_transformer.compute_distance(robot_pos, charge_pos)
            return distance <= tolerance
        
        # For non-UNDOCKING states, use unified charging state as primary indicator (most reliable)
        # Check robot state machine state instead of docking controller
        if self._unified_charging_state:
            if current_state in [RobotState.DOCKED, RobotState.CHARGING]:
                return True
            # If unified_charging_state is True but state machine is not DOCKED/CHARGING, don't trust it
            # (might be stale state after undocking)
        
        # Determine appropriate tolerance if not specified
        if tolerance is None:
            # For MANUAL/IDLE states, use larger tolerance (50cm) since GPS isn't as precise
            # This prevents false negatives when transitioning from CHARGING to MANUAL
            # while robot is still physically at charge position
            if current_state in [RobotState.MANUAL, RobotState.IDLE]:
                tolerance = 0.5  # 50cm for general position checks
            else:
                # For charging/docked states, use precise tolerance (5cm)
                tolerance = self._charge_position_tolerance_charging
        
        # Fallback: check GPS distance to charge position
        robot_pos = (self._last_gps[0], self._last_gps[1], self._last_gps[2])
        charge_pos = (self._charge_position[0], self._charge_position[1], 0.0)
        distance = self._coord_transformer.compute_distance(robot_pos, charge_pos)
        
        return distance <= tolerance
    
    def _is_at_home_position(self, tolerance: Optional[float] = None) -> bool:
        """Check if robot is at home position.
        
        Args:
            tolerance: Distance tolerance in meters. If None, uses home_tolerance from settings
                      (default 0.5m if not set in settings)
            
        Returns:
            True if at home position
        """
        if not self._home_position or not self._last_gps:
            return False
        
        # Use tolerance from settings if not specified
        if tolerance is None:
            # Load home_tolerance from settings (default 0.5m = 50cm if not set)
            tolerance = self._settings_cache.get('home_tolerance', 0.5)
        
        distance = self._get_home_distance()
        if distance is None:
            return False
        
        return distance <= tolerance
    
    def _is_too_close_to_charge_position(self, safety_distance: float = 1.0) -> bool:
        """Check if robot is too close to charge position (safety check to prevent recovery near charging area).
        
        This is used to prevent the robot from starting recovery navigation when it's still
        in or very close to the charging area (e.g., during or shortly after undocking).
        This prevents the robot from trying to navigate and potentially hitting the charging
        station or hut.
        
        This check only applies when is_at_charge_pos is False. If is_at_charge_pos is True,
        recovery is already blocked by the main condition check.
        
        Args:
            safety_distance: Minimum safe distance from charge position in meters (default: 1.0m)
            
        Returns:
            True if robot is too close to charge position (within safety_distance)
        """
        if not self._charge_position or not self._last_gps:
            return False
        
        # Calculate distance to charge position
        robot_pos = (self._last_gps[0], self._last_gps[1], self._last_gps[2])
        charge_pos = (self._charge_position[0], self._charge_position[1], 0.0)
        distance = self._coord_transformer.compute_distance(robot_pos, charge_pos)
        
        return distance <= safety_distance
    
    def _create_state_context(self) -> RobotStateContext:
        """Create RobotStateContext from current system state.
        
        Returns:
            RobotStateContext with current system information
        """
        # Check RTK status
        is_rtk_ready, _ = self._rtk_monitor.is_rtk_ready_for_docking()
        is_fusion_ok, _ = self._rtk_monitor.is_fusion_initialized()
        rtk_fix = is_rtk_ready and is_fusion_ok
        
        # Check sensor validity (GPS and orientation available)
        sensor_info_valid = (self._last_gps is not None and 
                            self._last_orientation is not None and
                            not self._gps_jump_detected)
        
        # Check if docked (use robot state machine state)
        current_robot_state = self._robot_state_machine.get_state()
        docked = (current_robot_state in [RobotState.DOCKED, RobotState.CHARGING])
        
        # Check route and schedule status
        route_active = self._waypoint_follower.is_active()
        # schedule_active should be True if:
        # 1. There's an active schedule from the scheduler (tracked independently), OR
        # 2. The route is currently active (being followed)
        # This ensures schedule_active remains True during state transitions even if route_active is briefly False
        schedule_active = self._has_active_schedule or route_active
        
        # Check if charging is needed - use value from scheduler topic
        need_charge = self._charging_requested
        
        return RobotStateContext(
            charge_pos_set=self._charge_position is not None,
            is_at_charge_pos=self._is_at_charge_position(),
            is_near_charge_pos=self._is_at_charge_position(tolerance=self._charge_position_tolerance_undocking),
            is_at_home_pos=self._is_at_home_position(),
            is_near_home_pos=self._is_at_home_position(tolerance=0.4),  # reduced to prevent premature home detection
            need_charge=need_charge,
            route_active=route_active,
            schedule_active=schedule_active,
            rtk_fix=rtk_fix,
            sensor_info_valid=sensor_info_valid,
            docked=docked,
            autonomous_operation_enabled=self._autonomous_operation_enabled,
            charging_relay_enabled=self._charging_relay_enabled
        )
    
    def _build_state_update_kwargs(self, robot_pose_map: PoseStamped) -> dict:
        """Build kwargs dictionary for state.on_update() calls.
        
        Args:
            robot_pose_map: Current robot pose (in map frame)
            
        Returns:
            Dictionary with kwargs for state.on_update()
        """
        kwargs = {
            'robot_pose': robot_pose_map,
            'coordinate_transformer': self._coord_transformer,
        }
        
        # Add docking-related kwargs
        if self._charge_position and self._home_position:
            # Get origin GPS - MUST use map origin (FP_ENU0) to match robot_pose frame
            # robot_pose is in map frame (from TF), so home/charge ENU coords must use same origin
            origin_gps = self._transform_manager.get_map_origin()
            
            if origin_gps is None and self._last_gps:
                # Map origin not set yet - try to set it from TF+GPS now
                # This is critical for docking to work correctly!
                # Rate-limit the log messages to avoid spam (every 5 seconds)
                current_time = time()
                should_log = (
                    self._last_map_origin_attempt_time is None or 
                    (current_time - self._last_map_origin_attempt_time) >= 5.0
                )
                
                if should_log:
                    self.get_logger().info(
                        "Map origin not set - attempting to set from TF+GPS"
                    )
                
                if self._transform_manager.auto_set_map_origin_from_tf_and_gps(self._last_gps):
                    origin_gps = self._transform_manager.get_map_origin()
                    self.get_logger().info(
                        f"Map origin successfully set from TF+GPS: {origin_gps}"
                    )
                    self._last_map_origin_attempt_time = None  # Reset on success
                else:
                    # TF+GPS failed - will retry on next control loop iteration
                    # DO NOT use coord_transformer origin as fallback - it's set to charge_position
                    # which would cause a coordinate system mismatch!
                    if should_log:
                        self.get_logger().warn(
                            "Failed to set map origin from TF+GPS - will retry. "
                            "Docking/undocking requires valid TF transforms."
                        )
                        self._last_map_origin_attempt_time = current_time
            
            if origin_gps:
                kwargs['target_gps'] = self._charge_position
                kwargs['home_gps'] = self._home_position
                kwargs['origin_gps'] = origin_gps
        else:
            # Log why GPS positions are not added
            self.get_logger().warn(
                f"GPS positions not added to kwargs: "
                f"charge_position={self._charge_position is not None}, "
                f"home_position={self._home_position is not None}"
            )
        
        # Add waypoint follower for NAVIGATING and RETURNING_TO_HOME states
        kwargs['waypoint_follower'] = self._waypoint_follower
        
        # Protective-stop reflex layer: ONLY in 'stop' mode. In 'avoid', Nav2 plans
        # around obstacles using its costmap, so the reflex must not run here — it would
        # override Nav2's commands with (0,0) and the robot could never execute the
        # manoeuvre. In 'off' there is no reaction. (The LiDAR-loss safety halt below is
        # gated separately on _enable_obstacle_avoidance and still arms for both modes.)
        kwargs['obstacle_avoidance'] = self._obstacle_avoidance if self._obstacle_mode == 'stop' else None
        kwargs['obstacle_sectors'] = self._obstacle_sectors_msg  # Sector analysis from detection

        # Callbacks will be added at call site as needed

        return kwargs
    
    def _publish_robot_state(self, state: RobotState, context: RobotStateContext):
        """Publish current robot state to topic.
        
        Args:
            state: Current RobotState
            context: Current RobotStateContext
        """
        state_data = {
            'state': state.name,
            'state_id': int(state),
            'previous_state': self._robot_state_machine.get_previous_state().name if self._robot_state_machine.get_previous_state() else None,
            'context': {
                'charge_pos_set': context.charge_pos_set,
                'is_at_charge_pos': context.is_at_charge_pos,
                'is_near_charge_pos': context.is_near_charge_pos,
                'is_at_home_pos': context.is_at_home_pos,
                'is_near_home_pos': context.is_near_home_pos,
                'need_charge': context.need_charge,
                'route_active': context.route_active,
                'schedule_active': context.schedule_active,
                'rtk_fix': context.rtk_fix,
                'sensor_info_valid': context.sensor_info_valid,
                'docked': context.docked,
                'charging_relay_enabled': context.charging_relay_enabled
            }
        }
        
        # Add active route information if available
        if self._current_route and context.route_active:
            # Convert waypoints from internal format (lat/lon/alt) to format expected by frontend (latitude/longitude/altitude)
            waypoints = []
            for wp in self._current_route.get('waypoints', []):
                waypoints.append({
                    'latitude': wp.get('lat', 0.0),
                    'longitude': wp.get('lon', 0.0),
                    'altitude': wp.get('alt', 0.0)
                })
            
            state_data['active_route'] = {
                'name': self._current_route.get('route_name', ''),
                'waypoints': waypoints,
                'mode': self._current_route.get('mode', 'once')
            }
        else:
            state_data['active_route'] = None
        
        msg = String()
        msg.data = json.dumps(state_data)
        # Only log on state transitions, not every control loop iteration
        if not hasattr(self, '_last_published_state') or self._last_published_state != state:
            self.get_logger().info(f"Robot state transition: {state.name} (context: {asdict(context)})")
            self._last_published_state = state
        self._robot_state_pub.publish(msg)
    
    def _control_loop(self):
        """Main control loop.
        
        Only publishes drive commands when an active mission/operation is running.
        If no active operation, doesn't publish anything to allow manual control.
        Exception: If obstacle detection in manual mode is enabled, it will stop the robot.
        """
        # Publish estimated speed (km/h) for monitoring and UI
        speed_msg = Float32()
        speed_msg.data = self._speed_estimator.get_speed_kmh()
        self._speed_estimate_pub.publish(speed_msg)
        
        # Update state machine based on current context
        context = self._create_state_context()
        current_state_before = self._robot_state_machine.get_state()
        
        # Startup recovery: check if we need to transition to DOCKED
        # This runs once when both GPS and RTK are available
        if not self._startup_recovery_done:
            if context.rtk_fix and self._last_gps is not None:
                self.get_logger().info("Control loop: Starting startup recovery check...")
                self._check_startup_charge_position_recovery()
                # Recreate context after startup recovery because _startup_at_charge_pos may have changed
                # This ensures is_at_charge_pos reflects the startup recovery detection
                context = self._create_state_context()
                self.get_logger().info(
                    f"Control loop: After startup recovery (context: {asdict(context)})"
                )
            # Log why we're not checking yet (only once per condition)
            elif not hasattr(self, '_startup_recovery_logged'):
                if self._last_gps is None:
                    self.get_logger().info("Startup recovery: Waiting for GPS...")
                elif not context.rtk_fix:
                    self.get_logger().info("Startup recovery: Waiting for RTK fix...")
                self._startup_recovery_logged = True
        
        # Periodic schedule query - check if we should query the scheduler
        # Only query in allowed states (IDLE, DOCKED, CHARGING, UNDOCKED)
        if current_state_before in self._allowed_schedule_query_states:
            self._check_periodic_schedule_query()
        
        # Let state machine handle automatic transitions via determine_next_state()
        # State classes' can_exit_to() methods handle transition validation
        next_state = self._robot_state_machine.determine_next_state(context)
        if next_state is not None:
            # Only log and transition if state actually changes (avoid repeated logs)
            if current_state_before != next_state:
                # Transition to next state (state classes' can_exit_to() already validated it)
                self.get_logger().info(f"State machine transition: {current_state_before.name} -> {next_state.name}")
                self._robot_state_machine.transition_to(next_state, context)
        
        current_state = self._robot_state_machine.get_state()
        self._current_state_name = current_state.name  # cached for [NAVTEL] telemetry chokepoint

        # Explicit transitions have been removed - state machine's determine_next_state()
        # handles all automatic transitions. State entry callbacks handle setup.

        # Publish current robot state
        self._publish_robot_state(current_state, context)
        
        # Get current pose once for all state-specific handling (avoids NameError in ERROR state)
        robot_pose_map = self._get_current_pose_map()
        
        # Handle state-specific behavior
        # Manual override: don't publish commands if in MANUAL state
        if current_state == RobotState.MANUAL:
            return
        
        # Error state: handled by ErrorState.on_update() - returns (0.0, 0.0)
        if current_state == RobotState.ERROR:
            # Build kwargs and call state machine update
            kwargs = self._build_state_update_kwargs(robot_pose_map) if robot_pose_map else {}
            commands = self._robot_state_machine.update(context, **kwargs)
            
            if commands is not None:
                steering, speed = commands
                if steering is not None and speed is not None:
                    self._publish_drive_command(steering, speed)
                else:
                    self._publish_drive_command(0.0, 0.0)
            else:
                self._publish_drive_command(0.0, 0.0)
            return
        
        # UNINITIALIZED: wait for configuration
        if current_state == RobotState.UNINITIALIZED:
            return
        
        # IDLE: no active operation, don't publish commands
        # State transitions are handled above (line ~2299) via determine_next_state()
        # so returning here is safe - robot can still transition to other states
        if current_state == RobotState.IDLE:
            return
        
        # If no GPS/pose available, don't publish anything (let manual control work)
        if robot_pose_map is None:
            if current_state == RobotState.UNDOCKING:
                # Try to set map origin if it's missing and GPS data is available
                # Rate limit: only try once every 2 seconds to avoid spam
                current_time = time()
                should_attempt = (
                    self._transform_manager.get_map_origin() is None and 
                    self._last_gps is not None and
                    (self._last_map_origin_attempt_time is None or 
                     current_time - self._last_map_origin_attempt_time >= 2.0)
                )
                
                if should_attempt:
                    self._last_map_origin_attempt_time = current_time
                    self.get_logger().info(
                        "UNDOCKING state: robot_pose_map is None, attempting to set map origin from TF and GPS"
                    )
                    robot_gps = self._last_gps
                    if self._transform_manager.auto_set_map_origin_from_tf_and_gps(robot_gps):
                        self.get_logger().info(
                            "UNDOCKING state: Map origin successfully set from TF and GPS"
                        )
                        # Retry getting robot_pose_map after setting origin
                        robot_pose_map = self._get_current_pose_map()
                        if robot_pose_map is not None:
                            self.get_logger().info(
                                "UNDOCKING state: robot_pose_map now available after setting map origin"
                            )
                        else:
                            self.get_logger().warn(
                                "UNDOCKING state: robot_pose_map still None after setting map origin - TF may not be available"
                            )
                    else:
                        self.get_logger().warn(
                            "UNDOCKING state: Failed to set map origin from TF and GPS - TF may not be available"
                        )
                else:
                    if self._transform_manager.get_map_origin() is None:
                        if self._last_gps is None:
                            self.get_logger().warn(
                                "UNDOCKING state: robot_pose_map is None - map origin not set and GPS data not available"
                            )
                        # else: rate limited, will try again in 2 seconds
                    else:
                        self.get_logger().warn(
                            "UNDOCKING state: robot_pose_map is None - TF lookup may have failed"
                        )
            # In NAVIGATING/RETURNING_TO_HOME: only halt (speed/rotation 0), keep follower active
            if current_state in (RobotState.NAVIGATING, RobotState.RETURNING_TO_HOME):
                if self._waypoint_follower.is_active():
                    if not getattr(self, '_pose_unavailable_nav_warned', False):
                        self._pose_unavailable_nav_warned = True
                        warn_msg = String()
                        warn_msg.data = (
                            "Pose/Map nicht verfügbar – Roboter angehalten, fährt weiter sobald Pose verfügbar ist."
                        )
                        self._log_warn_pub.publish(warn_msg)
                        self.get_logger().warn(
                            "Pose/Map not available - robot halted (speed/rotation 0), will resume when pose available."
                        )
                self._pose_unavailable_nav_halted = True
            self._publish_drive_command(0.0, 0.0)
            return
        
        # Check autonomous operation for states that require it
        # Exception: UNDOCKING should work even if autonomous operation is disabled (safety-critical)
        if current_state in [RobotState.NAVIGATING, RobotState.RETURNING_TO_HOME, RobotState.DOCKING]:
            if not self._autonomous_operation_enabled:
                return
        # UNDOCKING is allowed even if autonomous operation is disabled (safety-critical)
        
        # LiDAR timeout: when obstacle avoidance is on, do not drive in autonomous mode without recent LiDAR data
        if self._enable_obstacle_avoidance and current_state in [RobotState.NAVIGATING, RobotState.RETURNING_TO_HOME]:
            now = time()
            lidar_available = (
                self._last_lidar_msg_time is not None
                and (now - self._last_lidar_msg_time) <= self._lidar_timeout
            )
            if not lidar_available:
                # Only stop drive commands; do NOT stop the waypoint follower so that when
                # LiDAR returns we can resume the same route without state transition.
                self._publish_drive_command(0, 0)
                if not self._lidar_timeout_warn_sent:
                    warn_msg = String()
                    warn_msg.data = (
                        "Livox-LiDAR nicht erreichbar – autonome Fahrt angehalten. "
                        "Fahrt wird fortgesetzt, sobald wieder LiDAR-Daten ankommen."
                    )
                    self._log_warn_pub.publish(warn_msg)
                    self.get_logger().warning(
                        "LiDAR timeout – pausing drive and sending warning to web app"
                    )
                    self._lidar_timeout_warn_sent = True
                return
            # LiDAR was timed out, now available again – notify web app once
            if self._lidar_timeout_warn_sent:
                info_msg = String()
                info_msg.data = "Livox-LiDAR wieder verfügbar – Fahrt wird fortgesetzt."
                self._log_info_pub.publish(info_msg)
                self.get_logger().info("LiDAR available again – resuming drive, info sent to web app")
                self._lidar_timeout_warn_sent = False  # Reset so next timeout we warn again
        
        # GPS watchdog: if jump detected, stop and wait for RTK fixed
        if self._gps_jump_detected:
            is_ready, reason = self._rtk_monitor.is_rtk_ready_for_docking()
            if not is_ready:
                # Still waiting for RTK fixed after jump
                if current_state == RobotState.UNDOCKING:
                    self.get_logger().warn(f"UNDOCKING state: GPS jump detected - waiting for RTK fixed: {reason}")
                else:
                    self.get_logger().warn(f"GPS jump detected - waiting for RTK fixed: {reason}")
                self._publish_drive_command(0, 0)
                
                # Stop waypoint follower
                if self._waypoint_follower.is_active():
                    self._waypoint_follower.stop()
                
                return
            else:
                # RTK is now fixed - clear jump flag and continue
                self.get_logger().info(f"RTK fixed after GPS jump - resuming operation: {reason}")
                self._gps_jump_detected = False
        
        # Fusion status check: fusion engine should be green (initialized) all the time
        if self._rtk_check_enabled:
            is_fusion_ok, fusion_reason = self._rtk_monitor.is_fusion_initialized()
            if not is_fusion_ok:
                if current_state == RobotState.UNDOCKING:
                    self.get_logger().warn(f"UNDOCKING state: Fusion engine not initialized: {fusion_reason} - STOPPING")
                else:
                    self.get_logger().warn(f"Fusion engine not initialized: {fusion_reason} - STOPPING")
                self._publish_drive_command(0, 0)
                
                # Stop waypoint follower
                if self._waypoint_follower.is_active():
                    self._waypoint_follower.stop()
                    self.get_logger().warn("Waypoint following stopped due to fusion engine not initialized")
                
                return
        
        # RTK status check: Only require RTK fixed for:
        # 1. First time autonomous operation is enabled (checked in service)
        # 2. Docking (handled by docking controller)
        # 3. Undocking (checked in undock_callback)
        # After autonomous operation has started, allow RTK float (orange state) during normal navigation
        # But if RTK was fixed when starting, we can continue even if it goes to float
        # Only block if RTK is completely lost (no fix at all)
        # Check if we're in a docking/undocking state (which requires RTK fixed)
        current_robot_state = self._robot_state_machine.get_state()
        is_docking_or_undocking = current_robot_state in [RobotState.DOCKING, RobotState.UNDOCKING]
        
        if self._rtk_check_enabled and not is_docking_or_undocking:
            # Check if we have at least RTK float (status 5) or better
            # This allows orange state (RTK float) but blocks if RTK is completely lost
            if self._rtk_monitor._fusion_state is None:
                self.get_logger().warn("No fusion state available - STOPPING")
                self._publish_drive_command(0, 0)
                if self._waypoint_follower.is_active():
                    self._waypoint_follower.stop()
                return
            
            gnss1 = self._rtk_monitor._fusion_state.gnss1_status
            gnss2 = self._rtk_monitor._fusion_state.gnss2_status
            
            # Allow RTK float (5) or RTK fixed (8), but block if no fix or only SPP
            min_acceptable_status = RTKStatus.RTK_FLOAT
            has_acceptable_rtk = (gnss1 is not None and gnss1 >= min_acceptable_status) or \
                                (gnss2 is not None and gnss2 >= min_acceptable_status)
            
            if not has_acceptable_rtk:
                # RTK completely lost or degraded to SPP - emergency stop
                # Get status names from current status
                status_dict = self._rtk_monitor.get_current_status()
                gnss1_name = status_dict.get('gnss1_status_name', 'Unknown')
                gnss2_name = status_dict.get('gnss2_status_name', 'Unknown')
                self.get_logger().warn(f"RTK signal lost or degraded (GNSS1: {gnss1_name}, GNSS2: {gnss2_name}) - STOPPING")
                self._publish_drive_command(0, 0)
                
                # Stop waypoint follower to prevent continuing when signal returns
                if self._waypoint_follower.is_active():
                    self._waypoint_follower.stop()
                    self.get_logger().warn("Waypoint following stopped due to RTK signal loss")
                
                return
        
        # State-based control logic
        # Priority 1: DOCKING, UNDOCKING states
        # Delegate to state machine - states will handle docking/undocking in their on_update() methods
        if current_state in [RobotState.DOCKING, RobotState.UNDOCKING]:
            if current_state == RobotState.UNDOCKING:
                self.get_logger().info("UNDOCKING state: Reached control loop - processing undocking")
            # Build kwargs for state update
            kwargs = self._build_state_update_kwargs(robot_pose_map)
            
            # Diagnostic logging for UNDOCKING state
            if current_state == RobotState.UNDOCKING:
                missing_params = []
                if 'target_gps' not in kwargs:
                    missing_params.append('target_gps')
                if 'home_gps' not in kwargs:
                    missing_params.append('home_gps')
                if 'origin_gps' not in kwargs:
                    missing_params.append('origin_gps')
                if 'coordinate_transformer' not in kwargs or kwargs['coordinate_transformer'] is None:
                    missing_params.append('coordinate_transformer')
                if 'robot_pose' not in kwargs or kwargs['robot_pose'] is None:
                    missing_params.append('robot_pose')
                
                if missing_params:
                    self.get_logger().error(
                        f"UNDOCKING state: Missing required parameters: {', '.join(missing_params)}. "
                        f"charge_position={self._charge_position is not None}, "
                        f"home_position={self._home_position is not None}, "
                        f"coord_transformer={self._coord_transformer is not None}, "
                        f"robot_pose_map={robot_pose_map is not None}"
                    )
                else:
                    self.get_logger().info(
                        f"UNDOCKING state: All required parameters present. "
                        f"target_gps={kwargs.get('target_gps') is not None}, "
                        f"home_gps={kwargs.get('home_gps') is not None}, "
                        f"origin_gps={kwargs.get('origin_gps') is not None}"
                    )
            
            # Add callbacks for docking/undocking status updates and transitions
            def on_home_reached():
                # For docking status updates (could add debug info if needed)
                self._publish_charging_status("aligning", "Reached line at home position, aligning to line")
            
            def on_aligned():
                # For docking status updates
                self._publish_charging_status("following", "Aligned to line, following to charge point")
            
            def on_docked():
                # Don't process docked callback if we're in UNDOCKING robot state
                if current_state == RobotState.UNDOCKING:
                    self.get_logger().warn("on_docked callback called during UNDOCKING state - ignoring")
                    return
                
                self._publish_charging_status("docked", "Successfully docked")
                
                # Stop waypoint follower when docking completes (clear any active route)
                if self._waypoint_follower.is_active():
                    self.get_logger().info("on_docked callback: stopping waypoint follower")
                    self._waypoint_follower.stop()
                    self._current_route = None
                
                # Transition to DOCKED state
                # When on_docked is called, we know we're at charge position, so create context
                # that reflects this even if GPS check hasn't updated yet
                context = self._create_state_context()
                # Force is_at_charge_pos to True since we know we're docked
                # This ensures the transition works even if GPS position check lags
                context.is_at_charge_pos = True
                self.get_logger().info("on_docked callback: transitioning to DOCKED state")
                self._robot_state_machine.transition_to(RobotState.DOCKED, context)

            def on_dock_failed(reason):
                # Docking could not land on the charge point within tolerance.
                # There is no contact sensor, so we must NOT declare "docked" off
                # target: we stay in DOCKING (stopped), keep autonomy enabled, and
                # surface the failure to the operator. No transition to DOCKED.
                self._publish_charging_status(
                    "blocked",
                    f"Andocken nicht möglich ({reason}) — bitte Position prüfen und erneut starten."
                )
                self.get_logger().error(f"Docking failed (not docked): {reason}")
                if self._waypoint_follower.is_active():
                    self._waypoint_follower.stop()

            def on_undocked():
                self._publish_charging_status("completed", "Undocking complete")
                # Transition to UNDOCKED. The controller only calls this after
                # confirming the dock is clear, so the transition is expected to pass.
                # If it is somehow still rejected, fail safe rather than loop silently.
                context = self._create_state_context()
                if not self._robot_state_machine.transition_to(RobotState.UNDOCKED, context):
                    self.get_logger().error(
                        "on_undocked: UNDOCKED transition rejected despite cleared dock "
                        f"(at_charge={context.is_at_charge_pos}) - forcing ERROR"
                    )
                    self._publish_drive_command(0, 0)
                    if self._waypoint_follower.is_active():
                        self._waypoint_follower.stop()
                    self._robot_state_machine.set_error()

            def on_undock_failed(reason):
                # Undocking could not clear the dock (off-target finish, or timeout).
                # With no contact sensor we must not pretend the robot is free: stop,
                # tell the operator, and drop to ERROR (operator-recoverable, exits only
                # to IDLE/MANUAL). This is the fail-safe that replaces the silent
                # "stuck in UNDOCKING commanding zero" freeze (RCA 2026-06-05).
                self._publish_charging_status(
                    "error",
                    f"Abdocken fehlgeschlagen ({reason}) — bitte Position prüfen und manuell freifahren."
                )
                self.get_logger().error(f"Undocking failed (not undocked): {reason}")
                self._publish_drive_command(0, 0)
                if self._waypoint_follower.is_active():
                    self._waypoint_follower.stop()
                self._robot_state_machine.set_error()

            # Only add callbacks for DOCKING state
            if current_state == RobotState.DOCKING:
                kwargs['on_home_reached'] = on_home_reached
                kwargs['on_aligned'] = on_aligned
                kwargs['on_docked'] = on_docked
                kwargs['on_dock_failed'] = on_dock_failed
            elif current_state == RobotState.UNDOCKING:
                kwargs['on_undocked'] = on_undocked
                kwargs['on_undock_failed'] = on_undock_failed
            
            # Call state machine update - it will delegate to the current state's on_update()
            commands = self._robot_state_machine.update(context, **kwargs)
            
            if current_state == RobotState.UNDOCKING:
                if commands is None:
                    self.get_logger().warn("UNDOCKING state: update() returned None - no commands generated")
                elif commands[0] is None or commands[1] is None:
                    self.get_logger().warn(f"UNDOCKING state: update() returned partial commands: steering={commands[0]}, speed={commands[1]}")
                else:
                    self.get_logger().info(f"UNDOCKING state: update() returned commands: steering={commands[0]:.2f}, speed={commands[1]:.2f}")
            
            if commands is not None:
                steering, speed = commands
                if steering is not None and speed is not None:
                    self._publish_drive_command(steering, speed)
                elif steering is not None and speed is None:
                    self._publish_drive_command(steering, 0.0)
                else:
                    pass
            return
        
        # Priority 2: States that just stop the robot (DOCKED, CHARGING, UNDOCKED)
        # Delegate to state machine - states will handle their behavior in on_update()
        if current_state in [RobotState.DOCKED, RobotState.CHARGING, RobotState.UNDOCKED]:
            # Build kwargs for state update
            kwargs = self._build_state_update_kwargs(robot_pose_map) if robot_pose_map else {}
            
            # Call state machine update - it will delegate to the current state's on_update()
            commands = self._robot_state_machine.update(context, **kwargs)
            
            if commands is not None:
                steering, speed = commands
                if steering is not None and speed is not None:
                    self._publish_drive_command(steering, speed)
                elif steering is not None and speed is None:
                    self._publish_drive_command(steering, 0.0)
                else:
                    # No commands or (None, None) - state is waiting
                    if steering is None and speed is None:
                        # Explicitly stop for these states if they return None
                        if current_state in [RobotState.DOCKED, RobotState.CHARGING]:
                            self._publish_drive_command(0.0, 0.0)
            else:
                # State returned None - explicitly stop for DOCKED and CHARGING
                if current_state in [RobotState.DOCKED, RobotState.CHARGING]:
                    self._publish_drive_command(0.0, 0.0)
            return
        
        # State: RETURNING_TO_HOME - handled by waypoint follower with reverse route
        # This is handled in the waypoint following section below
        
        # Priority 2: Obstacle avoidance - handled by state machine-based controller
        # The new controller is integrated via navigation_helper and uses OccupancyGrid data
        # No need for separate stop-and-wait logic here - it's handled in the state machine
        
        # Priority 3: Waypoint following for NAVIGATING and RETURNING_TO_HOME states
        # Delegate to state machine - states will handle waypoint following in their on_update() methods
        if current_state in [RobotState.NAVIGATING, RobotState.RETURNING_TO_HOME]:
            # Recovery: pose was unavailable and we halted; now pose is available again
            if getattr(self, '_pose_unavailable_nav_halted', False):
                self._pose_unavailable_nav_halted = False
                self._pose_unavailable_nav_warned = False
                self.get_logger().info(
                    "Pose/Map available again - resuming waypoint following."
                )
            # Build kwargs for state update
            kwargs = self._build_state_update_kwargs(robot_pose_map)
            
            # Add callbacks for waypoint following
            def on_route_completed():
                self.get_logger().info("Route completed - notifying scheduler")
                msg = Bool()
                msg.data = True
                self._route_completed_pub.publish(msg)
                # Clear current route tracking before transition
                self._current_route = None
                # Transition to IDLE state (will be handled by state machine)
                context = self._create_state_context()
                self._robot_state_machine.transition_to(RobotState.IDLE, context)
                # Query scheduler immediately to avoid auto-docking between repetitions
                if self._autonomous_operation_enabled:
                    self._query_schedule_action_async()
            
            def on_waypoint_reached(idx: int):
                try:
                    # Update current waypoint index for tracking
                    if self._current_route is not None:
                        self._current_route['current_waypoint_index'] = int(idx)
                except Exception:
                    pass
            
            kwargs['on_route_completed'] = on_route_completed
            kwargs['on_waypoint_reached'] = on_waypoint_reached
            
            # Call state machine update - it will delegate to the current state's on_update()
            commands = self._robot_state_machine.update(context, **kwargs)
            
            if commands is not None:
                steering, speed = commands
                if steering is not None and speed is not None:
                    self._publish_drive_command(steering, speed)
                elif steering is not None and speed is None:
                    # Just steering, no speed (shouldn't happen with waypoint following)
                    self._publish_drive_command(steering, 0.0)
                else:
                    # No commands - state is waiting
                    pass
            return

    def _publish_drive_command(self, steer: float, speed: float):
        """Publish drive command."""
        
        # Convert steering and speed to wheel velocities using kinematics
        # Using same calculation as original (approximation)
        wheel_radius = 0.535 / 2  # meters
        wheel_base = 0.637  # meters
        
        # Clamp inputs
        steer = max(-100.0, min(100.0, steer))
        speed = max(-100.0, min(100.0, speed))
        
        # Override max_speed to 1.0 during rotation in place (PID regulator calibrated for 1.0x)
        # Rotation in place detection is handled in waypoint_follower (heading error > 22.5°)
        # Docking/undocking speed is controlled by state classes (DockingState, UndockingState)
        # which use their own speed settings in DockingConfig/UndockingConfig dataclasses
        effective_max_speed = self.max_speed
        if self._waypoint_follower.is_rotating_in_place():
            effective_max_speed = 1.0
        
        # Convert to linear and angular velocity
        v = (speed / 100.0) * effective_max_speed
        omega = (steer / 100.0) * (effective_max_speed / (wheel_base / 2.0))
        
        # Update speed estimator with commanded speed for sanity checking
        self._speed_estimator.update_commanded_speed(v)
        
        # Differential drive kinematics
        v_left = v - omega * wheel_base / 2.0
        v_right = v + omega * wheel_base / 2.0
        
        # Convert to angular velocities
        left_rad_s = v_left / wheel_radius
        right_rad_s = v_right / wheel_radius
        
        # Debug logging for docking/undocking is handled by state classes
        
        cmd = CommandDrive()
        cmd.left_vel = left_rad_s
        cmd.right_vel = right_rad_s

        self._cmd_pub.publish(cmd)

        # Telemetry chokepoint: every motion command (drive AND watchdog halt) passes
        # here. steer/speed are pre-kinematics follower units (±100). Fail-soft inside.
        self._emit_navtel(steer, speed)

    def _emit_navtel(self, steer: float, speed: float):
        """Emit one structured [NAVTEL] telemetry line for evidence-based troubleshooting.

        Single greppable line correlating, on one timeline: robot state, avoidance
        reflex state, obstacle mode, the final drive command, blocked sectors, and the
        sensor/health inputs that drove the decision (LiDAR freshness, GNSS, fusion,
        GPS-jump). Emitted on any change of the key decision signals, otherwise as a
        heartbeat every telemetry.heartbeat_period seconds.

        MUST be fail-soft: telemetry can never disturb the control loop, so the whole
        body is wrapped — any error degrades to a debug line, never an exception.
        """
        if not self._telemetry_enabled:
            return
        try:
            now = time()

            try:
                avoid_state = self._obstacle_avoidance.get_current_state_name()
            except Exception:
                avoid_state = '?'

            n_stop = n_slow = 0
            sec_msg = self._obstacle_sectors_msg
            if sec_msg is not None:
                for s in sec_msg.sectors:
                    if s.blocked and s.sector_type == 'STOP':
                        n_stop += 1
                    elif s.blocked and s.sector_type == 'SLOW':
                        n_slow += 1
            blocked = f"STOP:{n_stop},SLOW:{n_slow}"

            lidar_age = (now - self._last_lidar_msg_time) if self._last_lidar_msg_time else -1.0

            wp_idx = getattr(self._waypoint_follower, '_current_wp_idx', None)
            wps = (getattr(self._waypoint_follower, '_waypoints', None)
                   or getattr(self._waypoint_follower, '_waypoints_map', None))
            wp_n = len(wps) if wps else 0

            try:
                rtk = self._rtk_monitor.get_current_status()
                g1, g2 = rtk.get('gnss1_status', '?'), rtk.get('gnss2_status', '?')
                fus, rtk_age = rtk.get('fusion_status', '?'), rtk.get('data_age_seconds', '?')
            except Exception:
                g1 = g2 = fus = rtk_age = '?'

            moving = abs(steer) > 0.5 or abs(speed) > 0.5

            # Change-signature: emit immediately when any of these flip; otherwise heartbeat.
            sig = (self._current_state_name, avoid_state, self._obstacle_mode,
                   blocked, moving, self._gps_jump_detected)
            sig_changed = sig != self._navtel_last_sig
            if sig_changed or (now - self._navtel_last_emit) >= self._telemetry_heartbeat:
                self._navtel_last_sig = sig
                self._navtel_last_emit = now
                self.get_logger().info(
                    f"[NAVTEL] state={self._current_state_name} avoid={avoid_state} "
                    f"mode={self._obstacle_mode} wp={wp_idx}/{wp_n} "
                    f"cmd_out=(steer={steer:.0f},spd={speed:.0f}) moving={int(moving)} "
                    f"blocked={blocked} lidar_age={lidar_age:.2f} "
                    f"gnss={g1}/{g2} fusion={fus} rtk_age={rtk_age} "
                    f"gps_jump={int(self._gps_jump_detected)} "
                    f"speed_kmh={self._speed_estimator.get_speed_kmh():.1f}"
                )
                # Publish a user-friendly event on real change (not heartbeat).
                if sig_changed:
                    try:
                        ue = self._build_user_event_msg(
                            self._current_state_name, avoid_state,
                            self._obstacle_mode, n_stop, n_slow, moving,
                            self._gps_jump_detected,
                        )
                        if ue and ue != self._last_user_event_msg:
                            self._last_user_event_msg = ue
                            m = String()
                            m.data = ue
                            self._log_info_pub.publish(m)
                    except Exception as exc:
                        self.get_logger().debug(f"user-event publish failed: {exc}")
        except Exception as e:
            self.get_logger().debug(f"[NAVTEL] emit skipped: {e}")

    def _build_user_event_msg(self, state_name, avoid_state, obstacle_mode,
                              n_stop, n_slow, moving, gps_jump):
        """Map the current control snapshot to a short end-user event sentence.

        Returned string is meant for the dashboard 'Ereignisse' ticker. Keep it
        in German, present-tense, and prefix with an emoji so it parses at a
        glance. Order matters: more critical conditions win over less critical.
        """
        # Sicherheits-Overrides zuerst
        if gps_jump:
            return "📡 GPS-Sprung erkannt. Stabilisiere Position."
        if (avoid_state or '').upper() == 'EMERGENCY_STOP' or n_stop > 0:
            return "🛑 Hindernis erkannt. Warte bis frei."

        state = (state_name or '').upper()
        if state == 'MANUAL':
            return "🎮 Manuelle Steuerung aktiv."
        if state == 'IDLE':
            return "💤 Bereit. Warte auf Auftrag."
        if state == 'NAVIGATING':
            if (avoid_state or '').upper() == 'SLOW_APPROACH' or n_slow > 0:
                return "🐢 Hindernis in der Nähe. Reduziere Geschwindigkeit."
            if not moving:
                return "🔍 Analysiere Umgebung."
            return "🤖 Fahrt aktiv. Strecke frei."
        if state == 'DOCKING':
            if n_stop > 0 or n_slow > 0:
                return "🔌 Andocken pausiert. Hindernis vor Ladestation."
            return "🔌 Andocke an Ladestation."
        if state == 'RETURNING_TO_HOME':
            return "🏠 Kehre zur Ladestation zurück."
        if state == 'CHARGING':
            return "⚡ Lade. Bereit nach voller Ladung."
        return f"ℹ️ Status: {state_name}"

    def _publish_user_event_tick(self):
        """Periodic narrator: emit the current friendly event sentence on change.

        Fail-soft. Runs at 1 Hz regardless of drive activity so MANUAL / IDLE
        states also produce dashboard messages.
        """
        try:
            try:
                avoid_state = self._obstacle_avoidance.get_current_state_name()
            except Exception:
                avoid_state = '?'
            n_stop = n_slow = 0
            sec_msg = self._obstacle_sectors_msg
            if sec_msg is not None:
                for s in sec_msg.sectors:
                    if s.blocked and s.sector_type == 'STOP':
                        n_stop += 1
                    elif s.blocked and s.sector_type == 'SLOW':
                        n_slow += 1
            try:
                speed_kmh = self._speed_estimator.get_speed_kmh()
            except Exception:
                speed_kmh = 0.0
            moving = speed_kmh > 0.3
            ue = self._build_user_event_msg(
                self._current_state_name, avoid_state,
                self._obstacle_mode, n_stop, n_slow, moving,
                self._gps_jump_detected,
            )
            if ue and ue != self._last_user_event_msg:
                self._last_user_event_msg = ue
                m = String()
                m.data = ue
                self._log_info_pub.publish(m)
        except Exception as exc:
            self.get_logger().debug(f"user-event tick skipped: {exc}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TacticalWpFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

