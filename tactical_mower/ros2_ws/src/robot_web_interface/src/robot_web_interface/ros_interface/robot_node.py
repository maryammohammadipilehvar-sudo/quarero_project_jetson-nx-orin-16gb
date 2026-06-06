"""ROS2 Robot Node - Main node class"""
import asyncio
import threading
import time
from typing import Dict, Any, Optional
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from fixposition_driver_msgs.msg import FusionEpoch
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Point, PoseStamped
from interfaces.msg import GeoPath, Joy, Schedule, SecurityAlert
from interfaces.srv import CommandControl, WaypointService, ScheduleService

from .status_manager import StatusManager
from .subscribers import SubscriberCallbacks
from .publishers import PublisherMethods
from ..utils.file_manager import load_settings
from ..utils.connection_manager import ConnectionManager


class RobotNode(Node):
    """ROS2 node with logging and command acknowledgment"""

    def __init__(self, connection_manager: ConnectionManager, event_loop: Optional[asyncio.AbstractEventLoop] = None, node_name: str = 'robot_web_interface'):
        super().__init__(node_name)
        self.get_logger().info('Initializing Robot Node (With Logging & Command ACK)')

        # Initialize components
        self.status_manager = StatusManager()
        self.connection_manager = connection_manager
        self.event_loop = event_loop
        self.subscribers = SubscriberCallbacks(self, self.status_manager, connection_manager, event_loop)
        # Use _publisher_methods to avoid conflict with Node.publishers property
        self._publisher_methods = PublisherMethods(self, self.status_manager)

        # Declare topic parameters
        self.declare_parameter('topics.camera', '/camera/camera/color/image_raw')
        self.declare_parameter('topics.waypoint_pub', '/geopath')
        self.declare_parameter('topics.light_pub', '/control/light')
        self.declare_parameter('topics.alarm_pub', '/control/alarm')
        self.declare_parameter('topics.siren_pub', '/control/siren')
        self.declare_parameter('topics.move_pub', '/control/move')
        self.declare_parameter('topics.emergency_stop_pub', '/control/emergency_stop')
        self.declare_parameter('topics.joy_web_pub', '/joy_web')
        self.declare_parameter('topics.joy_drive_raw', '/joy_drive_raw')
        self.declare_parameter('topics.fusion_status', '/fixposition/fusion')
        self.declare_parameter('topics.set_speed_pub', '/control/set_speed')
        self.declare_parameter('topics.autonomous_operation_pub', '/control/autonomous_operation')
    
        # Scheduler Topics
        self.declare_parameter('topics.schedule_add_pub', '/tactical/control/schedule/add')
        self.declare_parameter('topics.schedule_remove_pub', '/tactical/control/schedule/remove')
        self.declare_parameter('topics.robot_ack_sub', '/tactical/control/schedule/acknowledge')
        
        # Logging Topics
        self.declare_parameter('topics.logging_info_sub', '/tactical/logging/info')
        self.declare_parameter('topics.logging_warn_sub', '/tactical/logging/warn')
        self.declare_parameter('topics.logging_error_sub', '/tactical/logging/error')
        
        # Command Acknowledge Topic
        self.declare_parameter('topics.command_ack_sub', '/tactical/control/acknowledge')

        # Read parameters
        pos_topic = "/fixposition/odometry_llh"
        odom_topic = "/fixposition/odometry_enu"
        camera_topic = self.get_parameter('topics.camera').value
        robot_state_topic = "/robot/state"
        waypoint_topic = self.get_parameter('topics.waypoint_pub').value
        light_topic = self.get_parameter('topics.light_pub').value
        alarm_topic = self.get_parameter('topics.alarm_pub').value
        siren_topic = self.get_parameter('topics.siren_pub').value
        move_topic = self.get_parameter('topics.move_pub').value
        emergency_stop_topic = self.get_parameter('topics.emergency_stop_pub').value
        joy_web_topic = self.get_parameter('topics.joy_web_pub').value
        fusion_topic = self.get_parameter('topics.fusion_status').value
        set_speed_topic = self.get_parameter('topics.set_speed_pub').value
        autonomous_operation_topic = self.get_parameter('topics.autonomous_operation_pub').value
        go_to_charge_topic = "/control/go_to_charge_pos_and_charge"
        charge_manual_topic = "/control/charge_manual"
        # Scheduler topics
        schedule_add_topic = self.get_parameter('topics.schedule_add_pub').value
        schedule_remove_topic = self.get_parameter('topics.schedule_remove_pub').value
        robot_ack_topic = self.get_parameter('topics.robot_ack_sub').value

        # Logging topics
        logging_info_topic = self.get_parameter('topics.logging_info_sub').value
        logging_warn_topic = self.get_parameter('topics.logging_warn_sub').value
        logging_error_topic = self.get_parameter('topics.logging_error_sub').value
        
        # Command ACK topic
        command_ack_topic = self.get_parameter('topics.command_ack_sub').value
        
        # Joy controller topic
        joy_drive_raw_topic = self.get_parameter('topics.joy_drive_raw').value

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.subscribers.odom_callback,
            10
        )

        self.position_sub = self.create_subscription(
            NavSatFix,
            pos_topic,
            self.subscribers.position_callback,
            10
        )

        # Camera subscribers
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.camera_sub = self.create_subscription(
            Image,
            camera_topic,
            self.subscribers.camera_callback,
            camera_qos
        )
        self.thermal1_sub = self.create_subscription(
            Image, 
            '/ip_camera/thermal_raw', 
            self.subscribers.thermal1_callback, 
            camera_qos
        )
        
        self.thermal2_sub = self.create_subscription(
            Image,
            '/ip_camera/rgb_raw',
            self.subscribers.thermal2_callback,
            camera_qos
        )

        self.rgb2_sub = self.create_subscription(
            Image,
            '/ip_camera/rgb2_raw',
            self.subscribers.rgb2_callback,
            camera_qos
        )

        self.lidar_debug_sub = self.create_subscription(
            Image,
            '/obstacles/image',
            self.subscribers.lidar_debug_callback,
            camera_qos
        )

        self.depth_debug_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/obstacle_debug',
            self.subscribers.depth_debug_callback,
            camera_qos
        )

        # Robot state subscriber
        self.robot_state_sub = self.create_subscription(
            String,
            robot_state_topic,
            self.subscribers.robot_state_callback,
            10
        )
        self.get_logger().info(f'Subscribed to robot state topic: {robot_state_topic}')

        # ArUco dock marker (fused two-marker pose) for the in-app dock setup tool.
        self.dock_marker_sub = self.create_subscription(
            PoseStamped,
            '/docking/aruco_pose',
            self._dock_marker_callback,
            10
        )

        # Schedule ACK subscriber
        self.robot_ack_sub = self.create_subscription(
            Bool,
            robot_ack_topic,
            self.robot_ack_callback,
            10
        )
        
        # Logging Subscribers
        self.logging_info_sub = self.create_subscription(
            String,
            logging_info_topic,
            lambda msg: self.subscribers.logging_callback(msg, 'info'),
            10
        )
        self.logging_warn_sub = self.create_subscription(
            String,
            logging_warn_topic,
            lambda msg: self.subscribers.logging_callback(msg, 'warn'),
            10
        )
        self.logging_error_sub = self.create_subscription(
            String,
            logging_error_topic,
            lambda msg: self.subscribers.logging_callback(msg, 'error'),
            10
        )
        
        # Command ACK Subscriber
        self.command_ack_sub = self.create_subscription(
            String,
            command_ack_topic,
            self.command_ack_callback,
            10
        )

        # Fusion subscriber
        self.fusion_sub = self.create_subscription(
            FusionEpoch,           
            fusion_topic,
            self.subscribers.fusion_callback,
            10
        )
        
        # Waypoint save trigger (Circle button on PS5 or web button)
        self.save_waypoint_sub = self.create_subscription(
            Bool,
            '/control/save_waypoint',
            self._save_waypoint_callback,
            10
        )

        # Security alert subscriber
        self.security_alert_sub = self.create_subscription(
            SecurityAlert,
            '/security_alert',
            self.subscribers.security_alert_callback,
            10
        )
        
        # Speed estimate subscriber (filtered km/h from tactical_wp_follower)
        self.speed_estimate_sub = self.create_subscription(
            Float32,
            '/tactical/robot/speed_kmh',
            self.subscribers.speed_estimate_callback,
            10
        )
        
        # Joy controller subscriber (for select button detection)
        self.joy_sub = self.create_subscription(
            Joy,
            joy_drive_raw_topic,
            self._joy_select_callback,
            10
        )
        
        # Select button state tracking
        self._select_button_prev = False
        self._select_button_last_press_time = 0.0
        self._select_button_cooldown = 3.0  # 3 seconds cooldown

        # Create publishers
        self.waypoint_pub = self.create_publisher(GeoPath, waypoint_topic, 10)
        self.light_pub = self.create_publisher(Bool, light_topic, 10)
        self.alarm_pub = self.create_publisher(Bool, alarm_topic, 10)
        self.siren_pub = self.create_publisher(Bool, siren_topic, 10)
        self.move_pub = self.create_publisher(Point, move_topic, 10)
        self.emergency_stop_pub = self.create_publisher(Bool, emergency_stop_topic, 10)
        self.joy_web_pub = self.create_publisher(Joy, joy_web_topic, 10)
        self.set_speed_pub = self.create_publisher(Float32, set_speed_topic, 10)
        self.autonomous_operation_pub = self.create_publisher(Bool, autonomous_operation_topic, 10)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.obstacle_avoidance_pub = self.create_publisher(Bool, '/control/obstacle_avoidance_enabled', latched_qos)
        self.speed_factor = 1.0

        # Schedule Publishers
        self.schedule_add_pub = self.create_publisher(Schedule, schedule_add_topic, 10)
        self.schedule_remove_pub = self.create_publisher(String, schedule_remove_topic, 10)
        self.go_to_charge_pub = self.create_publisher(Bool, go_to_charge_topic, 10)
        self.charge_manual_pub = self.create_publisher(Bool, charge_manual_topic, 10)
        
        # Service clients for command control
        self.light_service_client = self.create_client(CommandControl, '/control/light')
        self.alarm_service_client = self.create_client(CommandControl, '/control/alarm')
        self.siren_service_client = self.create_client(CommandControl, '/control/siren')
        self.emergency_stop_service_client = self.create_client(CommandControl, '/control/emergency_stop')
        self.autonomous_operation_service_client = self.create_client(CommandControl, '/control/autonomous_operation')
        self.go_to_charge_service_client = self.create_client(CommandControl, '/control/go_to_charge_pos_and_charge')
        self.charge_manual_service_client = self.create_client(CommandControl, '/control/charge_manual')
        self.waypoint_service_client = self.create_client(WaypointService, '/control/waypoints')
        self.schedule_service_client = self.create_client(ScheduleService, '/tactical/control/schedule')
        
        # Remove command_id publisher - no longer needed with services
        # Schedule ACK handling
        self.last_ack_received = None
        self.last_ack_lock = threading.Lock()
        
        # Command ACK handling
        self.pending_commands: Dict[str, asyncio.Future] = {}
        self.pending_commands_lock = threading.Lock()

        self.joy_zero_timer = self.create_timer(0.05, self._joy_zero_timer_cb)
        
        # Connection status monitoring timer (check every 2 seconds)
        self.connection_check_timer = self.create_timer(2.0, self._check_connection_status)
        
        try:
            settings = load_settings()
            self.speed_factor = float(settings.get("speed_factor", 1.0))
            self._publisher_methods.publish_speed_factor(self.speed_factor)
            obstacle_avoidance_enabled = settings.get("enable_obstacle_avoidance", True)
            self._publisher_methods.publish_obstacle_avoidance_enabled(obstacle_avoidance_enabled)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize speed factor from settings: {e}")

        # Initial autonomous operation state is False by default
        # We can't call the async service during __init__, but the robot defaults to False

        self.get_logger().info('Robot Node initialized')

    def _joy_zero_timer_cb(self) -> None:
        if self.status_manager.emergency_active:
            try:
                self._publisher_methods.publish_joy_command(0, 0)
            except Exception as e:
                self.get_logger().error(f'Failed to publish zero Joy command: {e}')
    
    def _check_connection_status(self) -> None:
        """Periodically check robot connection status"""
        CONNECTION_TIMEOUT = 5.0  # Consider disconnected if no message for 5 seconds
        
        with self.status_manager.connection_status_lock:
            current_time = time.time()
            last_message_time = self.status_manager.last_robot_state_time
            
            if last_message_time is None:
                # No message received yet
                if self.status_manager.robot_connected:
                    self.status_manager.robot_connected = False
                    self.subscribers._broadcast_connection_status(False)
            else:
                time_since_last = current_time - last_message_time
                if time_since_last > CONNECTION_TIMEOUT:
                    # Connection lost
                    if self.status_manager.robot_connected:
                        self.status_manager.robot_connected = False
                        self.subscribers._broadcast_connection_status(False)
                else:
                    # Connection active
                    if not self.status_manager.robot_connected:
                        self.status_manager.robot_connected = True
                        self.subscribers._broadcast_connection_status(True)

    def _save_waypoint_callback(self, msg: Bool) -> None:
        pos = self.status_manager.current_position
        lat = pos.get("latitude", 0.0)
        lon = pos.get("longitude", 0.0)
        if lat == 0.0 and lon == 0.0:
            self.get_logger().warn("Waypoint save triggered but no GPS fix")
            return
        self.get_logger().info(f"Waypoint save: {lat:.7f}, {lon:.7f}")
        if self.event_loop and self.connection_manager:
            self.event_loop.call_soon_threadsafe(
                self.event_loop.create_task,
                self.connection_manager.broadcast({
                    "type": "waypoint_saved",
                    "data": {"latitude": lat, "longitude": lon}
                })
            )

    def robot_ack_callback(self, msg: Bool) -> None:
        with self.last_ack_lock:
            self.last_ack_received = msg.data
            self.get_logger().info(f'Received robot ACK: {msg.data}')

    def command_ack_callback(self, msg: String) -> None:
        try:
            import json
            data = json.loads(msg.data)
            command_id = data.get("id")
            success = data.get("success", False)
            message = data.get("message", "")
            
            self.get_logger().info(f'Command ACK received: {command_id} -> {success}, pending commands: {list(self.pending_commands.keys())}')
            
            with self.pending_commands_lock:
                if command_id in self.pending_commands:
                    future = self.pending_commands[command_id]
                    if not future.done():
                        # Set result in the correct event loop thread
                        if self.event_loop is not None and self.event_loop.is_running():
                            self.get_logger().info(f'Setting result for command_id {command_id} via event loop')
                            self.event_loop.call_soon_threadsafe(
                                future.set_result, 
                                {"success": success, "message": message}
                            )
                        else:
                            # Fallback if event loop not available
                            self.get_logger().warn(f'Event loop not available, setting result directly for {command_id}')
                            future.set_result({"success": success, "message": message})
                    else:
                        self.get_logger().warn(f'Future for {command_id} already done')
                    del self.pending_commands[command_id]
                else:
                    self.get_logger().warn(f'Command ACK received for unknown command_id: {command_id}')
            
            self.get_logger().info(f'Command ACK processed: {command_id} -> {success}')
        except Exception as e:
            self.get_logger().error(f'Command ACK parse error: {e}', exc_info=True)

    # Delegate methods to components
    def get_robot_state(self) -> Dict[str, Any]:
        return self.status_manager.get_robot_state()

    def get_position(self) -> Dict[str, Any]:
        return self.status_manager.get_position()

    def get_frame(self):
        """Get main camera frame. Returns (frame, timestamp) or (None, 0.0)"""
        return self.status_manager.get_frame()

    def get_thermal1_frame(self):
        """Get thermal1 frame. Returns (frame, timestamp) or (None, 0.0)"""
        return self.status_manager.get_thermal1_frame()

    def get_thermal2_frame(self):
        """Get thermal2 frame. Returns (frame, timestamp) or (None, 0.0)"""
        return self.status_manager.get_thermal2_frame()

    def get_rgb2_frame(self):
        """Get rgb2 frame (Axis channel 1). Returns (frame, timestamp) or (None, 0.0)"""
        return self.status_manager.get_rgb2_frame()

    def get_lidar_debug_frame(self):
        """Get lidar debug frame. Returns (frame, timestamp) or (None, 0.0)"""
        return self.status_manager.get_lidar_debug_frame()

    def get_depth_debug_frame(self):
        """Get lidar debug frame. Returns (frame, timestamp) or (None, 0.0)"""
        return self.status_manager.get_depth_debug_frame()

    def get_fusion_state(self) -> Dict[str, Any]:
        return self.status_manager.get_fusion_state()

    def _dock_marker_callback(self, msg: PoseStamped) -> None:
        """Cache the latest fused dock-marker pose for the dock-setup tool."""
        self.status_manager.dock_marker_pose = msg.pose
        self.status_manager.dock_marker_time = time.time()

    def get_dock_align(self) -> Dict[str, Any]:
        """Live dock alignment (cross/heading/perp + aligned) for the UI.

        Green-light tolerances come from aruco_dock.yaml (live-tunable, no rebuild)."""
        from ..services.dock_align import compute_align
        from ..utils.file_manager import load_aruco_dock
        pose = self.status_manager.dock_marker_pose
        t = self.status_manager.dock_marker_time
        age = (time.time() - t) if t else None
        try:
            cfg = load_aruco_dock()
            cross_tol = float(cfg.get("setup_cross_tol_cm", 6.0)) / 100.0
            head_tol = float(cfg.get("setup_heading_tol_deg", 10.0))
        except Exception:
            cross_tol, head_tol = 0.06, 10.0
        return compute_align(pose, age, cross_tol_m=cross_tol, head_tol_deg=head_tol)

    def publish_waypoints(self, waypoints, loop_mode: bool = False, command_id: str = None, route_name: str = '') -> None:
        self._publisher_methods.publish_waypoints(waypoints, loop_mode, command_id, route_name)

    def publish_go_to_charge(self, state: bool, command_id: str = None) -> None:
        self._publisher_methods.publish_go_to_charge(state, command_id)

    def publish_charge_manual(self, state: bool, command_id: str = None) -> None:
        self._publisher_methods.publish_charge_manual(state, command_id)

    def publish_schedule_add(self, schedule_config: Dict) -> None:
        self._publisher_methods.publish_schedule_add(schedule_config)

    def publish_schedule_remove(self, schedule_id: str) -> None:
        self._publisher_methods.publish_schedule_remove(schedule_id)
    
    async def wait_for_ack(self, timeout: float = 10.0) -> bool:
        start_time = asyncio.get_event_loop().time()
        
        while (asyncio.get_event_loop().time() - start_time) < timeout:
            with self.last_ack_lock:
                if self.last_ack_received is not None:
                    ack_result = self.last_ack_received
                    self.last_ack_received = None
                    return ack_result
            await asyncio.sleep(0.1)
        
        self.get_logger().error(f'No ACK received within {timeout}s')
        return False
    
    async def register_command_ack(self, command_id: str) -> asyncio.Future:
        """Register a future for command ACK. Call this BEFORE publishing the command."""
        loop = asyncio.get_event_loop()
        future = loop.create_future()
        
        with self.pending_commands_lock:
            self.pending_commands[command_id] = future
            self.get_logger().info(f'Registered command ACK future for: {command_id}, total pending: {len(self.pending_commands)}')
        
        return future
    
    async def wait_for_command_ack(self, command_id: str, timeout: float = 10.0) -> Dict:
        """Wait for command ACK. Future should already be registered via register_command_ack()."""
        with self.pending_commands_lock:
            future = self.pending_commands.get(command_id)
            if future is None:
                # Future not registered - this shouldn't happen
                self.get_logger().error(f'Command ACK future not registered for: {command_id}, available: {list(self.pending_commands.keys())}')
                return {"success": False, "message": "Command ACK future not registered"}
        
        self.get_logger().info(f'Waiting for command ACK: {command_id}, timeout: {timeout}s')
        try:
            result = await asyncio.wait_for(future, timeout=timeout)
            self.get_logger().info(f'Command ACK received for {command_id}: {result}')
            return result
        except asyncio.TimeoutError:
            self.get_logger().error(f'Timeout waiting for command ACK: {command_id}')
            with self.pending_commands_lock:
                if command_id in self.pending_commands:
                    del self.pending_commands[command_id]
            return {"success": False, "message": "Timeout - Keine Antwort vom Roboter"}

    def publish_joy_command(self, left_stick_forward: int, right_stick_left: int) -> None:
        self._publisher_methods.publish_joy_command(left_stick_forward, right_stick_left)

    async def set_light(self, state: bool, command_id: str = None) -> Dict:
        """Set light state via service."""
        return await self._publisher_methods.call_light_service(state, command_id)

    async def set_alarm(self, state: bool, command_id: str = None) -> Dict:
        """Set alarm state via service."""
        return await self._publisher_methods.call_alarm_service(state, command_id)
    
    async def set_siren(self, state: bool, command_id: str = None) -> Dict:
        """Set siren state via service."""
        return await self._publisher_methods.call_siren_service(state, command_id)
    
    async def trigger_emergency_stop_service(self, state: bool, command_id: str = None) -> Dict:
        """Trigger emergency stop via service."""
        return await self._publisher_methods.call_emergency_stop_service(state, command_id)
    
    async def set_autonomous_operation(self, enabled: bool, command_id: str = None) -> Dict:
        """Set autonomous operation via service."""
        return await self._publisher_methods.call_autonomous_operation_service(enabled, command_id)
    
    async def publish_go_to_charge(self, state: bool, command_id: str = None) -> Dict:
        """Go to charge position via service."""
        return await self._publisher_methods.call_go_to_charge_service(state, command_id)
    
    async def publish_charge_manual(self, state: bool, command_id: str = None) -> Dict:
        """Charge manual relay via service."""
        return await self._publisher_methods.call_charge_manual_service(state, command_id)
    
    async def publish_waypoints(self, waypoints, loop_mode: bool = False, command_id: str = None, route_name: str = '') -> Dict:
        """Publish waypoints via service."""
        return await self._publisher_methods.call_waypoint_service(waypoints, loop_mode, command_id, route_name)
    
    async def publish_schedule_add(self, schedule_config: Dict, command_id: str = None) -> Dict:
        """Add schedule via service."""
        return await self._publisher_methods.call_schedule_service("add", schedule_config, None, command_id)
    
    async def publish_schedule_remove(self, schedule_id: str, command_id: str = None) -> Dict:
        """Remove schedule via service."""
        return await self._publisher_methods.call_schedule_service("remove", None, schedule_id, command_id)
    
    async def publish_schedule_update(self, schedule_config: Dict, old_schedule_id: str, command_id: str = None) -> Dict:
        """Update schedule via service (typically for activation/deactivation toggle)."""
        return await self._publisher_methods.call_schedule_service("update", schedule_config, old_schedule_id, command_id)

    def publish_speed_factor(self, factor: float) -> None:
        self._publisher_methods.publish_speed_factor(factor)

    def publish_obstacle_avoidance_enabled(self, enabled: bool) -> None:
        self._publisher_methods.publish_obstacle_avoidance_enabled(enabled)

    def send_move_command(self, x: float, y: float) -> None:
        self._publisher_methods.send_move_command(x, y)

    def trigger_emergency_stop(self) -> None:
        self._publisher_methods.trigger_emergency_stop()

    def clear_emergency_stop(self) -> None:
        self._publisher_methods.clear_emergency_stop()

    # Removed old synchronous set_autonomous_operation - now using async set_autonomous_operation method

    @property
    def light_status(self) -> bool:
        return self.status_manager.light_status

    @property
    def alarm_status(self) -> bool:
        return self.status_manager.alarm_status

    @property
    def siren_status(self) -> bool:
        return self.status_manager.siren_status

    @property
    def emergency_active(self) -> bool:
        return self.status_manager.emergency_active

    @property
    def autonomous_enabled(self) -> bool:
        return self.status_manager.autonomous_enabled
    
    def _joy_select_callback(self, msg: Joy) -> None:
        """Handle select button press from joy controller.
        
        Detects select button edge (False -> True) and broadcasts to web app
        if cooldown period has passed.
        """
        current_time = time.time()
        select_pressed = msg.select
        
        # Edge detection: button just pressed (was False, now True)
        if select_pressed and not self._select_button_prev:
            # Check cooldown
            time_since_last = current_time - self._select_button_last_press_time
            if time_since_last >= self._select_button_cooldown:
                # Cooldown expired, process the button press
                self._select_button_last_press_time = current_time
                
                # Get current position
                position = self.status_manager.get_position()
                
                # Broadcast to web app via WebSocket
                try:
                    if self.event_loop is not None and self.event_loop.is_running():
                        asyncio.run_coroutine_threadsafe(
                            self._broadcast_select_button_press(position),
                            self.event_loop
                        )
                    else:
                        self.get_logger().warn("Could not broadcast select button press: event loop not available")
                except Exception as e:
                    self.get_logger().error(f'Select button broadcast error: {e}')
        
        # Update previous state
        self._select_button_prev = select_pressed
    
    async def _broadcast_select_button_press(self, position: Dict[str, Any]) -> None:
        """Broadcast select button press event to all WebSocket clients."""
        message = {
            "type": "select_button_pressed",
            "data": {
                "latitude": position.get("latitude", 0.0),
                "longitude": position.get("longitude", 0.0),
                "altitude": position.get("height", 0.0),
                "timestamp": time.time()
            }
        }
        await self.connection_manager.broadcast(message)
        self.get_logger().info(f"Broadcasted select button press at position: {position.get('latitude', 0.0):.6f}, {position.get('longitude', 0.0):.6f}")

