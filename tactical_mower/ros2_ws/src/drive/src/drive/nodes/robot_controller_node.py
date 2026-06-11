#!/usr/bin/env python3
"""ROS2 node for controlling a differential drive robot.

This node orchestrates hardware interfaces, controllers, and services
to handle manual driving via joystick input, emergency stops, and
peripheral control (lights, charging).

UNIFIED CHARGING TRACKING SYSTEM:
This node maintains the single source of truth for the robot's charging state
via the _charging_state variable. This state is:
- Updated when /control/enable_charging messages are received
- Published in the /robot/state topic as 'charging_state' field
- Accessible via the is_charging() helper method
All other nodes should consume charging state from /robot/state rather than
maintaining their own separate tracking.
"""

import json
import math
import rclpy
from rclpy.node import Node
from pathlib import Path
from typing import Optional
from time import time

from interfaces.msg import CommandDrive, Joy, GeoPath
from interfaces.srv import CommandControl
from std_msgs.msg import String, Float32, Bool, Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from fixposition_driver_msgs.msg import Speed, WheelSensor

from ..hardware.uart_device import UartDevice
from ..kinematics.differential_drive import DifferentialDriveKinematics
from ..controllers.joystick_controller import JoystickController
from ..controllers.emergency_stop_controller import EmergencyStopController
from ..controllers.speed_controller import SpeedController
from ..services.light_service import LightService
from ..services.charging_service import ChargingService

# Import geo helper from control package
try:
    from control.navigation.geo_helper import calculate_route_distance_from_waypoints
except ImportError:
    # Fallback if control package is not available
    calculate_route_distance_from_waypoints = None


class RobotControllerNode(Node):
    """ROS2 node for robot control - orchestrates components."""

    def __init__(self):
        """Initialize the RobotController node."""
        super().__init__('robot_controller')

        # Declare parameters
        self.declare_parameter('wheel_diameter', 0.535)
        self.declare_parameter('wheel_base', 0.637)
        self.declare_parameter('flip_steering', False)
        self.declare_parameter('topic_input', '/joy_drive_raw')
        self.declare_parameter('topic_web_input', '/joy_web')
        self.declare_parameter('gamepad_timeout', 0.5)  # Use web input if gamepad inactive for 500ms
        self.declare_parameter('topic_output', '/cmd_drive')
        self.declare_parameter('topic_robot_state', '/robot/state')
        self.declare_parameter('topic_fixposition_speed', '/fixposition/speed')
        self.declare_parameter('topic_light_control', '/control/light')
        self.declare_parameter('uart_port', '/dev/ttyTHS1')
        self.declare_parameter('uart_baudrate', 19200)
        self.declare_parameter('settings_file', '/routen/settings/settings.yaml')

        # Load parameters
        wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.topic_input = self.get_parameter('topic_input').get_parameter_value().string_value
        self.topic_web_input = self.get_parameter('topic_web_input').get_parameter_value().string_value
        self.gamepad_timeout = self.get_parameter('gamepad_timeout').get_parameter_value().double_value
        self.topic_output = self.get_parameter('topic_output').get_parameter_value().string_value
        self.topic_robot_state = self.get_parameter('topic_robot_state').get_parameter_value().string_value
        self.topic_fixposition_speed = self.get_parameter('topic_fixposition_speed').get_parameter_value().string_value
        self.topic_light_control = self.get_parameter('topic_light_control').get_parameter_value().string_value
        uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        uart_baudrate = self.get_parameter('uart_baudrate').get_parameter_value().integer_value
        flip_steering = self.get_parameter('flip_steering').get_parameter_value().bool_value
        settings_file = Path(self.get_parameter('settings_file').get_parameter_value().string_value).expanduser()

        # Initialize components
        # 1. Hardware - UART is managed by joy_controller_node, not here
        # Light and charging commands are sent via ROS topics to joy_controller_node
        # which forwards them to ESP32 via UART
        self._uart_device = None  # UART is not opened here to avoid conflicts
        self.get_logger().info("UART is managed by joy_controller_node - light/charging via ROS topics")

        # 2. Kinematics
        self._kinematics = DifferentialDriveKinematics(wheel_diameter, wheel_base)

        # 3. Controllers
        self._speed_controller = SpeedController(default_max_speed=1.0)
        if settings_file.exists():
            if self._speed_controller.load_from_settings(settings_file):
                self.get_logger().info(f"Loaded max_speed from settings: {self._speed_controller.get_max_speed()}")
            else:
                self.get_logger().warning(f"Settings file found but speed_factor not loaded, using default")

        self._emergency_stop = EmergencyStopController()
        self._joystick_controller = JoystickController(
            self._kinematics,
            self._speed_controller,
            flip_steering
        )

        # 4. Services - UART is managed by joy_controller_node
        # Light/charging commands are published to topics, not sent directly via UART

        # Publishers
        self._cmd_pub = self.create_publisher(CommandDrive, self.topic_output, 10)
        self._state_pub = self.create_publisher(String, self.topic_robot_state, 10)
        self._fixposition_speed_pub = self.create_publisher(Speed, self.topic_fixposition_speed, 10)
        self._ack_pub = self.create_publisher(Bool, '/tactical/control/schedule/acknowledge', 10)
        self._light_pub = self.create_publisher(Bool, self.topic_light_control, 10)
        # Log publishers (for web UI)
        self._log_info_pub = self.create_publisher(String, '/tactical/logging/info', 10)
        self._log_warn_pub = self.create_publisher(String, '/tactical/logging/warn', 10)
        self._log_error_pub = self.create_publisher(String, '/tactical/logging/error', 10)

        # Subscribers - both gamepad and web joystick
        self.create_subscription(Joy, self.topic_input, self._joy_gamepad_callback, 10)
        self.create_subscription(Joy, self.topic_web_input, self._joy_web_callback, 10)
        self.create_subscription(NavSatFix, '/fixposition/odometry_llh', self._gps_callback, 10)
        self.create_subscription(Float32, '/drive/battery_percentage', self._battery_percentage_callback, 10)
        self.create_subscription(Float32, '/drive/runtime_estimate', self._runtime_estimate_callback, 10)
        self.create_subscription(GeoPath, '/geopath', self._geopath_callback, 10)
        # Mirror actual light commands (from the web light service) into light_state
        # so /robot/state stays truthful no matter who toggled the light.
        self.create_subscription(Bool, self.topic_light_control, self._light_state_sync_callback, 10)
        self.create_service(CommandControl, '/control/light', self._light_control_service)
        self.create_service(CommandControl, '/control/alarm', self._alarm_control_service)
        self.create_service(CommandControl, '/control/siren', self._siren_control_service)
        self.create_service(CommandControl, '/control/emergency_stop', self._emergency_stop_service)
        
        # Topic subscriptions
        self.create_subscription(Float32, '/control/set_speed', self._set_speed_callback, 10)
        self.create_subscription(Bool, '/control/enable_charging', self._enable_charging_callback, 10)
        # Subscribe to autonomous operation state (published by tactical_wp_follower_node)
        self.create_subscription(Bool, '/control/autonomous_operation', self._autonomous_operation_callback, 10)
        # Subscribe to tactical robot state to get active route information
        self.create_subscription(String, '/tactical/robot/state', self._tactical_robot_state_callback, 10)

        # State variables
        self._tactical_mode_active = False
        self._autonomous_enabled = False  # Track autonomous operation state
        self._last_gps: Optional[NavSatFix] = None
        self._current_speed: Optional[float] = None
        self._current_battery_percentage: Optional[float] = None
        self._current_runtime_estimate: Optional[float] = None  # Runtime estimate in seconds
        self._last_left_vel = 0.0
        self._last_right_vel = 0.0
        self._last_gamepad_time: float = 0.0  # Track when last gamepad message was received
        self._pending_web_joy: Optional[Joy] = None  # Store pending web joystick command
        self._last_web_joy_time: float = 0.0  # Track when last web joy message arrived
        self._web_hold_timeout: float = 0.5  # Keep republishing last web cmd for up to N seconds (dashboard heartbeats at 150ms, this is the safety stop window)
        
        # Button state tracking (single source of truth)
        self._light_state = False
        self._alarm_state = False
        self._siren_state = False
        self._charging_state = False  # Charging relay state
        
        # Remove command ID tracking - services handle this automatically
        
        # Route tracking
        self._active_route_waypoints: list = []  # List of waypoint dicts with lat, lon, alt
        self._active_route_mode: str = 'none'  # 'none', 'once', 'loop', 'ping_pong'
        self._active_route_name: str = ''  # Name of the active route
        # Index of the waypoint the follower is currently targeting (mirrors the value
        # published by tactical_wp_follower in /tactical/robot/state). None when no
        # mission is loaded; 0 .. len(waypoints)-1 while a route is active.
        self._active_route_current_wp_idx: Optional[int] = None

        # Timer for state publishing and web joystick processing
        self.create_timer(1.0, self._publish_state)
        self.create_timer(0.05, self._process_web_joy_if_allowed)  # Check every 50ms if web input allowed

        self.get_logger().info("RobotControllerNode initialized")
        self.get_logger().info(f"Subscribing to gamepad: {self.topic_input}, web: {self.topic_web_input}")

    def _log_info(self, text: str):
        """Publish info log."""
        msg = String()
        msg.data = text
        self._log_info_pub.publish(msg)
        self.get_logger().info(f"Sent: {text}")

    def _log_warning(self, text: str):
        """Publish warning log."""
        msg = String()
        msg.data = text
        self._log_warn_pub.publish(msg)
        self.get_logger().warning(f"Sent: {text}")

    def _log_error(self, text: str):
        """Publish error log."""
        msg = String()
        msg.data = text
        self._log_error_pub.publish(msg)
        self.get_logger().error(f"Sent: {text}")

    def _joy_gamepad_callback(self, msg: Joy):
        """Handle joystick input from gamepad (priority source).
        
        Args:
            msg: Joystick message from gamepad
        """
        # Update last gamepad message time - this establishes priority
        self._last_gamepad_time = time()
        
        # Clear any pending web joystick commands when gamepad is active
        self._pending_web_joy = None
        
        # Process gamepad input (same as before)
        self._process_joy_command(msg, source="gamepad")

    def _joy_web_callback(self, msg: Joy):
        """Handle joystick input from web interface (lower priority).

        Args:
            msg: Joystick message from web interface
        """
        # Store web joystick command - will be processed if gamepad is inactive
        self._pending_web_joy = msg
        self._last_web_joy_time = time()

    def _process_web_joy_if_allowed(self):
        """Process pending web joystick command if gamepad is inactive.

        Web sends one-shot POSTs (no continuous stream), but the roboclaw
        watchdog brakes after ~300ms of silence on /cmd_drive. To keep manual
        driving smooth we re-publish the last web command at timer rate until
        either a newer one arrives or _web_hold_timeout has passed.
        """
        now = time()
        time_since_gamepad = now - self._last_gamepad_time
        if time_since_gamepad <= self.gamepad_timeout:
            return  # gamepad has priority

        if self._pending_web_joy is None:
            return

        # Expire the held command if web went silent (button released without
        # an explicit zero-msg, connection dropped, etc.) — safety net.
        if (now - self._last_web_joy_time) > self._web_hold_timeout:
            self._pending_web_joy = None
            return

        # Re-publish the last web command — robot keeps moving smoothly until
        # web either updates the command or stops sending.
        self._process_joy_command(self._pending_web_joy, source="web")

    def _process_joy_command(self, msg: Joy, source: str = "unknown"):
        """Process joystick command and publish drive commands.

        Args:
            msg: Joystick message
            source: Source of joystick input ("gamepad" or "web")
        """
        # Check emergency stop first
        if self._emergency_stop.is_active():
            self.get_logger().warn("Emergency stop active - joystick ignored")
            left_vel, right_vel = self._emergency_stop.get_stop_command()
        # Ignore manual input when charging is active (safety feature)
        elif self._charging_state:
            self._log_warning("⚠️ Die manuelle Steuerung ist während des Ladevorgangs deaktiviert.")
            return
        # Ignore manual input in tactical mode
        elif self._autonomous_enabled:
            self.get_logger().debug("Tactical mode active, ignoring manual joystick input")
            return
        else:
            # Process joystick input
            left_vel, right_vel = self._joystick_controller.process_joystick(msg)
            
        # Publish drive command
        cmd = CommandDrive()
        cmd.left_vel = left_vel
        cmd.right_vel = right_vel
        self._cmd_pub.publish(cmd)

        self._last_left_vel = left_vel
        self._last_right_vel = right_vel

    # Removed _emergency_stop_callback - now handled by service

    def _light_state_sync_callback(self, msg: Bool):
        """Keep light_state in sync with the actual /control/light command."""
        self._light_state = bool(msg.data)

    def _light_control_service(self, request, response):
        """Handle light control service request.
        
        Args:
            request: CommandControl request with command_id and state
            response: CommandControl response
        """
        state = request.state
        command_id = request.command_id
        
        self.get_logger().info(f"Light service called: {'ON' if state else 'OFF'} (command_id: {command_id})")
        
        # Update tracked state (single source of truth)
        self._light_state = state
        
        # Forward command to joy_controller_node via topic (for ESP32)
        # joy_controller_node subscribes to /control/light topic
        light_msg = Bool()
        light_msg.data = state
        self._light_pub.publish(light_msg)
        self.get_logger().info(f"Light {'ON' if state else 'OFF'} command forwarded to joy_controller_node")
        
        response.success = True
        response.message = f"Light {'ON' if state else 'OFF'}"
        self.get_logger().info(f"Light service response: {response.message} (command_id: {command_id})")
        return response
    
    def _alarm_control_service(self, request, response):
        """Handle alarm control service request."""
        state = request.state
        command_id = request.command_id
        
        self.get_logger().info(f"Alarm service called: {'ON' if state else 'OFF'} (command_id: {command_id})")
        
        # Update tracked state (single source of truth)
        self._alarm_state = state
        
        # Alarm control: state is tracked but actual hardware control not yet implemented
        response.success = True
        response.message = f"Alarm {'ON' if state else 'OFF'}"
        return response
    
    def _siren_control_service(self, request, response):
        """Handle siren control service request."""
        state = request.state
        command_id = request.command_id
        
        self.get_logger().info(f"Siren service called: {'ON' if state else 'OFF'} (command_id: {command_id})")
        
        # Update tracked state (single source of truth)
        self._siren_state = state
        
        # Siren control: state is tracked but actual hardware control not yet implemented
        response.success = True
        response.message = f"Siren {'ON' if state else 'OFF'}"
        return response
    
    def _emergency_stop_service(self, request, response):
        """Handle emergency stop service request."""
        state = request.state
        command_id = request.command_id
        
        self.get_logger().info(f"Emergency stop service called: {'ACTIVATE' if state else 'DEACTIVATE'} (command_id: {command_id})")
        
        if state:
            self._emergency_stop.activate()
            cmd = CommandDrive()
            cmd.left_vel = 0.0
            cmd.right_vel = 0.0
            self._cmd_pub.publish(cmd)
            self.get_logger().warn("EMERGENCY STOP ACTIVATED - Motors forced to 0")
            response.message = "Emergency stop activated"
        else:
            self._emergency_stop.deactivate()
            self.get_logger().info("Emergency stop released")
            response.message = "Emergency stop deactivated"
        
        response.success = True
        return response

    def _enable_charging_callback(self, msg: Bool):
        """Handle charging enable/disable command.

        The command is forwarded to joy_controller_node via ROS topic,
        which sends it to ESP32 via UART. This node no longer handles UART directly.

        Args:
            msg: Charging control message (True = enable, False = disable)
        """
        # Update charging state tracking (single source of truth)
        self._charging_state = msg.data
        
        # Command is automatically forwarded by joy_controller_node which subscribes
        # to the same topic. We just acknowledge receipt.
        self.get_logger().info(f"Charging {'ENABLED' if msg.data else 'DISABLED'} command received (forwarded via joy_controller_node)")

    def _set_speed_callback(self, msg: Float32):
        """Handle max speed update.

        Args:
            msg: New max speed value
        """
        self._speed_controller.set_max_speed(msg.data)
        self.get_logger().info(f"Max speed updated to: {msg.data}")

    def _autonomous_operation_callback(self, msg: Bool):
        """Handle autonomous operation state update.
        
        Args:
            msg: Bool message (True = autonomous enabled, False = autonomous disabled)
        """
        self._autonomous_enabled = msg.data
        self.get_logger().info(f"Autonomous operation state updated: {self._autonomous_enabled}")

    def _gps_callback(self, msg: NavSatFix):
        """Store latest GPS position.

        Args:
            msg: GPS fix message
        """
        self._last_gps = msg

    def _battery_percentage_callback(self, msg: Float32):
        """Store latest battery percentage.

        Args:
            msg: Battery percentage message
        """
        self._current_battery_percentage = msg.data
    
    def _runtime_estimate_callback(self, msg: Float32):
        """Store latest runtime estimate.

        Args:
            msg: Runtime estimate message in seconds (-1.0 if unavailable)
        """
        self._current_runtime_estimate = msg.data if msg.data >= 0.0 else None
    
    def _calculate_route_distance(self, waypoints: list, mode: str) -> float:
        """Calculate total distance of a route in meters.
        
        Args:
            waypoints: List of waypoint dicts with latitude, longitude, altitude
            mode: Route mode ('once', 'loop', 'ping_pong', 'none')
            
        Returns:
            Total distance in meters
        """
        if calculate_route_distance_from_waypoints is not None:
            # Use helper function from control package
            return calculate_route_distance_from_waypoints(waypoints, mode)
        else:
            # Fallback: return 0 if helper is not available
            self.get_logger().warn("geo_helper not available, route distance calculation disabled")
            return 0.0
    
    def _tactical_robot_state_callback(self, msg: String):
        """Track active route from tactical robot state messages.
        
        Args:
            msg: JSON string with robot state including active_route
        """
        try:
            import json
            data = json.loads(msg.data)
            
            # Extract active_route if present
            if 'active_route' in data and data['active_route'] is not None:
                route = data['active_route']
                self._active_route_name = route.get('name', '')
                self._active_route_mode = route.get('mode', 'none')

                # Convert waypoints format (already in latitude/longitude/altitude format)
                waypoints = route.get('waypoints', [])
                self._active_route_waypoints = [
                    {
                        'latitude': wp.get('latitude', 0.0),
                        'longitude': wp.get('longitude', 0.0),
                        'altitude': wp.get('altitude', 0.0)
                    }
                    for wp in waypoints
                ]
                # Forward the live waypoint index from the follower.
                # Keep as None if upstream didn't include the field (older builds).
                raw_idx = route.get('current_waypoint_index', None)
                self._active_route_current_wp_idx = (
                    int(raw_idx) if raw_idx is not None else None
                )

                self.get_logger().debug(
                    f"Route updated from tactical state: '{self._active_route_name}' - {len(self._active_route_waypoints)} waypoints, mode={self._active_route_mode}, wp_idx={self._active_route_current_wp_idx}"
                )
            else:
                # No active route
                self._active_route_waypoints = []
                self._active_route_mode = 'none'
                self._active_route_current_wp_idx = None
                self._active_route_name = ''
        except Exception as e:
            self.get_logger().error(f"Error parsing tactical robot state: {e}")
    
    def _geopath_callback(self, msg: GeoPath):
        """Track active route from GeoPath messages.
        
        Args:
            msg: GeoPath message with waypoints and mode
        """
        # Map GeoPath mode constants to string representation
        mode_map = {
            GeoPath.STOP: 'none',
            GeoPath.ONCE: 'once',
            GeoPath.LOOP: 'loop',
            GeoPath.PING_PONG: 'ping_pong'
        }
        
        self._active_route_mode = mode_map.get(msg.mode, 'none')
        self._active_route_name = msg.route_name if hasattr(msg, 'route_name') else ''
        
        # Convert waypoints to list of dicts
        if msg.mode == GeoPath.STOP or len(msg.waypoints) == 0:
            self._active_route_waypoints = []
            self._active_route_mode = 'none'
            self._active_route_name = ''
            self._active_route_current_wp_idx = None
        else:
            self._active_route_waypoints = [
                {
                    'latitude': wp.x,
                    'longitude': wp.y,
                    'altitude': wp.z
                }
                for wp in msg.waypoints
            ]
            # New GeoPath received → assume Cold Start. The live value will be
            # corrected by the next /tactical/robot/state message once the
            # follower reports actual progress.
            self._active_route_current_wp_idx = 0
        
        self.get_logger().info(
            f"Route updated: '{self._active_route_name}' - {len(self._active_route_waypoints)} waypoints, mode={self._active_route_mode}"
        )

    def is_charging(self) -> bool:
        """Check if robot is currently charging.
        
        Returns:
            True if charging relay is enabled, False otherwise
        """
        return self._charging_state
    
    def _publish_state(self):
        """Publish robot state periodically."""
        # Calculate battery percentage
        battery_percentage = None
        if self._current_battery_percentage is not None:
            battery_percentage = self._current_battery_percentage
            battery_percentage = max(0.0, min(100.0, battery_percentage))

        # Calculate remaining runtime in minutes (rounded, no decimals)
        remaining_runtime_minutes = None
        if self._current_runtime_estimate is not None:
            remaining_runtime_minutes = int(round(self._current_runtime_estimate / 60.0))

        state_dict = {
            "battery_percentage": battery_percentage,
            "velocity": self._current_speed,
            "autonomous_mode": self._autonomous_enabled,
            "autonomous_enabled": self._autonomous_enabled,  # Autonomous operation state (can be toggled via web app)
            "error_status": "",
            # Button states (single source of truth)
            "light_state": self._light_state,
            "alarm_state": self._alarm_state,
            "siren_state": self._siren_state,
            "emergency_stop_active": self._emergency_stop.is_active(),
            "charging_state": self._charging_state,  # Charging relay state
            "remaining_runtime_minutes": remaining_runtime_minutes,  # Remaining operational time in minutes
            # Active route information
            "active_route": {
                "name": self._active_route_name,
                "waypoints": self._active_route_waypoints,
                "mode": self._active_route_mode,
                "current_waypoint_index": self._active_route_current_wp_idx,
                "distance_meters": self._calculate_route_distance(self._active_route_waypoints, self._active_route_mode) if self._active_route_waypoints else None
            }
        }

        msg = String()
        msg.data = json.dumps(state_dict)
        self._state_pub.publish(msg)
        self.get_logger().debug(f"Published state: {msg.data}")

    def publish_fixposition_speed(self, left_vel_m_s: float, right_vel_m_s: float):
        """Publish wheel speeds to Fixposition device.

        Args:
            left_vel_m_s: Left wheel velocity in m/s
            right_vel_m_s: Right wheel velocity in m/s
        """
        # Convert m/s to mm/s (Fixposition expects mm/s as integers)
        left_vel_mm_s = int(left_vel_m_s * 1000.0)
        right_vel_mm_s = int(right_vel_m_s * 1000.0)

        # Create Speed message
        speed_msg = Speed()

        # Create left wheel sensor
        left_sensor = WheelSensor()
        left_sensor.header = Header()
        left_sensor.header.stamp = self.get_clock().now().to_msg()
        left_sensor.header.frame_id = 'base_link'
        left_sensor.location = 'RL'  # Rear Left
        left_sensor.vx = left_vel_mm_s
        left_sensor.vy = 0
        left_sensor.vz = 0
        left_sensor.vx_valid = True
        left_sensor.vy_valid = False
        left_sensor.vz_valid = False

        # Create right wheel sensor
        right_sensor = WheelSensor()
        right_sensor.header = Header()
        right_sensor.header.stamp = self.get_clock().now().to_msg()
        right_sensor.header.frame_id = 'base_link'
        right_sensor.location = 'RR'  # Rear Right
        right_sensor.vx = right_vel_mm_s
        right_sensor.vy = 0
        right_sensor.vz = 0
        right_sensor.vx_valid = True
        right_sensor.vy_valid = False
        right_sensor.vz_valid = False

        # Add both sensors to Speed message
        speed_msg.sensors = [left_sensor, right_sensor]

        # Publish
        self._fixposition_speed_pub.publish(speed_msg)

        self.get_logger().debug(
            f"Published Fixposition speed -> L:{left_vel_mm_s} mm/s, R:{right_vel_mm_s} mm/s"
        )

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        if self._uart_device is not None:
            self._uart_device.close()
            self.get_logger().info("UART connection closed")
        super().destroy_node()


def main(args=None):
    """Main entry point for RobotControllerNode."""
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

