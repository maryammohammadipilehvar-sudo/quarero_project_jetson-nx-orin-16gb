"""ROS2 Subscriber callbacks"""
import math
import json
import asyncio
import time
from datetime import datetime
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from fixposition_driver_msgs.msg import FusionEpoch
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from typing import Optional

from .status_manager import StatusManager
from ..utils.connection_manager import ConnectionManager
from interfaces.msg import SecurityAlert
from ..services.security_service import handle_security_alert


class SubscriberCallbacks:
    """Handles all ROS2 subscriber callbacks"""
    
    def __init__(self, node, status_manager: StatusManager, connection_manager: ConnectionManager, event_loop: Optional[asyncio.AbstractEventLoop] = None):
        self.node = node
        self.status_manager = status_manager
        self.connection_manager = connection_manager
        self.event_loop = event_loop
        self.bridge = CvBridge()
    
    def position_callback(self, msg: NavSatFix) -> None:
        try:
            self.status_manager.current_position.update({
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "height": msg.altitude,
            })
        except Exception as e:
            self.node.get_logger().error(f'Position error: {e}')

    def fusion_callback(self, msg: FusionEpoch) -> None:
        try:
            if not getattr(msg, "fpa_odomstatus_avail", False):
                return
            
            status = msg.fpa_odomstatus
            
            self.status_manager.fusion_state["imu_status"] = int(getattr(status, "imu_status", -1))
            self.status_manager.fusion_state["gnss1_status"] = int(getattr(status, "gnss1_status", -1))  
            self.status_manager.fusion_state["gnss2_status"] = int(getattr(status, "gnss2_status", -1))
            self.status_manager.fusion_state["fusion_status"] = int(getattr(status, "init_status", -1))
            
            # Raw velocity from fusion is kept for reference but filtered speed is preferred
            if getattr(msg, "fpa_odomsh_avail", False):
                odomsh = msg.fpa_odomsh
                velocity = odomsh.velocity.twist.linear
                
                vx = velocity.x
                vy = velocity.y
                
                speed_ms = math.sqrt(vx**2 + vy**2)
                speed_kmh = speed_ms * 3.6
                
                self.status_manager.robot_state["velocity"] = round(speed_ms, 2)
                # velocity_kmh will be overwritten by filtered speed estimate if available
                if "velocity_kmh_filtered" not in self.status_manager.robot_state:
                    self.status_manager.robot_state["velocity_kmh"] = round(speed_kmh, 1)
            
        except Exception as e:
            self.node.get_logger().error(f'Fusion status error: {e}')
    
    def speed_estimate_callback(self, msg: Float32) -> None:
        """Handle filtered speed estimate from tactical_wp_follower (preferred over raw GPS)."""
        try:
            speed_kmh = msg.data
            # Update with filtered speed (this takes priority over raw GPS speed)
            self.status_manager.robot_state["velocity_kmh_filtered"] = round(speed_kmh, 1)
            self.status_manager.robot_state["velocity_kmh"] = round(speed_kmh, 1)
            self.status_manager.robot_state["velocity"] = round(speed_kmh / 3.6, 2)
        except Exception as e:
            self.node.get_logger().error(f'Speed estimate callback error: {e}')

    def odom_callback(self, msg: Odometry) -> None:
        try:
            q = msg.pose.pose.orientation

            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            yaw_deg = math.degrees(yaw_rad)

            self.status_manager.current_position["yaw"] = yaw_deg
        except Exception as e:
            self.node.get_logger().error(f'Odom yaw error: {e}')

    def camera_callback(self, msg: Image) -> None:
        try:
            # Always process frames to ensure StatusManager always has the latest frame
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Use message timestamp if valid, otherwise fallback to current time
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg_time <= 0:
                msg_time = time.time()
            
            # ALWAYS update frame - no timestamp filtering in callback
            with self.status_manager.frame_lock:
                self.status_manager.current_frame = cv_image
                self.status_manager.current_frame_time = msg_time
        except Exception as e:
            self.node.get_logger().error(f'Camera error: {e}')

    def thermal1_callback(self, msg: Image) -> None:
        try:
            # Always process frames to ensure StatusManager always has the latest frame
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Use message timestamp if valid, otherwise fallback to current time
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg_time <= 0:
                # Invalid timestamp - use current time
                msg_time = time.time()
            
            # ALWAYS update frame - no timestamp filtering in callback
            # The WebSocket handler will do newest-frame filtering
            with self.status_manager.frame_lock:
                self.status_manager.thermal1_frame = cv_image
                self.status_manager.thermal1_frame_time = msg_time
        except Exception as e:
            self.node.get_logger().error(f'Thermal1 error: {e}')

    def thermal2_callback(self, msg: Image) -> None:
        try:
            # Always process frames to ensure StatusManager always has the latest frame
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Use message timestamp if valid, otherwise fallback to current time
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg_time <= 0:
                msg_time = time.time()
            
            # ALWAYS update frame - no timestamp filtering in callback
            with self.status_manager.frame_lock:
                self.status_manager.thermal2_frame = cv_image
                self.status_manager.thermal2_frame_time = msg_time
        except Exception as e:
            self.node.get_logger().error(f'Thermal2 error: {e}')
    
    def lidar_debug_callback(self, msg: Image) -> None:
        try:
            # Only process if stream is active (optimization: skip conversion if no one is watching)
            if not self.connection_manager.is_camera_stream_active('lidar_debug'):
                return
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Use message timestamp if valid, otherwise fallback to current time
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg_time <= 0:
                msg_time = time.time()
            
            # ALWAYS update frame - no timestamp filtering in callback
            with self.status_manager.frame_lock:
                self.status_manager.lidar_debug_frame = cv_image
                self.status_manager.lidar_debug_frame_time = msg_time
        except Exception as e:
            self.node.get_logger().error(f'LIDAR debug error: {e}')


    def depth_debug_callback(self, msg: Image) -> None:
        try:
            # Only process if stream is active (optimization: skip conversion if no one is watching)
            if not self.connection_manager.is_camera_stream_active('depth_debug'):
                return
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Use message timestamp if valid, otherwise fallback to current time
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg_time <= 0:
                msg_time = time.time()
            
            # ALWAYS update frame - no timestamp filtering in callback
            with self.status_manager.frame_lock:
                self.status_manager.depth_debug_frame = cv_image
                self.status_manager.depth_debug_frame_time = msg_time
        except Exception as e:
            self.node.get_logger().error(f'LIDAR debug error: {e}')

    def robot_state_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            # Convert battery_percentage to battery (round float values properly)
            battery_value = data.get("battery_percentage")
            if battery_value is not None:
                battery_value = round(float(battery_value))
            else:
                battery_value = 0
            
            self.status_manager.robot_state = {
                "battery": battery_value,
                "velocity": data.get("velocity", "N/A"),
                "error_status": data.get("error_status", ""),
                "active_route": data.get("active_route", None),  # Include active_route if present
                "remaining_runtime_minutes": data.get("remaining_runtime_minutes", None)  # Remaining operational time in minutes
            }
            
            # Update button states from robot (single source of truth)
            # Only update if the robot provides these values (backward compatibility)
            if "light_state" in data:
                self.status_manager.light_status = bool(data.get("light_state", False))
            if "alarm_state" in data:
                self.status_manager.alarm_status = bool(data.get("alarm_state", False))
            if "siren_state" in data:
                self.status_manager.siren_status = bool(data.get("siren_state", False))
            if "emergency_stop_active" in data:
                self.status_manager.emergency_active = bool(data.get("emergency_stop_active", False))
            if "autonomous_enabled" in data:
                self.status_manager.autonomous_enabled = bool(data.get("autonomous_enabled", False))
            if "charging_state" in data:
                self.status_manager.charging_status = bool(data.get("charging_state", False))
            
            # Update connection status
            current_time = time.time()
            with self.status_manager.connection_status_lock:
                was_connected = self.status_manager.robot_connected
                self.status_manager.last_robot_state_time = current_time
                self.status_manager.robot_connected = True
                
                # If connection was just restored, broadcast connection restored message
                if not was_connected:
                    self._broadcast_connection_status(True)
        except Exception as e:
            self.node.get_logger().error(f'Robot state error: {e}')
    
    def _broadcast_connection_status(self, connected: bool) -> None:
        """Broadcast connection status change"""
        async def broadcast_log(event: dict) -> None:
            await self.connection_manager.broadcast({"type": "event", "data": event})
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        if connected:
            message = "✅ Verbindung zum Roboter hergestellt"
        else:
            message = "❌ Keine Verbindung zum Roboter"
        
        event = {
            "timestamp": timestamp,
            "message": message,
            "level": "warn" if not connected else "info",
            "connection_status": True if connected else False  # Special flag for frontend
        }
        
        try:
            if self.event_loop is not None and self.event_loop.is_running():
                asyncio.run_coroutine_threadsafe(broadcast_log(event), self.event_loop)
        except Exception as e:
            self.node.get_logger().error(f'Connection status broadcast error: {e}')

    def robot_ack_callback(self, msg: Bool) -> None:
        # This is handled by the RobotNode class
        pass
    
    def logging_callback(self, msg: String, level: str) -> None:
        # Validate inputs to prevent "undefined" in web logs
        if msg is None:
            self.node.get_logger().warn('Logging callback received None message')
            return
        
        # Ensure level has a default value
        if level is None:
            level = 'info'
        
        # Ensure message data is not None
        message_data = msg.data if hasattr(msg, 'data') and msg.data is not None else ''
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        event = {
            "timestamp": timestamp,
            "message": message_data,
            "level": level
        }

        async def broadcast_log(event: dict) -> None:
            await self.connection_manager.broadcast({"type": "event", "data": event})

        try:
            if self.event_loop is not None and self.event_loop.is_running():
                asyncio.run_coroutine_threadsafe(broadcast_log(event), self.event_loop)
            else:
                self.node.get_logger().warn(
                    f'[{level.upper()}] Eventloop not running, skipping WS broadcast'
                )
        except Exception as e:
            self.node.get_logger().error(f'Logging broadcast error: {e}')

        self.node.get_logger().info(f'[{level.upper()}] {message_data}')

    def command_ack_callback(self, msg: String) -> None:
        # This is handled by the RobotNode class
        pass
    
    def security_alert_callback(self, msg: SecurityAlert) -> None:
        """Handle incoming security alerts (e.g. person or fire detected by cameras)."""
        try:
            handle_security_alert(self.node, msg)
        except Exception as e:
            self.node.get_logger().error(f'Security alert handling error: {e}')

