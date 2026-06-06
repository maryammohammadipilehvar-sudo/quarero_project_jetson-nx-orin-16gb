"""Status caching and management for robot state"""
import threading
import time
from typing import Dict, Any, Optional
import cv2


class StatusManager:
    """Manages cached robot status (position, frames, state, etc.)"""
    
    def __init__(self):
        self.current_position = {
            "latitude": 0.0, 
            "longitude": 0.0, 
            "height": 0.0,
            "yaw": 0.0
        }
        self.current_frame: Optional[Any] = None
        self.thermal1_frame: Optional[Any] = None
        self.thermal2_frame: Optional[Any] = None
        self.rgb2_frame: Optional[Any] = None
        self.lidar_debug_frame: Optional[Any] = None
        self.depth_debug_frame: Optional[Any] = None
        # Frame timestamps to track when frames were received (for newest-first sending)
        self.current_frame_time: float = 0.0
        self.thermal1_frame_time: float = 0.0
        self.thermal2_frame_time: float = 0.0
        self.rgb2_frame_time: float = 0.0
        self.lidar_debug_frame_time: float = 0.0
        self.depth_debug_frame_time: float = 0.0
        self.frame_lock = threading.Lock()
        
        self.robot_state = {
            "battery": 0.0,
            "velocity": "N/A",
            "error_status": ""
        }
        
        self.fusion_state = {
            "fusion_status": None,
            "imu_status": None,
            "gnss_status": None,
            "rtk_status": None
        }
        
        self.light_status = False
        self.alarm_status = False
        self.siren_status = False
        self.emergency_active = False
        self.autonomous_enabled = False
        self.charging_status = False
        
        # Connection status tracking
        self.last_robot_state_time: Optional[float] = None
        self.robot_connected = False
        self.connection_status_lock = threading.Lock()

        # ArUco dock marker (fused two-marker pose, camera optical frame) for dock setup.
        self.dock_marker_pose = None       # geometry_msgs/Pose or None
        self.dock_marker_time: float = 0.0  # wall time of last marker pose
    
    def get_position(self) -> Dict[str, Any]:
        return self.current_position
    
    def get_robot_state(self) -> Dict[str, Any]:
        return self.robot_state
    
    def get_fusion_state(self) -> Dict[str, Any]:
        return self.fusion_state
    
    def get_frame(self):
        """Get main camera frame. Returns (frame_copy, timestamp) or (None, 0.0)
        Makes a copy to avoid blocking - lock is held only briefly for read"""
        with self.frame_lock:
            frame = self.current_frame
            timestamp = self.current_frame_time
        # Copy frame outside lock to avoid blocking async handlers
        if frame is not None:
            return frame.copy(), timestamp
        return None, 0.0
    
    def get_thermal1_frame(self):
        """Get thermal1 frame. Returns (frame_copy, timestamp) or (None, 0.0)
        Makes a copy to avoid blocking - lock is held only briefly for read"""
        with self.frame_lock:
            frame = self.thermal1_frame
            timestamp = self.thermal1_frame_time
        # Copy frame outside lock to avoid blocking async handlers
        if frame is not None:
            return frame.copy(), timestamp
        return None, 0.0
    
    def get_thermal2_frame(self):
        """Get thermal2 frame. Returns (frame_copy, timestamp) or (None, 0.0)
        Makes a copy to avoid blocking - lock is held only briefly for read"""
        with self.frame_lock:
            frame = self.thermal2_frame
            timestamp = self.thermal2_frame_time
        # Copy frame outside lock to avoid blocking async handlers
        if frame is not None:
            return frame.copy(), timestamp
        return None, 0.0
    
    def get_rgb2_frame(self):
        """Get rgb2 frame (Axis channel 1 RGB). Returns (frame_copy, timestamp) or (None, 0.0)"""
        with self.frame_lock:
            frame = self.rgb2_frame
            timestamp = self.rgb2_frame_time
        if frame is not None:
            return frame.copy(), timestamp
        return None, 0.0

    def get_lidar_debug_frame(self):
        """Get lidar debug frame. Returns (frame_copy, timestamp) or (None, 0.0)
        Makes a copy to avoid blocking - lock is held only briefly for read"""
        with self.frame_lock:
            frame = self.lidar_debug_frame
            timestamp = self.lidar_debug_frame_time
        # Copy frame outside lock to avoid blocking async handlers
        if frame is not None:
            return frame.copy(), timestamp
        return None, 0.0

    def get_depth_debug_frame(self):
        """Get lidar debug frame. Returns (frame_copy, timestamp) or (None, 0.0)
        Makes a copy to avoid blocking - lock is held only briefly for read"""
        with self.frame_lock:
            frame = self.depth_debug_frame
            timestamp = self.depth_debug_frame_time
        # Copy frame outside lock to avoid blocking async handlers
        if frame is not None:
            return frame.copy(), timestamp
        return None, 0.0
    
    def clear_frame(self, frame_type: str = "main"):
        """Clear frame cache"""
        with self.frame_lock:
            if frame_type == "main":
                self.current_frame = None
                self.current_frame_time = 0.0
            elif frame_type == "thermal1":
                self.thermal1_frame = None
                self.thermal1_frame_time = 0.0
            elif frame_type == "thermal2":
                self.thermal2_frame = None
                self.thermal2_frame_time = 0.0
            elif frame_type == "rgb2":
                self.rgb2_frame = None
                self.rgb2_frame_time = 0.0
            elif frame_type == "lidar_debug":
                self.lidar_debug_frame = None
                self.lidar_debug_frame_time = 0.0
                self.depth_debug_frame = None
                self.depth_debug_frame_time = 0.0

