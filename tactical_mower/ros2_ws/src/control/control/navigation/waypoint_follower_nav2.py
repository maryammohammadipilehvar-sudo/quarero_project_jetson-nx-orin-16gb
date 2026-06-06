"""Nav2-based waypoint follower using ros2_navigation service.

This module provides a Nav2-based implementation of the WaypointFollower interface,
using the ros2_navigation service for path planning instead of nav2_simple_commander.
"""

import math
import time
import rclpy
from rclpy.node import Node
from typing import List, Optional, Tuple, Callable, Any
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ros2_navigation.srv import ComputePathToPose, ReplanPath

from .waypoint_follower import WaypointMode
from .transform_manager import TransformManager


class WaypointFollowerNav2:
    """Nav2-based waypoint follower compatible with WaypointFollower interface.
    
    Uses ros2_navigation service for path planning.
    Follows sub-paths from the navigation service to reach waypoints from web interface.
    """
    
    def __init__(
        self,
        node: Node,
        transform_manager: TransformManager,
        waypoint_tolerance: float = 1.0,
        max_steering: float = 100.0,
        mode: int = WaypointMode.ONCE,
        replan_check_interval: float = 2.0
    ):
        """Initialize Nav2 waypoint follower.
        
        Args:
            node: ROS2 node instance (required for service client)
            transform_manager: TransformManager for GPS → Map conversions
            waypoint_tolerance: Distance threshold to consider waypoint reached (meters)
            max_steering: Maximum steering command value
            mode: Initial mode (ONCE, LOOP, or PING_PONG)
            replan_check_interval: Interval in seconds between path collision checks.
                If <= 0, path will be checked continuously (check immediately after previous check completes).
                Default: 2.0 seconds
        """
        self.waypoint_tolerance = waypoint_tolerance
        self.max_steering = max_steering
        self.mode = mode
        self._node = node
        self._transform_manager = transform_manager
        
        # Service client for path planning
        self._path_service_client = self._node.create_client(
            ComputePathToPose,
            'compute_path_to_pose'
        )
        
        # Wait for service to be available
        if not self._path_service_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn(
                "Path planning service 'compute_path_to_pose' not available. "
                "Will retry on first path planning request."
            )
        
        # Service client for replanning
        self._replan_service_client = self._node.create_client(
            ReplanPath,
            'replan_path'
        )
        
        # Wait for service to be available (non-blocking, will retry on first use)
        if not self._replan_service_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn(
                "Replanning service 'replan_path' not available. "
                "Will retry on first replanning request."
            )
        
        # Waypoint state (from web interface, in map frame)
        self._waypoints_map: List[PoseStamped] = []
        self._current_wp_idx = 0
        self._forward_direction = True
        self._round_started = False
        self._mission_active = False
        
        # Sub-path state (from nav-package)
        self._current_sub_path: Optional[Path] = None
        self._current_sub_wp_idx = 0
        self._sub_path_waypoint_tolerance = 0.5  # meters - tolerance for sub-waypoints in path
        
        # Path planning state
        self._planning_in_progress = False
        self._current_goal_waypoint_map: Optional[PoseStamped] = None
        self._path_planning_future: Optional[Any] = None  # rclpy.task.Future from call_async
        self._path_planning_start_time: Optional[float] = None
        self._path_planning_timeout_sec = 20.0  # Timeout for path planning requests
        
        # Replanning state
        self._replan_check_interval = replan_check_interval
        self._last_replan_check_time: Optional[float] = None
        self._replanning_in_progress = False
        self._replan_future: Optional[Any] = None
        self._replan_start_time: Optional[float] = None
        self._replan_timeout_sec = 10.0  # Timeout for replanning requests
        
        # Rotation / deadlock handling (for rotation in place)
        self._is_rotating_in_place = False
        self._last_heading_error: Optional[float] = None
        self._last_update_time: Optional[float] = None
        self._deadlock_check_yaw: Optional[float] = None
        self._deadlock_check_time: Optional[float] = None
        self._deadlock_steering_boost: float = 0.0
        # Spin-guard: abort mission if we rotate in place too long without progress
        self._rotation_in_place_start_time: Optional[float] = None
        self._max_rotation_in_place_seconds: float = 15.0
        # Tuning constants - reduced to prevent overshoot at higher speeds
        self._rotation_gain_p = 1.2
        self._rotation_gain_d = 0.15
        self._min_steering_command = 12.0
        self._deadlock_check_interval = 0.5
        self._deadlock_yaw_threshold = 0.3
        self._deadlock_steering_increment = 3.0
        self._deadlock_max_steering = min(35.0, max_steering)
    
    def set_waypoints(self, waypoints: List[PoseStamped]):
        """Set waypoints to follow.
        
        Args:
            waypoints: List of waypoint poses (expected in map frame)
        """
        # Validate frame IDs
        invalid_frames = [wp for wp in waypoints if wp.header.frame_id != "map"]
        if invalid_frames:
            invalid_frame_ids = set(wp.header.frame_id for wp in invalid_frames)
            self._node.get_logger().warn(
                f"Warning: {len(invalid_frames)} waypoint(s) not in 'map' frame. "
                f"Expected 'map', but got: {invalid_frame_ids}. "
                f"Waypoints may not work correctly with navigation system."
            )
        
        self._waypoints_map = waypoints
        self._current_wp_idx = 0
        self._forward_direction = True
        self._round_started = False
        # Reset sub-path when waypoints change
        self._current_sub_path = None
        self._current_sub_wp_idx = 0
        self._current_goal_waypoint_map = None
    
    def set_mode(self, mode: int):
        """Set waypoint following mode.
        
        Args:
            mode: WaypointMode.ONCE, WaypointMode.LOOP, or WaypointMode.PING_PONG
        """
        self.mode = mode
    
    def start(self, start_index: Optional[int] = None):
        """Start waypoint following mission.
        
        Args:
            start_index: Optional starting waypoint index (defaults to 0)
        """
        if len(self._waypoints_map) == 0:
            self._node.get_logger().warn("Cannot start Nav2 navigation: no waypoints set")
            return
        
        if start_index is not None:
            self._current_wp_idx = max(0, min(start_index, len(self._waypoints_map) - 1))
        else:
            self._current_wp_idx = 0
        
        self._forward_direction = True
        self._round_started = False
        self._mission_active = True
        
        # Reset sub-path state
        self._current_sub_path = None
        self._current_sub_wp_idx = 0
        self._current_goal_waypoint_map = None
        self._planning_in_progress = False
        self._path_planning_future = None
        self._path_planning_start_time = None
        
        self._node.get_logger().info(
            f"Nav2 navigation started: {len(self._waypoints_map)} waypoints, mode={self.mode}"
        )
    
    def stop(self):
        """Stop waypoint following."""
        self._mission_active = False
        self._current_sub_path = None
        self._current_sub_wp_idx = 0
        self._current_goal_waypoint_map = None
        self._planning_in_progress = False
        self._path_planning_future = None
        self._path_planning_start_time = None
        # Reset replanning state when stopping
        self._replanning_in_progress = False
        self._replan_future = None
        self._replan_start_time = None
        self._last_replan_check_time = None
        # Reset spin-guard tracking
        self._is_rotating_in_place = False
        self._rotation_in_place_start_time = None
    
    def is_active(self) -> bool:
        """Check if mission is active.
        
        Returns:
            True if mission is active, False otherwise
        """
        return self._mission_active
    
    def _start_path_planning(self, goal_pose_map: PoseStamped) -> bool:
        """Start asynchronous path planning request.
        
        Args:
            goal_pose_map: Target waypoint pose (in map frame)
            
        Returns:
            True if request was started successfully, False otherwise
        """
        if not self._path_service_client.service_is_ready():
            if not self._path_service_client.wait_for_service(timeout_sec=0.1):
                self._node.get_logger().warn(
                    "Path planning service 'compute_path_to_pose' not available"
                )
                return False
        
        request = ComputePathToPose.Request()
        request.goal_pose = goal_pose_map
        # Empty start_pose means service will read current robot pose from TF
        request.start_pose = PoseStamped()
        request.start_pose.header.frame_id = ""
        
        try:
            self._path_planning_future = self._path_service_client.call_async(request)
            self._path_planning_start_time = time.monotonic()
            self._planning_in_progress = True
            return True
        except Exception as e:
            self._node.get_logger().error(f"Path planning request failed: {e}")
            self._path_planning_future = None
            self._path_planning_start_time = None
            self._planning_in_progress = False
            return False
    
    def _check_path_planning_result(self) -> Optional[Path]:
        """Check if path planning request is complete and return result.
        
        Returns:
            Planned path if successful and complete, None if still in progress or failed
        """
        if self._path_planning_future is None:
            return None
        
        # Check for timeout
        if self._path_planning_start_time is not None:
            elapsed = time.monotonic() - self._path_planning_start_time
            if elapsed > self._path_planning_timeout_sec:
                self._node.get_logger().warn(
                    f"Path planning request timed out after {elapsed:.2f}s"
                )
                self._path_planning_future = None
                self._path_planning_start_time = None
                self._planning_in_progress = False
                return None
        
        # Check if future is done
        if not self._path_planning_future.done():
            return None  # Still in progress
        
        # Future is done, get result
        try:
            response = self._path_planning_future.result()
            self._path_planning_future = None
            self._path_planning_start_time = None
            self._planning_in_progress = False
            
            if response.path_found and len(response.path.poses) > 0:
                # Validate that path is in map frame (performance: check only once here)
                path_frame = response.path.header.frame_id
                if path_frame != "map":
                    self._node.get_logger().warn(
                        f"Path planning returned path in frame '{path_frame}' instead of 'map'. "
                        f"This may cause navigation errors. Path will be used but may not work correctly."
                    )
                
                self._node.get_logger().info(
                    f"Path planning successful: {len(response.path.poses)} sub-waypoints (frame: {path_frame})"
                )
                return response.path
            else:
                self._node.get_logger().warn(
                    f"Path planning failed: {response.message}"
                )
                return None
        except Exception as e:
            self._node.get_logger().error(f"Path planning result error: {e}")
            self._path_planning_future = None
            self._path_planning_start_time = None
            self._planning_in_progress = False
            return None
    
    def _check_and_replan_if_needed(self, robot_pose_map: PoseStamped) -> bool:
        """Check current path for collisions and replan if needed.
        
        This method is rate-limited based on replan_check_interval.
        If replan_check_interval <= 0, it will check continuously (immediately
        after previous check completes).
        
        Args:
            robot_pose_map: Current robot pose (in map frame)
            
        Returns:
            True if replanning was triggered, False otherwise
        """
        # Only check if we have an active sub-path
        if self._current_sub_path is None or len(self._current_sub_path.poses) == 0:
            return False
        
        # Only check if we have a goal waypoint
        if self._current_goal_waypoint_map is None:
            return False
        
        # Don't check if replanning is already in progress (prevent parallel checks)
        if self._replanning_in_progress:
            return False
        
        # Don't check if path planning is in progress
        if self._planning_in_progress:
            return False
        
        # Rate limiting: only check every replan_check_interval seconds
        # If interval <= 0, check continuously (immediately after previous check)
        current_time = time.monotonic()
        if self._replan_check_interval > 0.0:
            if self._last_replan_check_time is not None:
                elapsed = current_time - self._last_replan_check_time
                if elapsed < self._replan_check_interval:
                    return False
        
        # Check if service is available
        if not self._replan_service_client.service_is_ready():
            if not self._replan_service_client.wait_for_service(timeout_sec=0.1):
                return False
        
        # Update last check time (for rate limiting)
        # If continuous mode (interval <= 0), we'll update after response
        if self._replan_check_interval > 0.0:
            self._last_replan_check_time = current_time
        
        # Create request
        request = ReplanPath.Request()
        request.path = self._current_sub_path
        request.goal_pose = self._current_goal_waypoint_map
        # Start checking from previous waypoint to include the segment the robot is currently on
        # This ensures we check the segment from (current-1) to current, not just from current onwards
        request.start_waypoint_index = max(0, self._current_sub_wp_idx - 1)
        
        try:
            # Call service asynchronously
            self._replan_future = self._replan_service_client.call_async(request)
            self._replan_start_time = current_time
            self._replanning_in_progress = True
            self._node.get_logger().debug(
                f"Checking path for collisions (current_wp_idx={self._current_sub_wp_idx}, "
                f"start_check_idx={request.start_waypoint_index}, "
                f"path_length={len(self._current_sub_path.poses)})"
            )
            return True
        except Exception as e:
            self._node.get_logger().error(f"Replanning request failed: {e}")
            self._replan_future = None
            self._replan_start_time = None
            self._replanning_in_progress = False
            return False
    
    def _check_replan_result(self) -> Optional[Path]:
        """Check if replanning request is complete and return result.
        
        Returns:
            New path if replanning was successful and collision was detected,
            None if still in progress, failed, or no collision detected
        """
        if self._replan_future is None:
            return None
        
        # Check for timeout
        if self._replan_start_time is not None:
            elapsed = time.monotonic() - self._replan_start_time
            if elapsed > self._replan_timeout_sec:
                self._node.get_logger().warn(
                    f"Replanning request timed out after {elapsed:.2f}s"
                )
                self._replan_future = None
                self._replan_start_time = None
                self._replanning_in_progress = False
                # Update last check time for continuous mode
                if self._replan_check_interval <= 0.0:
                    self._last_replan_check_time = time.monotonic()
                return None
        
        # Check if future is done
        if not self._replan_future.done():
            return None  # Still in progress
        
        # Future is done, get result
        try:
            response = self._replan_future.result()
            self._replan_future = None
            self._replan_start_time = None
            self._replanning_in_progress = False
            
            # Update last check time for continuous mode (immediately after response)
            if self._replan_check_interval <= 0.0:
                self._last_replan_check_time = time.monotonic()
            
            if response.has_collision:
                if response.path_found and len(response.path.poses) > 0:
                    self._node.get_logger().info(
                        f"Path collision detected and replanned: "
                        f"{len(response.path.poses)} waypoints in new path "
                        f"(was {len(self._current_sub_path.poses)} waypoints)"
                    )
                    return response.path
                else:
                    self._node.get_logger().warn(
                        f"Path collision detected but replanning failed: {response.message}"
                    )
                    return None
            else:
                # No collision - path is still valid
                self._node.get_logger().debug("Path collision check: no collision detected")
                return None
        except Exception as e:
            self._node.get_logger().error(f"Replanning result error: {e}")
            self._replan_future = None
            self._replan_start_time = None
            self._replanning_in_progress = False
            # Update last check time for continuous mode
            if self._replan_check_interval <= 0.0:
                self._last_replan_check_time = time.monotonic()
            return None
    
    def _extract_heading(self, pose: PoseStamped) -> float:
        """Extract heading (radians) from pose quaternion."""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def compute_steering(
        self,
        robot_pose_map: PoseStamped,
        goal_pose_map: PoseStamped
    ) -> float:
        """Compute steering command to reach goal from current pose.
        
        Args:
            robot_pose_map: Current robot pose (in map frame)
            goal_pose_map: Target pose (in map frame)
            
        Returns:
            Steering command in range [-max_steering, max_steering]
        """
        # Extract robot heading from quaternion
        heading = self._extract_heading(robot_pose_map)
        
        # Calculate angle to goal
        dx = goal_pose_map.pose.position.x - robot_pose_map.pose.position.x
        dy = goal_pose_map.pose.position.y - robot_pose_map.pose.position.y
        angle_to_goal = math.atan2(dy, dx)
        
        # Heading error (normalized to [-pi, pi])
        heading_error = self._normalize_angle(angle_to_goal - heading)
        
        # Convert to steering command
        steering = max(
            -self.max_steering,
            min(self.max_steering, heading_error * 100 / math.pi)
        )
        
        return steering
    
    def _compute_rotation_steering(self, error_deg: float, current_yaw_deg: float) -> float:
        """Compute steering for pure rotation with deadlock compensation.
        
        Args:
            error_deg: Heading error in degrees
            current_yaw_deg: Current yaw in degrees
            
        Returns:
            Steering command
        """
        # If error is within a very small tolerance, clear boost and return 0
        if abs(error_deg) < 1.0:
            self._deadlock_steering_boost = 0.0
            return 0.0
        
        now = time.time()
        
        # Deadlock detection: check if yaw is changing over interval
        if self._deadlock_check_time is None:
            self._deadlock_check_time = now
            self._deadlock_check_yaw = current_yaw_deg
        elif (now - self._deadlock_check_time) >= self._deadlock_check_interval:
            if self._deadlock_check_yaw is not None:
                yaw_change = abs(current_yaw_deg - self._deadlock_check_yaw)
                if yaw_change < self._deadlock_yaw_threshold:
                    # increase boost but clamp
                    self._deadlock_steering_boost = min(
                        self._deadlock_steering_boost + self._deadlock_steering_increment,
                        self._deadlock_max_steering - self._min_steering_command
                    )
                else:
                    # robot is rotating normally -> reduce boost gradually
                    self._deadlock_steering_boost = max(
                        0.0, 
                        self._deadlock_steering_boost - self._deadlock_steering_increment
                    )
            
            # reset check
            self._deadlock_check_time = now
            self._deadlock_check_yaw = current_yaw_deg
        
        # Derivative term (simple)
        d_term = 0.0
        if self._last_heading_error is not None and self._last_update_time is not None:
            dt = now - self._last_update_time
            if dt > 0.001:
                error_rate = (error_deg - self._last_heading_error) / dt
                d_term = error_rate * self._rotation_gain_d
        
        p_term = error_deg * self._rotation_gain_p
        steering = p_term + d_term
        
        # Apply minimum steering for significant rotations (>10°)
        if abs(error_deg) > 10.0:
            if abs(steering) < self._min_steering_command:
                steering = math.copysign(self._min_steering_command, steering)
        
        # Add deadlock boost
        steering = steering + math.copysign(self._deadlock_steering_boost, steering)
        
        # Clamp
        steering = max(-self.max_steering, min(self.max_steering, steering))
        
        # update tracking
        self._last_heading_error = error_deg
        self._last_update_time = now
        
        return steering
    
    @staticmethod
    def _compute_distance(pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Compute Euclidean distance between two poses.
        
        Args:
            pose1: First pose
            pose2: Second pose
            
        Returns:
            Distance in meters
        """
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx * dx + dy * dy)
    
    def _advance_to_next_waypoint(self, on_route_completed: Optional[Callable[[], None]]):
        """Advance to next waypoint based on mode."""
        # Handle mission completion based on mode
        if self._current_wp_idx >= len(self._waypoints_map) - 1:
            if self.mode == WaypointMode.ONCE:
                self._mission_active = False
                if on_route_completed:
                    on_route_completed()
                return
            elif self.mode == WaypointMode.LOOP:
                # Reset to first waypoint to continue looping
                # Mission completion is handled when reaching waypoint 0 after a full round
                self._current_wp_idx = 0
                self._forward_direction = True
            elif self.mode == WaypointMode.PING_PONG:
                # Switch direction and go to second-to-last waypoint
                # Mission completion is handled when reaching waypoint 0 backward after a round started
                self._forward_direction = False
                self._current_wp_idx = len(self._waypoints_map) - 2
        else:
            # Advance to next waypoint
            if self._forward_direction:
                self._current_wp_idx += 1
            else:
                self._current_wp_idx -= 1
            
            # Mark round started when reaching any waypoint other than 0
            if self._current_wp_idx != 0:
                self._round_started = True
        
        # Reset sub-path state for next waypoint
        self._current_sub_path = None
        self._current_sub_wp_idx = 0
        self._current_goal_waypoint_map = None
        self._planning_in_progress = False
        # Reset replanning state when advancing to next waypoint
        self._replanning_in_progress = False
        self._replan_future = None
        self._replan_start_time = None
        self._last_replan_check_time = None
    
    def update(
        self,
        robot_pose_map: PoseStamped,
        on_waypoint_reached: Optional[Callable[[int], None]] = None,
        on_route_completed: Optional[Callable[[], None]] = None
    ) -> Tuple[Optional[float], Optional[bool], Optional[float]]:
        """Update waypoint follower and get current command.
        
        Args:
            robot_pose_map: Current robot pose (in map frame)
            on_waypoint_reached: Optional callback when waypoint is reached (idx)
            on_route_completed: Optional callback when route is completed
            
        Returns:
            Tuple of (steering, continue, speed) where:
            - steering: Steering command (None if route completed)
            - continue: True to continue, False if route completed
            - speed: Speed percentage (0-100) to apply when continuing; None if not continuing
        """
        if not self._mission_active or len(self._waypoints_map) == 0:
            return None, False, None
        
        # Check for replanning result first (before path planning check)
        if self._replanning_in_progress:
            new_path = self._check_replan_result()
            if new_path is not None:
                # Replanning successful - use new path
                self._current_sub_path = new_path
                self._current_sub_wp_idx = 0  # Start from beginning of new path
                self._node.get_logger().info(
                    f"Using replanned path: {len(new_path.poses)} waypoints"
                )
            elif self._replanning_in_progress:
                # Still in progress - stop and wait
                self._node.get_logger().debug("Waiting for replanning result...")
                return 0.0, True, 0.0
        
        # Check if path planning is in progress and if result is available
        if self._planning_in_progress:
            sub_path = self._check_path_planning_result()
            if sub_path is not None:
                # Path planning completed successfully
                if len(sub_path.poses) > 0:
                    # Validate path frame once when setting it (performance: check only here)
                    if sub_path.header.frame_id != "map":
                        self._node.get_logger().warn(
                            f"Sub-path frame mismatch: expected 'map', got '{sub_path.header.frame_id}'. "
                            f"Navigation may fail. Checking first waypoint frame..."
                        )
                        # Check first waypoint frame as additional validation
                        if len(sub_path.poses) > 0 and sub_path.poses[0].header.frame_id != "map":
                            self._node.get_logger().error(
                                f"First sub-waypoint also in wrong frame '{sub_path.poses[0].header.frame_id}'. "
                                f"Path will be rejected to prevent navigation errors."
                            )
                            self._current_sub_path = None
                            return 0.0, True, 0.0  # Stop and wait for next path planning attempt
                    
                    self._current_sub_path = sub_path
                    self._current_sub_wp_idx = 0
                    self._node.get_logger().info(
                        f"Following sub-path to waypoint {self._current_wp_idx}: "
                        f"{len(sub_path.poses)} sub-waypoints (frame: {sub_path.header.frame_id})"
                    )
                else:
                    # Path planning returned empty path - use direct navigation
                    self._node.get_logger().warn(
                        f"Path planning returned empty path for waypoint {self._current_wp_idx}, "
                        "using direct navigation"
                    )
                    self._current_sub_path = None
            else:
                # Path planning is still in progress - stop and wait
                self._node.get_logger().debug(
                    f"Waiting for path planning result for waypoint {self._current_wp_idx}"
                )
                return 0.0, True, 0.0  # Stop robot (steering=0, speed=0) while waiting
        
        # Check if we need to start planning a new sub-path
        if self._current_sub_path is None and not self._planning_in_progress:
            if self._current_wp_idx >= len(self._waypoints_map):
                # Handle mission completion based on mode
                if self.mode == WaypointMode.ONCE:
                    self._mission_active = False
                    if on_route_completed:
                        on_route_completed()
                    return None, False, None
                elif self.mode == WaypointMode.LOOP:
                    self._current_wp_idx = 0
                    self._forward_direction = True
                elif self.mode == WaypointMode.PING_PONG:
                    self._forward_direction = False
                    self._current_wp_idx = len(self._waypoints_map) - 2
            
            # Handle PING_PONG backward movement
            if self.mode == WaypointMode.PING_PONG and not self._forward_direction:
                if self._current_wp_idx < 0:
                    self._forward_direction = True
                    self._current_wp_idx = 1
            
            if self._current_wp_idx < len(self._waypoints_map):
                goal_waypoint_map = self._waypoints_map[self._current_wp_idx]
                self._current_goal_waypoint_map = goal_waypoint_map
                
                # Start asynchronous path planning
                if self._start_path_planning(goal_waypoint_map):
                    self._node.get_logger().debug(
                        f"Started path planning for waypoint {self._current_wp_idx}"
                    )
                else:
                    # Path planning service not available - use direct navigation
                    self._node.get_logger().warn(
                        f"Path planning service not available for waypoint {self._current_wp_idx}, "
                        "using direct navigation"
                    )
                    self._current_sub_path = None
        
        # If we have a sub-path, follow it
        if self._current_sub_path is not None and len(self._current_sub_path.poses) > 0:
            # Check for collisions and replan if needed (non-blocking, rate-limited)
            # This is called every update cycle, but internally rate-limited
            self._check_and_replan_if_needed(robot_pose_map)
            
            # Check if we've completed the sub-path
            if self._current_sub_wp_idx >= len(self._current_sub_path.poses):
                # Sub-path completed - check if we reached the goal waypoint
                if self._current_goal_waypoint_map is not None:
                    dist_to_goal = self._compute_distance(robot_pose_map, self._current_goal_waypoint_map)
                    if dist_to_goal < self.waypoint_tolerance:
                        # Waypoint reached
                        if on_waypoint_reached:
                            on_waypoint_reached(self._current_wp_idx)
                        
                        # Check if we reached home position (waypoint 0) in LOOP or PING_PONG mode
                        if self._current_wp_idx == 0:
                            # For LOOP mode: complete when reaching waypoint 0 forward after a round started
                            if self.mode == WaypointMode.LOOP and self._forward_direction and self._round_started:
                                self._mission_active = False
                                if on_route_completed:
                                    on_route_completed()
                                return None, False, None
                            # For PING_PONG mode: complete when reaching waypoint 0 backward after a round started
                            elif self.mode == WaypointMode.PING_PONG and not self._forward_direction and self._round_started:
                                self._mission_active = False
                                if on_route_completed:
                                    on_route_completed()
                                return None, False, None
                            # Mark that a round has started (first time reaching waypoint 0 after starting)
                            if not self._round_started:
                                self._round_started = True
                        else:
                            # Mark round started when reaching any waypoint other than 0
                            self._round_started = True
                        
                        # Advance to next waypoint
                        self._advance_to_next_waypoint(on_route_completed)
                        
                        # Reset rotation in place flag
                        self._is_rotating_in_place = False
                        self._rotation_in_place_start_time = None
                        # Return zero steering to allow state update; provide default speed
                        return 0.0, True, 100.0
                    else:
                        # Sub-path completed but not at goal - continue to goal directly
                        self._current_sub_path = None
                        self._current_sub_wp_idx = 0
                        # Break out of sub-path following and fall through to direct waypoint following
                else:
                    # No goal waypoint set - advance to next waypoint
                    self._advance_to_next_waypoint(on_route_completed)
                    return 0.0, True, 100.0
            
            # Follow current sub-waypoint in sub-path (only if sub-path is still valid)
            if self._current_sub_path is not None and self._current_sub_wp_idx < len(self._current_sub_path.poses):
                current_sub_waypoint_map = self._current_sub_path.poses[self._current_sub_wp_idx]
                dist_to_sub_wp = self._compute_distance(robot_pose_map, current_sub_waypoint_map)
                
                # Check if sub-waypoint reached
                if dist_to_sub_wp < self._sub_path_waypoint_tolerance:
                    # Advance to next sub-waypoint
                    self._current_sub_wp_idx += 1
                    # Reset rotation in place flag when sub-waypoint reached
                    self._is_rotating_in_place = False
                    self._rotation_in_place_start_time = None
                    # Continue to next sub-waypoint
                    if self._current_sub_wp_idx < len(self._current_sub_path.poses):
                        # Get next sub-waypoint
                        next_sub_waypoint_map = self._current_sub_path.poses[self._current_sub_wp_idx]
                        # Compute steering to next sub-waypoint
                        steering = self.compute_steering(robot_pose_map, next_sub_waypoint_map)
                        return steering, True, 100.0
                    else:
                        # Last sub-waypoint reached - will be handled in next update
                        return 0.0, True, 100.0
                
                # Compute heading to sub-waypoint and current heading
                current_heading = self._extract_heading(robot_pose_map)
                angle_to_sub_wp = math.atan2(
                    current_sub_waypoint_map.pose.position.y - robot_pose_map.pose.position.y,
                    current_sub_waypoint_map.pose.position.x - robot_pose_map.pose.position.x
                )
                heading_error = self._normalize_angle(angle_to_sub_wp - current_heading)
                heading_error_deg = math.degrees(heading_error)
                
                # If heading error large (>22.5°) rotate on the spot first
                if abs(heading_error_deg) > 22.5:
                    now_ts = time.time()
                    if not self._is_rotating_in_place:
                        self._rotation_in_place_start_time = now_ts
                    elif (self._rotation_in_place_start_time is not None and
                          (now_ts - self._rotation_in_place_start_time) > self._max_rotation_in_place_seconds):
                        # Spin-guard: rotated too long without progress. Fail SOFT —
                        # skip this waypoint (mirrors the linear follower's rotation
                        # timeout) instead of aborting the route. Calling
                        # on_route_completed() here would falsely report SUCCESS to the
                        # scheduler and silently drop the remaining waypoints.
                        try:
                            self._node.get_logger().warn(
                                f"Spin-guard: rotated in place > {self._max_rotation_in_place_seconds:.0f}s "
                                f"without sub-waypoint progress (heading_error={heading_error_deg:.1f}°). Skipping waypoint."
                            )
                        except Exception:
                            pass
                        self._is_rotating_in_place = False
                        self._rotation_in_place_start_time = None
                        # _advance_to_next_waypoint only signals real completion when
                        # this was the final ONCE waypoint; LOOP/PING_PONG continue.
                        self._advance_to_next_waypoint(on_route_completed)
                        return 0.0, True, 100.0
                    current_yaw_deg = math.degrees(current_heading)
                    steering = self._compute_rotation_steering(heading_error_deg, current_yaw_deg)
                    self._is_rotating_in_place = True
                    return steering, True, 0.0

                # Otherwise compute normal steering and drive at full speed
                self._is_rotating_in_place = False
                self._rotation_in_place_start_time = None
                steering = self.compute_steering(robot_pose_map, current_sub_waypoint_map)
                return steering, True, 100.0
        
        # Fallback: direct navigation to waypoint (if no sub-path available)
        if self._current_goal_waypoint_map is not None:
            dist = self._compute_distance(robot_pose_map, self._current_goal_waypoint_map)
            
            # Check if waypoint reached
            if dist < self.waypoint_tolerance:
                if on_waypoint_reached:
                    on_waypoint_reached(self._current_wp_idx)
                
                # Check if we reached home position (waypoint 0) in LOOP or PING_PONG mode
                if self._current_wp_idx == 0:
                    # For LOOP mode: complete when reaching waypoint 0 forward after a round started
                    if self.mode == WaypointMode.LOOP and self._forward_direction and self._round_started:
                        self._mission_active = False
                        if on_route_completed:
                            on_route_completed()
                        return None, False, None
                    # For PING_PONG mode: complete when reaching waypoint 0 backward after a round started
                    elif self.mode == WaypointMode.PING_PONG and not self._forward_direction and self._round_started:
                        self._mission_active = False
                        if on_route_completed:
                            on_route_completed()
                        return None, False, None
                    # Mark that a round has started (first time reaching waypoint 0 after starting)
                    if not self._round_started:
                        self._round_started = True
                else:
                    # Mark round started when reaching any waypoint other than 0
                    self._round_started = True
                
                # Advance to next waypoint
                self._advance_to_next_waypoint(on_route_completed)

                # Reset rotation in place flag
                self._is_rotating_in_place = False
                self._rotation_in_place_start_time = None
                return 0.0, True, 100.0

            # Compute heading to waypoint and current heading
            current_heading = self._extract_heading(robot_pose_map)
            angle_to_goal = math.atan2(
                self._current_goal_waypoint_map.pose.position.y - robot_pose_map.pose.position.y,
                self._current_goal_waypoint_map.pose.position.x - robot_pose_map.pose.position.x
            )
            heading_error = self._normalize_angle(angle_to_goal - current_heading)
            heading_error_deg = math.degrees(heading_error)

            # If heading error large (>22.5°) rotate on the spot first
            if abs(heading_error_deg) > 22.5:
                now_ts = time.time()
                if not self._is_rotating_in_place:
                    self._rotation_in_place_start_time = now_ts
                elif (self._rotation_in_place_start_time is not None and
                      (now_ts - self._rotation_in_place_start_time) > self._max_rotation_in_place_seconds):
                    # Spin-guard: rotated too long without progress. Fail SOFT — skip
                    # this waypoint (mirrors the linear follower's rotation timeout)
                    # instead of aborting the route. Calling on_route_completed() here
                    # would falsely report SUCCESS to the scheduler and silently drop
                    # the remaining waypoints.
                    try:
                        self._node.get_logger().warn(
                            f"Spin-guard: rotated in place > {self._max_rotation_in_place_seconds:.0f}s "
                            f"toward goal waypoint (heading_error={heading_error_deg:.1f}°). Skipping waypoint."
                        )
                    except Exception:
                        pass
                    self._is_rotating_in_place = False
                    self._rotation_in_place_start_time = None
                    # _advance_to_next_waypoint only signals real completion when this
                    # was the final ONCE waypoint; LOOP/PING_PONG continue.
                    self._advance_to_next_waypoint(on_route_completed)
                    return 0.0, True, 100.0
                current_yaw_deg = math.degrees(current_heading)
                steering = self._compute_rotation_steering(heading_error_deg, current_yaw_deg)
                self._is_rotating_in_place = True
                return steering, True, 0.0

            # Otherwise compute normal steering and drive at full speed
            self._is_rotating_in_place = False
            self._rotation_in_place_start_time = None
            steering = self.compute_steering(robot_pose_map, self._current_goal_waypoint_map)
            return steering, True, 100.0
        
        # No active navigation
        return None, False, None
    
    def get_current_waypoint_index(self) -> int:
        """Get current waypoint index.
        
        Returns:
            Current waypoint index
        """
        return self._current_wp_idx
    
    def get_waypoint_count(self) -> int:
        """Get total number of waypoints.
        
        Returns:
            Number of waypoints
        """
        return len(self._waypoints_map)
    
    def find_nearest_waypoint(self, robot_pose_map: PoseStamped) -> int:
        """Find the index of the nearest waypoint.
        
        Args:
            robot_pose_map: Current robot pose (in map frame)
            
        Returns:
            Index of nearest waypoint
        """
        if len(self._waypoints_map) == 0:
            return 0
        
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, wp in enumerate(self._waypoints_map):
            dist = self._compute_distance(robot_pose_map, wp)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def is_rotating_in_place(self) -> bool:
        """Check if the robot is currently rotating in place.
        
        Returns:
            True if rotating in place, False otherwise
        """
        return self._is_rotating_in_place
    
    def get_next_waypoint(self, robot_pose_map: PoseStamped) -> Optional[PoseStamped]:
        """Get the next active waypoint for navigation.
        
        Returns the next waypoint that the robot is currently navigating to.
        This can be either:
        - The current sub-waypoint from the sub-path (if following a planned path)
        - The current goal waypoint (if no sub-path is available)
        
        Args:
            robot_pose_map: Current robot pose (in map frame) - currently unused but
                kept for potential future use (e.g., to determine closest waypoint)
        
        Returns:
            Next waypoint pose (in map frame) if available, None otherwise
        """
        # If we have an active sub-path, return the current sub-waypoint
        if self._current_sub_path is not None and len(self._current_sub_path.poses) > 0:
            if self._current_sub_wp_idx < len(self._current_sub_path.poses):
                return self._current_sub_path.poses[self._current_sub_wp_idx]
        
        # Fallback: return the current goal waypoint (if no sub-path available)
        if self._current_goal_waypoint_map is not None:
            return self._current_goal_waypoint_map
        
        # No active waypoint
        return None
