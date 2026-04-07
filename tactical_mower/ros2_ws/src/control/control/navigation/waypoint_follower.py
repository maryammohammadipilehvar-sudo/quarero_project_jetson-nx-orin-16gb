
"""Core waypoint following algorithm (pure logic, no ROS dependencies)."""

import math
import time
from typing import List, Optional, Tuple, Callable
from geometry_msgs.msg import PoseStamped, Quaternion


# Waypoint follower modes (matching GeoPath constants)
class WaypointMode:
    ONCE = 0
    LOOP = 1
    PING_PONG = 2


class WaypointFollower:
    """Pure waypoint following algorithm without ROS dependencies.
    
    Handles waypoint navigation logic, mode management (ONCE, LOOP, PING_PONG),
    and steering/speed command generation.
    """
    
    def __init__(
        self,
        waypoint_tolerance: float = 1.0,
        max_steering: float = 100.0,
        mode: int = WaypointMode.ONCE
    ):
        """Initialize waypoint follower.
        
        Args:
            waypoint_tolerance: Distance threshold to consider waypoint reached (meters)
            max_steering: Maximum steering command value
            mode: Initial mode (ONCE, LOOP, or PING_PONG)
        """
        self.waypoint_tolerance = waypoint_tolerance
        self.max_steering = max_steering
        self.mode = mode
        
        # State
        self._waypoints: List[PoseStamped] = []
        self._current_wp_idx = 0
        self._forward_direction = True
        self._round_started = False
        self._mission_active = False
        self._is_rotating_in_place = False  # Track if currently rotating in place
        self._rotation_start_time: Optional[float] = None  # When rotation started
        self._rotation_timeout: float = 15.0  # Max seconds to rotate before skipping waypoint
        # Rotation / deadlock handling (to overcome rolling resistance when rotating in place)
        self._last_heading_error: Optional[float] = None
        self._last_update_time: Optional[float] = None
        self._deadlock_check_yaw: Optional[float] = None
        self._deadlock_check_time: Optional[float] = None
        self._deadlock_steering_boost: float = 0.0
        # tuning constants - reduced to prevent overshoot at higher speeds
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
            waypoints: List of waypoint poses
        """
        self._waypoints = waypoints
        self._current_wp_idx = 0
        self._forward_direction = True
        self._round_started = False
    
    def set_mode(self, mode: int):
        """Set waypoint following mode.
        
        Args:
            mode: WaypointMode.ONCE, WaypointMode.LOOP, or WaypointMode.PING_PONG
        """
        self.mode = mode
    
    def start(self, start_index: Optional[int] = None):
        """Start waypoint following mission.
        
        Args:
            start_index: Optional starting waypoint index (defaults to nearest)
        """
        if start_index is not None:
            self._current_wp_idx = max(0, min(start_index, len(self._waypoints) - 1))
        else:
            self._current_wp_idx = 0
        self._forward_direction = True
        self._round_started = False
        self._mission_active = True
    
    def stop(self):
        """Stop waypoint following."""
        self._mission_active = False
    
    def is_active(self) -> bool:
        """Check if mission is active.
        
        Returns:
            True if mission is active, False otherwise
        """
        return self._mission_active
    
    def compute_steering(
        self,
        robot_pose: PoseStamped,
        goal_pose: PoseStamped
    ) -> float:
        """Compute steering command to reach goal from current pose.
        
        Args:
            robot_pose: Current robot pose
            goal_pose: Target waypoint pose
            
        Returns:
            Steering command in range [-max_steering, max_steering]
        """
        # Extract robot heading from quaternion
        q = robot_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        heading = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate angle to goal
        dx = goal_pose.pose.position.x - robot_pose.pose.position.x
        dy = goal_pose.pose.position.y - robot_pose.pose.position.y
        angle_to_goal = math.atan2(dy, dx)
        
        # Heading error (normalized to [-pi, pi])
        heading_error = math.atan2(
            math.sin(angle_to_goal - heading),
            math.cos(angle_to_goal - heading)
        )
        
        # Convert to steering command
        steering = max(
            -self.max_steering,
            min(self.max_steering, heading_error * 100 / math.pi)
        )
        
        return steering

    def _extract_heading(self, pose: PoseStamped) -> float:
        """Extract heading (radians) from pose quaternion."""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _compute_rotation_steering(self, error_deg: float, current_yaw_deg: float) -> float:
        """Compute steering for pure rotation with deadlock compensation.

        This mirrors the deadlock/boost behaviour used in the docking controller:
        - Apply PD control for rotation
        - If error is large, ensure a minimum steering command to overcome deadband
        - If yaw change is negligible over an interval, increase a deadlock boost
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
                    self._deadlock_steering_boost = max(0.0, self._deadlock_steering_boost - self._deadlock_steering_increment)

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
    def compute_distance(pose1: PoseStamped, pose2: PoseStamped) -> float:
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
    
    def update(
        self,
        robot_pose: PoseStamped,
        on_waypoint_reached: Optional[Callable[[int], None]] = None,
        on_route_completed: Optional[Callable[[], None]] = None
    ) -> Tuple[Optional[float], Optional[bool], Optional[float]]:
        """Update waypoint follower and get current command.

        Args:
            robot_pose: Current robot pose
            on_waypoint_reached: Optional callback when waypoint is reached (idx)
            on_route_completed: Optional callback when route is completed

        Returns:
            Tuple of (steering, continue, speed) where:
            - steering: Steering command (None if route completed)
            - continue: True to continue, False if route completed
            - speed: Speed percentage (0-100) to apply when continuing; None if not continuing

        For now the speed value is fixed to 100 when continuing navigation.
        """
        if not self._mission_active or len(self._waypoints) == 0:
            return None, False, None
        
        # Handle mission completion based on mode
        if self._current_wp_idx >= len(self._waypoints):
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
                self._current_wp_idx = len(self._waypoints) - 2
        
        # Handle PING_PONG backward movement
        if self.mode == WaypointMode.PING_PONG and not self._forward_direction:
            if self._current_wp_idx < 0:
                self._forward_direction = True
                self._current_wp_idx = 1
        
        # Get current waypoint
        wp = self._waypoints[self._current_wp_idx]
        dist = self.compute_distance(robot_pose, wp)
        
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
                # (this means we've done forward->backward and returned to start)
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
            if self._forward_direction:
                self._current_wp_idx += 1
            else:
                self._current_wp_idx -= 1
            
            # Reset rotation in place flag when waypoint reached
            self._is_rotating_in_place = False
            self._rotation_start_time = None
            # Return zero steering to allow state update; provide default speed
            return 0.0, True, 100.0
        
        # Compute heading to waypoint and current heading
        current_heading = self._extract_heading(robot_pose)
        angle_to_goal = math.atan2(
            wp.pose.position.y - robot_pose.pose.position.y,
            wp.pose.position.x - robot_pose.pose.position.x
        )
        heading_error = self._normalize_angle(angle_to_goal - current_heading)
        heading_error_deg = math.degrees(heading_error)

        # If heading error large (>22.5°) rotate on the spot first.
        if abs(heading_error_deg) > 22.5:
            # Track rotation start time
            now = time.time()
            if self._rotation_start_time is None:
                self._rotation_start_time = now
            # Rotation timeout — skip waypoint if stuck rotating too long
            elif (now - self._rotation_start_time) > self._rotation_timeout:
                print(f"[WP_FOLLOWER] Rotation timeout after {self._rotation_timeout}s on waypoint {self._current_wp_idx} (heading error={heading_error_deg:.1f}°) — skipping")
                self._rotation_start_time = None
                self._is_rotating_in_place = False
                # Advance to next waypoint
                if self._forward_direction:
                    self._current_wp_idx += 1
                else:
                    self._current_wp_idx -= 1
                return 0.0, True, 0.0
            # compute rotation steering with deadlock boost and return zero speed
            current_yaw_deg = math.degrees(current_heading)
            steering = self._compute_rotation_steering(heading_error_deg, current_yaw_deg)
            self._is_rotating_in_place = True
            return steering, True, 0.0

        # Otherwise compute normal steering and drive at full speed
        self._is_rotating_in_place = False
        self._rotation_start_time = None  # Reset timer when aligned
        steering = self.compute_steering(robot_pose, wp)
        return steering, True, 100.0
    
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
        return len(self._waypoints)
    
    def find_nearest_waypoint(self, robot_pose: PoseStamped) -> int:
        """Find the index of the nearest waypoint.
        
        Args:
            robot_pose: Current robot pose
            
        Returns:
            Index of nearest waypoint
        """
        if len(self._waypoints) == 0:
            return 0
        
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, wp in enumerate(self._waypoints):
            dist = self.compute_distance(robot_pose, wp)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def is_rotating_in_place(self) -> bool:
        """Check if the robot is currently rotating in place.
        
        Returns:
            True if rotating in place (heading error > 22.5°), False otherwise
        """
        return self._is_rotating_in_place

