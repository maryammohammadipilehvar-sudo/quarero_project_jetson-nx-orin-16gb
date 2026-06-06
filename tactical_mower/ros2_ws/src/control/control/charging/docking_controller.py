"""Line-Based Docking Controller for Differential Drive Robots.

This controller implements a line-following approach with discretized path:
1. Calculate a line from home position (2m behind charge point) to charge point
2. Discretize the line into segments (default 10cm)
3. Move to the line at home position
4. Rotate to align with line direction
5. Follow the line using cost function optimization with lookahead

Key Features:
- Discretized path for precise tracking
- Cost function: w_lateral * |lateral_error| + w_heading * |heading_error|
- Lookahead point for smooth trajectory
- Signed lateral error (positive = left of line, negative = right)
- Critical zone enforcement (last 50cm must be within ±3cm)
- Constant speed (30% of max_speed)
- Continuous yaw correction
"""

import math
import time
from enum import IntEnum
from typing import Optional, Tuple, Callable, List
from dataclasses import dataclass, field
from geometry_msgs.msg import PoseStamped


class DockingState(IntEnum):
    """Docking operation states."""
    IDLE = 0
    APPROACH_LINE = 10      # Move to line at home position
    ALIGN_TO_LINE = 20      # Rotate to face charge point
    FOLLOW_LINE = 30        # Follow discretized line with corrections
    DOCKED = 40
    UNDOCKING = 50          # Move backwards along line (from charge to home)


@dataclass
class DockingConfig:
    """Configuration parameters for docking controller."""
    # GPS Sensor Offset (meters)
    gps_offset_x: float = 0.0  # GPS is 18cm in front of rotation center
    gps_offset_y: float = 0.0   # GPS is centered laterally
    
    # Speed settings
    max_speed: float = 1.0
    docking_speed_ratio: float = 0.5  # 50% of max_speed — used in critical/final zone (raised from 0.3 for grass traction at low battery)
    docking_approach_speed_ratio: float = 0.7  # 70% of max_speed — used before critical zone (raised from 0.5)
    undock_speed_ratio: float = 0.5    # 50% of max_speed for undocking (backwards) - raised from 0.4
    
    # Undocking settings
    home_position_tolerance: float = 0.40  # Tolerance for reaching home position during undocking (40cm)
    
    # Undocking-specific control gains (more aggressive than docking)
    undock_steering_gain_lateral_p: float = 2.0    # Proportional gain for lateral correction (higher than docking)
    undock_steering_gain_lateral_d: float = 0.3    # Derivative gain for lateral correction
    undock_steering_gain_heading_p: float = 2.0    # Proportional gain for heading (higher than docking)
    undock_steering_gain_heading_d: float = 0.3    # Derivative gain for heading
    undock_max_lateral_correction_deg: float = 20.0  # Max steering from lateral error (degrees) - larger than docking
    undock_lateral_multiplier: float = 80.0         # Multiplier for lateral error to degrees (higher = more aggressive)
    
    # Steering settings
    max_steering: float = 100.0
    
    # Control frequency (Hz) - recommended 50 Hz for responsive control
    control_frequency: float = 50.0
    
    # Path discretization
    path_resolution: float = 0.10  # 10cm segments
    
    # Distance settings (meters)
    line_approach_tolerance: float = 0.03   # 3cm - get very close to line before rotating
    charge_position_tolerance: float = 0.05 # Final docking precision (5cm)
    
    # Critical zone (last 50cm)
    critical_zone_distance: float = 0.50    # Start of critical zone
    critical_zone_lateral_tolerance: float = 0.03  # ±3cm in critical zone
    
    # Final approach zone (last 30cm) - must be very precise
    final_approach_distance: float = 0.30   # Last 30cm before charge point
    
    # Alignment settings (degrees)
    alignment_tolerance: float = 2.0        # Tolerance for yaw alignment
    
    # Lookahead
    lookahead_distance: float = 0.30        # 30cm lookahead on line
    
    # Cost function weights
    weight_lateral: float = 2.0             # Weight for lateral error
    weight_heading: float = 1.0             # Weight for heading error
    weight_lateral_critical: float = 5.0    # Weight in critical zone
    weight_lateral_final: float = 15.0      # Weight in final approach zone
    
    # Steering limits
    max_lateral_correction_deg: float = 12.0    # Max steering from lateral error (degrees)
    max_lateral_correction_final_deg: float = 8.0  # Even less in final approach
    
    # PD Controller gains (Proportional + Derivative)
    # Lateral correction
    steering_gain_lateral_p: float = 2.0    # Proportional gain for lateral
    steering_gain_lateral_d: float = 0.0    # Derivative gain for lateral (disabled: was 0.5)
    
    # Heading correction  
    steering_gain_heading_p: float = 1.5    # Proportional gain for heading
    steering_gain_heading_d: float = 0.0    # Derivative gain for heading (disabled: was 0.3)
    
    # Pure rotation
    rotation_gain_p: float = 1.5            # Proportional gain for rotation
    rotation_gain_d: float = 0.0            # Derivative gain for rotation (disabled: was 0.3)
    
    # Minimum steering (deadband compensation)
    min_steering_command: float = 12.0      # Minimum steering to overcome friction/deadband
    
    # Deadlock detection (adaptive steering increase)
    deadlock_check_interval: float = 0.5    # Check every 0.5 seconds
    deadlock_yaw_threshold: float = 0.3     # Degrees: if yaw change < this, consider stuck
    deadlock_steering_increment: float = 3.0  # Increase steering by this amount when stuck
    deadlock_max_steering: float = 60.0     # Maximum steering when trying to break deadlock


@dataclass 
class DockingDebugInfo:
    """Debug information for monitoring docking progress."""
    state: str = "IDLE"
    sub_state: str = ""
    
    # Positions
    robot_position: Tuple[float, float] = (0.0, 0.0)
    raw_gps_position: Tuple[float, float] = (0.0, 0.0)
    home_position: Optional[Tuple[float, float]] = None
    charge_position: Optional[Tuple[float, float]] = None
    
    # Line information
    line_direction_deg: float = 0.0
    num_path_points: int = 0
    current_target_index: int = 0
    lookahead_point: Optional[Tuple[float, float]] = None
    
    # Errors
    lateral_error: float = 0.0              # Signed: positive = left, negative = right
    heading_error: float = 0.0              # Degrees
    distance_to_charge: float = 0.0
    
    # Derivative terms (error rate of change)
    lateral_error_derivative: float = 0.0   # m/s
    heading_error_derivative: float = 0.0   # deg/s
    
    # Cost function
    current_cost: float = 0.0
    in_critical_zone: bool = False
    
    # Angles (degrees)
    current_yaw: float = 0.0
    target_yaw: float = 0.0
    
    # Control outputs
    steering_command: float = 0.0
    speed_command: float = 0.0
    steering_p_component: float = 0.0       # Proportional contribution
    steering_d_component: float = 0.0       # Derivative contribution
    steering_deadlock_boost: float = 0.0    # Deadlock compensation
    
    # Timing
    control_dt: float = 0.0                 # Actual time between updates
    
    # Warnings
    warnings: List[str] = field(default_factory=list)


class CoordinateTransformerInterface:
    """Interface for coordinate transformation (GPS <-> ENU)."""
    
    def set_origin(self, lat: float, lon: float, alt: float) -> None:
        raise NotImplementedError
    
    def gps_to_enu(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        raise NotImplementedError


class DockingController:
    """Line-based docking controller with discretized path and cost function."""
    
    def __init__(
        self,
        coordinate_transformer: CoordinateTransformerInterface,
        config: Optional[DockingConfig] = None
    ):
        self.coord_transformer = coordinate_transformer
        self.config = config or DockingConfig()
        
        # State
        self._state = DockingState.IDLE
        
        # Target positions
        self._charge_point_enu: Optional[Tuple[float, float]] = None
        self._charge_yaw_rad: Optional[float] = None
        self._home_position_enu: Optional[Tuple[float, float]] = None
        
        # Discretized path (list of points from home to charge)
        self._path_points: List[Tuple[float, float]] = []
        self._line_direction_rad: Optional[float] = None
        
        # Derivative tracking (for PD control)
        self._last_lateral_error: Optional[float] = None
        self._last_heading_error: Optional[float] = None
        self._last_update_time: Optional[float] = None
        
        # Deadlock detection
        self._deadlock_check_yaw: Optional[float] = None
        self._deadlock_check_time: Optional[float] = None
        self._deadlock_steering_boost: float = 0.0
        
        # Undocking state
        self._undock_start_position: Optional[Tuple[float, float]] = None
        self._undock_start_time: Optional[float] = None  # Track when undocking started
        
        # Debug
        self._debug = DockingDebugInfo()
    
    def start_docking(
        self,
        target_gps: Tuple[float, float, float],
        home_gps: Tuple[float, float, float],
        origin_gps: Tuple[float, float, float]
    ) -> bool:
        """Start docking procedure.
        
        Args:
            target_gps: Target charging position (lat, lon, yaw_degrees)
            home_gps: Home position to approach from (lat, lon, yaw_degrees)
            origin_gps: Origin for ENU conversion (lat, lon, alt)
            
        Returns:
            True if docking started successfully
        """
        if self._state != DockingState.IDLE:
            return False
        
        # Set coordinate origin
        self.coord_transformer.set_origin(origin_gps[0], origin_gps[1], origin_gps[2])
        
        # Convert charge point to ENU
        charge_enu = self.coord_transformer.gps_to_enu(target_gps[0], target_gps[1], 0.0)
        self._charge_yaw_rad = math.radians(target_gps[2])
        
        # IMPORTANT: Offset charge point forward by GPS offset
        # The GPS sensor is gps_offset_x ahead of rotation center
        # So when GPS reports "at charge point", rotation center is still gps_offset_x behind
        # We shift the target forward so rotation center ends up at the actual charge point
        offset_x = self.config.gps_offset_x * math.cos(self._charge_yaw_rad)
        offset_y = self.config.gps_offset_x * math.sin(self._charge_yaw_rad)
        self._charge_point_enu = (
            charge_enu[0] - offset_x,
            charge_enu[1] - offset_y
        )
        
        # Convert home position to ENU (no offset needed - we just need to be on the line)
        home_enu = self.coord_transformer.gps_to_enu(home_gps[0], home_gps[1], 0.0)
        self._home_position_enu = (home_enu[0], home_enu[1])
        
        # Line direction (from home to charge)
        dx = self._charge_point_enu[0] - self._home_position_enu[0]
        dy = self._charge_point_enu[1] - self._home_position_enu[1]
        self._line_direction_rad = math.atan2(dy, dx)
        
        # Discretize path
        self._discretize_path()
        
        # Reset derivative tracking
        self._last_lateral_error = None
        self._last_heading_error = None
        self._last_update_time = time.monotonic()
        
        # Reset deadlock detection
        self._deadlock_check_yaw = None
        self._deadlock_check_time = None
        self._deadlock_steering_boost = 0.0
        
        # Start state machine
        self._state = DockingState.APPROACH_LINE
        self._debug.warnings.clear()
        
        return True
    
    def start_undocking(
        self,
        charge_gps: Tuple[float, float, float],
        home_gps: Tuple[float, float, float],
        origin_gps: Tuple[float, float, float],
        current_robot_position_enu: Tuple[float, float],
        current_robot_heading_rad: Optional[float] = None
    ) -> bool:
        """Start undocking procedure.
        
        Args:
            charge_gps: Charge position (lat, lon, yaw_degrees)
            home_gps: Home position (lat, lon, yaw_degrees)
            origin_gps: Origin for ENU conversion (lat, lon, alt)
            current_robot_position_enu: Current robot position in ENU (x, y)
            
        Returns:
            True if undocking started successfully
        """
        # Debug: Check state comparison
        state_value = int(self._state) if hasattr(self._state, '__int__') else self._state
        docked_value = int(DockingState.DOCKED)
        idle_value = int(DockingState.IDLE)
        undocking_value = int(DockingState.UNDOCKING)
        
        # Allow starting undocking from DOCKED, IDLE, or UNDOCKING (restart if stuck)
        if state_value not in (docked_value, idle_value, undocking_value):
            return False
        
        # If already undocking, reset first to restart
        if state_value == undocking_value:
            self.reset()
        

        try:
            # Set coordinate origin
            self.coord_transformer.set_origin(origin_gps[0], origin_gps[1], origin_gps[2])
            
            # Convert charge point to ENU
            charge_enu = self.coord_transformer.gps_to_enu(charge_gps[0], charge_gps[1], 0.0)
            self._charge_yaw_rad = math.radians(charge_gps[2])
            
            # Offset charge point forward by GPS offset (same as docking)
            offset_x = self.config.gps_offset_x * math.cos(self._charge_yaw_rad)
            offset_y = self.config.gps_offset_x * math.sin(self._charge_yaw_rad)
            self._charge_point_enu = (
                charge_enu[0] - offset_x,
                charge_enu[1] - offset_y
            )
            
            # Convert home position to ENU
            home_enu = self.coord_transformer.gps_to_enu(home_gps[0], home_gps[1], 0.0)
            self._home_position_enu = (home_enu[0], home_enu[1])
            
            # Line direction (from charge to home - opposite of docking)
            dx = self._home_position_enu[0] - self._charge_point_enu[0]
            dy = self._home_position_enu[1] - self._charge_point_enu[1]
            self._line_direction_rad = math.atan2(dy, dx)
            
            # Store starting position for undocking (where we start from)
            self._undock_start_position = current_robot_position_enu
            
            # Reset derivative tracking
            self._last_lateral_error = None
            self._last_heading_error = None
            self._last_update_time = time.monotonic()
            
            # Reset deadlock detection
            self._deadlock_check_yaw = None
            self._deadlock_check_time = None
            self._deadlock_steering_boost = 0.0
            
            # Record undocking start time for initial speed boost
            self._undock_start_time = time.monotonic()
            
            # Start undocking state
            self._state = DockingState.UNDOCKING
            self._debug.warnings.clear()
            self._debug.state = "UNDOCKING"
            
            return True
        except Exception as e:
            # Exception in start_undocking - return False to indicate failure
            # Error details should be logged by the caller if logger is available
            return False
    
    def _discretize_path(self) -> None:
        """Discretize the line from home to charge into segments."""
        if not self._home_position_enu or not self._charge_point_enu:
            return
        
        self._path_points.clear()
        
        dx = self._charge_point_enu[0] - self._home_position_enu[0]
        dy = self._charge_point_enu[1] - self._home_position_enu[1]
        total_distance = math.sqrt(dx * dx + dy * dy)
        
        num_segments = max(1, int(total_distance / self.config.path_resolution))
        
        for i in range(num_segments + 1):
            t = i / num_segments
            x = self._home_position_enu[0] + t * dx
            y = self._home_position_enu[1] + t * dy
            self._path_points.append((x, y))
        
        # Ensure charge point is the last point
        if self._path_points[-1] != self._charge_point_enu:
            self._path_points.append(self._charge_point_enu)
    
    def get_state(self) -> DockingState:
        """Get current docking state."""
        return self._state
    
    def get_debug_info(self) -> DockingDebugInfo:
        """Get debug information."""
        return self._debug
    
    def is_active(self) -> bool:
        """Check if docking is active."""
        return self._state != DockingState.IDLE
    
    def reset(self) -> None:
        """Reset controller to idle state."""
        self._state = DockingState.IDLE
        self._charge_point_enu = None
        self._charge_yaw_rad = None
        self._home_position_enu = None
        self._path_points.clear()
        self._line_direction_rad = None
        self._last_lateral_error = None
        self._last_heading_error = None
        self._last_update_time = None
        self._deadlock_check_yaw = None
        self._deadlock_check_time = None
        self._deadlock_steering_boost = 0.0
        self._undock_start_position = None
        self._undock_start_time = None
        self._debug = DockingDebugInfo()
    
    def update(
        self,
        robot_pose: PoseStamped,
        on_home_reached: Optional[Callable[[], None]] = None,
        on_aligned: Optional[Callable[[], None]] = None,
        on_docked: Optional[Callable[[], None]] = None,
        on_undock_complete: Optional[Callable[[], None]] = None
    ) -> Tuple[Optional[float], Optional[float]]:
        """Update controller and get commands.
        
        Args:
            robot_pose: Current robot pose in ENU frame
            on_home_reached: Callback when line reached at home position
            on_aligned: Callback when aligned to line direction
            on_docked: Callback when docking complete
            
        Returns:
            Tuple of (steering, speed) commands, or (None, None) if idle
        """
        if self._state == DockingState.IDLE:
            return None, None
        
        # Extract position and heading
        raw_x, raw_y = robot_pose.pose.position.x, robot_pose.pose.position.y
        heading = self._extract_heading(robot_pose)
        
        # Compensate GPS offset to get rotation center
        robot_x, robot_y = self._compensate_gps_offset(raw_x, raw_y, heading)
        
        # Update debug info
        self._debug.robot_position = (robot_x, robot_y)
        self._debug.raw_gps_position = (raw_x, raw_y)
        self._debug.current_yaw = math.degrees(heading)
        self._debug.home_position = self._home_position_enu
        self._debug.charge_position = self._charge_point_enu
        self._debug.num_path_points = len(self._path_points)
        
        if self._line_direction_rad is not None:
            self._debug.line_direction_deg = math.degrees(self._line_direction_rad)
        
        # Calculate distance to charge point
        if self._charge_point_enu:
            dx = self._charge_point_enu[0] - robot_x
            dy = self._charge_point_enu[1] - robot_y
            self._debug.distance_to_charge = math.sqrt(dx * dx + dy * dy)
        
        # Check if docked (but NOT if we're undocking - we start from charge_pos!)
        if self._state != DockingState.UNDOCKING:
            if self._debug.distance_to_charge < self.config.charge_position_tolerance:
                self._state = DockingState.DOCKED
                self._debug.state = "DOCKED"
                if on_docked:
                    on_docked()
                return 0.0, 0.0
        
        # Execute current state
        if self._state == DockingState.APPROACH_LINE:
            return self._execute_approach_line(robot_x, robot_y, heading, on_home_reached)
        elif self._state == DockingState.ALIGN_TO_LINE:
            return self._execute_align_to_line(robot_x, robot_y, heading, on_aligned)
        elif self._state == DockingState.FOLLOW_LINE:
            return self._execute_follow_line(robot_x, robot_y, heading, on_docked)
        elif self._state == DockingState.DOCKED:
            self._debug.state = "DOCKED"
            return 0.0, 0.0
        elif self._state == DockingState.UNDOCKING:
            return self._execute_undocking(robot_x, robot_y, heading, on_undock_complete)
        
        return None, None
    
    def _execute_approach_line(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]]
    ) -> Tuple[float, float]:
        """Move towards the line - drive perpendicular to reach line precisely."""
        self._debug.state = "APPROACH_LINE"
        
        if not self._home_position_enu or self._line_direction_rad is None:
            self.reset()
            return 0.0, 0.0
        
        # Calculate signed lateral error to the line
        lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
        self._debug.lateral_error = lateral_error
        
        # Calculate distance along line (negative = before home, positive = past home)
        distance_along = self._compute_distance_along_line(robot_x, robot_y)
        
        # Check if we've reached the line (very close laterally)
        if abs(lateral_error) < self.config.line_approach_tolerance:
            if on_complete:
                on_complete()
            self._state = DockingState.ALIGN_TO_LINE
            # Reset derivative tracking for next phase
            self._last_lateral_error = None
            self._last_heading_error = None
            return 0.0, 0.0
        
        # TARGET: Drive perpendicular to line to reach it
        # Perpendicular direction: line_direction ± 90°
        # Choose direction based on which side of line we are
        if lateral_error > 0:
            # Robot is LEFT of line, need to go RIGHT (line_direction - 90°)
            target_heading = self._line_direction_rad - math.pi / 2
        else:
            # Robot is RIGHT of line, need to go LEFT (line_direction + 90°)
            target_heading = self._line_direction_rad + math.pi / 2
        
        target_heading = self._normalize_angle(target_heading)
        
        heading_error = self._normalize_angle(target_heading - heading)
        heading_error_deg = math.degrees(heading_error)
        
        self._debug.target_yaw = math.degrees(target_heading)
        self._debug.heading_error = heading_error_deg
        
        # If heading error is large, rotate first
        if abs(heading_error_deg) > 30.0:
            self._debug.sub_state = "rotating_to_line"
            current_yaw_deg = math.degrees(heading)
            steering = self._compute_rotation_steering(heading_error_deg, current_yaw_deg)
            self._debug.steering_command = steering
            self._debug.speed_command = 0.0
            return steering, 0.0
        
        # Drive towards line with PD steering correction
        self._debug.sub_state = "driving_to_line"
        
        # Use PD control for heading while driving
        now = time.monotonic()
        dt = 0.02
        if self._last_update_time is not None:
            dt = max(0.001, now - self._last_update_time)
        
        # P term
        p_term = heading_error_deg * self.config.steering_gain_heading_p
        
        # D term
        d_term = 0.0
        if self._last_heading_error is not None:
            heading_rate = (heading_error_deg - self._last_heading_error) / dt
            d_term = heading_rate * self.config.steering_gain_heading_d
            self._debug.heading_error_derivative = heading_rate
        
        steering = p_term + d_term
        steering = max(-self.config.max_steering, min(self.config.max_steering, steering))
        
        # Update tracking
        self._last_heading_error = heading_error_deg
        self._last_update_time = now
        self._debug.control_dt = dt
        self._debug.steering_p_component = p_term
        self._debug.steering_d_component = d_term
        
        speed = self._get_constant_speed()
        
        self._debug.steering_command = steering
        self._debug.speed_command = speed
        
        return steering, speed
    
    def _execute_align_to_line(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]]
    ) -> Tuple[float, float]:
        """Rotate to align with line direction (towards charge point)."""
        self._debug.state = "ALIGN_TO_LINE"
        self._debug.sub_state = "rotating"
        
        if self._line_direction_rad is None:
            self.reset()
            return 0.0, 0.0
        
        # Calculate yaw error to line direction
        yaw_error = self._normalize_angle(self._line_direction_rad - heading)
        yaw_error_deg = math.degrees(yaw_error)
        
        self._debug.target_yaw = math.degrees(self._line_direction_rad)
        self._debug.heading_error = yaw_error_deg
        
        # Check if aligned
        if abs(yaw_error_deg) < self.config.alignment_tolerance:
            if on_complete:
                on_complete()
            self._state = DockingState.FOLLOW_LINE
            # Reset derivative tracking for next phase
            self._last_lateral_error = None
            self._last_heading_error = None
            self._deadlock_steering_boost = 0.0
            return 0.0, 0.0
        
        # Pure rotation with PD control and deadlock detection
        current_yaw_deg = math.degrees(heading)
        steering = self._compute_rotation_steering(yaw_error_deg, current_yaw_deg)
        
        self._debug.steering_command = steering
        self._debug.speed_command = 0.0
        
        return steering, 0.0
    
    def _execute_follow_line(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]]
    ) -> Tuple[float, float]:
        """Follow the discretized line with PD control.
        
        Strategy:
        - Heading should follow LINE DIRECTION (not lookahead point)
        - Lateral error causes small heading adjustment to drift back to line
        - This prevents oscillation from chasing a moving target
        
        Zones:
        - Normal: gentle corrections
        - Critical (50cm): tighter tolerance
        - Final (30cm): very precise, minimal corrections
        """
        self._debug.state = "FOLLOW_LINE"
        
        if not self._path_points or not self._charge_point_enu or self._line_direction_rad is None:
            self.reset()
            return 0.0, 0.0
        
        # Find closest point on path
        closest_idx, closest_point = self._find_closest_path_point(robot_x, robot_y)
        lookahead_point = self._get_lookahead_point(closest_idx, robot_x, robot_y)
        
        self._debug.current_target_index = closest_idx
        self._debug.lookahead_point = lookahead_point
        
        # Calculate signed lateral error
        lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
        self._debug.lateral_error = lateral_error
        
        # PRIMARY: Heading error relative to LINE DIRECTION (not lookahead)
        # This keeps the robot parallel to the line
        line_heading_error = self._normalize_angle(self._line_direction_rad - heading)
        line_heading_error_deg = math.degrees(line_heading_error)
        
        self._debug.target_yaw = math.degrees(self._line_direction_rad)
        self._debug.heading_error = line_heading_error_deg
        
        # Determine zone and select parameters
        distance_to_charge = self._debug.distance_to_charge
        in_final_zone = distance_to_charge < self.config.final_approach_distance
        in_critical_zone = distance_to_charge < self.config.critical_zone_distance
        
        self._debug.in_critical_zone = in_critical_zone
        
        if in_final_zone:
            # Final 30cm - very precise, minimal corrections
            w_lateral = self.config.weight_lateral_final
            max_lateral_correction = self.config.max_lateral_correction_final_deg
            self._debug.sub_state = "final_approach"
            
            if abs(lateral_error) > self.config.critical_zone_lateral_tolerance:
                self._debug.warnings.append(
                    f"Final zone: lateral error {lateral_error*100:.1f}cm exceeds ±{self.config.critical_zone_lateral_tolerance*100:.0f}cm"
                )
        elif in_critical_zone:
            # Critical zone 50-30cm - tighter control
            w_lateral = self.config.weight_lateral_critical
            max_lateral_correction = self.config.max_lateral_correction_deg
            self._debug.sub_state = "critical_zone"
            
            if abs(lateral_error) > self.config.critical_zone_lateral_tolerance:
                self._debug.warnings.append(
                    f"Critical zone: lateral error {lateral_error*100:.1f}cm exceeds ±{self.config.critical_zone_lateral_tolerance*100:.0f}cm"
                )
        else:
            # Normal following
            w_lateral = self.config.weight_lateral
            max_lateral_correction = self.config.max_lateral_correction_deg
            self._debug.sub_state = "following"
        
        # Compute cost
        cost = w_lateral * abs(lateral_error) + self.config.weight_heading * abs(math.radians(line_heading_error_deg))
        self._debug.current_cost = cost
        
        # STEERING STRATEGY:
        # 1. Heading correction: align with line direction
        # 2. Lateral correction: add small angle offset to drift back to line
        
        now = time.monotonic()
        dt = 0.02
        if self._last_update_time is not None:
            dt = max(0.001, now - self._last_update_time)
        self._debug.control_dt = dt
        
        # Heading P term - keep parallel to line
        heading_p = line_heading_error_deg * self.config.steering_gain_heading_p
        
        # Heading D term
        heading_d = 0.0
        if self._last_heading_error is not None:
            heading_rate = (line_heading_error_deg - self._last_heading_error) / dt
            heading_d = heading_rate * self.config.steering_gain_heading_d
            self._debug.heading_error_derivative = heading_rate
        
        # Lateral correction - convert lateral error to heading adjustment
        # Base: 10cm off = ~5° correction
        lateral_correction_deg = -lateral_error * self.config.steering_gain_lateral_p * 50.0
        
        # Clamp lateral correction based on zone
        lateral_correction_deg = max(-max_lateral_correction, min(max_lateral_correction, lateral_correction_deg))
        
        # Lateral D term - dampen oscillation
        lateral_d = 0.0
        if self._last_lateral_error is not None:
            lateral_rate = (lateral_error - self._last_lateral_error) / dt
            lateral_d = -lateral_rate * self.config.steering_gain_lateral_d * 50.0
            # Also clamp D term
            lateral_d = max(-max_lateral_correction, min(max_lateral_correction, lateral_d))
            self._debug.lateral_error_derivative = lateral_rate
        
        # Combine: heading alignment + lateral drift correction
        p_total = heading_p + lateral_correction_deg
        d_total = heading_d + lateral_d
        
        steering = p_total + d_total
        
        # NO min_steering here - smooth corrections are important for line following
        
        steering = max(-self.config.max_steering, min(self.config.max_steering, steering))
        
        # Update tracking
        self._last_lateral_error = lateral_error
        self._last_heading_error = line_heading_error_deg
        self._last_update_time = now
        
        self._debug.steering_p_component = p_total
        self._debug.steering_d_component = d_total
        
        # Speed: higher approach speed outside critical zone for grass traction
        if in_critical_zone:
            speed = self._get_constant_speed()
        else:
            speed = self.config.docking_approach_speed_ratio * 100.0

        self._debug.steering_command = steering
        self._debug.speed_command = speed

        # Check if docked
        if distance_to_charge < self.config.charge_position_tolerance:
            self._state = DockingState.DOCKED
            if on_complete:
                on_complete()
            return 0.0, 0.0
        
        return steering, speed
    
    def _execute_undocking(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]]
    ) -> Tuple[float, float]:
        """Execute undocking: move backwards along the line from charge to home.
        
        Uses the same line-following logic as docking, but moving backwards.
        Maintains heading parallel to line (backwards direction) and corrects lateral errors.
        
        Args:
            robot_x: Robot x position (rotation center) in ENU
            robot_y: Robot y position (rotation center) in ENU
            heading: Current robot heading in radians
            on_complete: Callback when undocking is complete
            
        Returns:
            Tuple of (steering, speed) commands
        """
        self._debug.state = "UNDOCKING"
        self._debug.sub_state = "following_line_backwards"
        
        if not self._home_position_enu or not self._charge_point_enu or self._line_direction_rad is None:
            # Log warning with details about what's missing
            missing = []
            if not self._home_position_enu:
                missing.append("home_position_enu")
            if not self._charge_point_enu:
                missing.append("charge_point_enu")
            if self._line_direction_rad is None:
                missing.append("line_direction_rad")
            # Missing required data for undocking - resetting and stopping
            # Error details should be logged by the caller if logger is available
            self.reset()
            return 0.0, 0.0
        
        # Check if robot has reached home position
        # Use two criteria:
        # 1. Distance along the line: if robot has moved backwards past home, stop
        # 2. Euclidean distance: if robot is close enough to home, stop
        
        # Calculate distance along line from charge to robot (projection)
        # Line direction is from charge to home
        dx_line = self._home_position_enu[0] - self._charge_point_enu[0]
        dy_line = self._home_position_enu[1] - self._charge_point_enu[1]
        line_length = math.sqrt(dx_line * dx_line + dy_line * dy_line)
        
        if line_length < 1e-6:
            # Invalid line, stop
            if on_complete:
                on_complete()
            self._state = DockingState.IDLE
            self._debug.state = "IDLE"
            return 0.0, 0.0
        
        # Normalize line direction vector
        line_dir_x = dx_line / line_length
        line_dir_y = dy_line / line_length
        
        # Vector from charge to robot
        dx_robot = robot_x - self._charge_point_enu[0]
        dy_robot = robot_y - self._charge_point_enu[1]
        
        # Project robot position onto line (distance along line from charge)
        distance_along_line = dx_robot * line_dir_x + dy_robot * line_dir_y
        
        # Euclidean distance to home
        dx_to_home = robot_x - self._home_position_enu[0]
        dy_to_home = robot_y - self._home_position_enu[1]
        distance_to_home = math.sqrt(dx_to_home * dx_to_home + dy_to_home * dy_to_home)
        
        self._debug.distance_to_charge = distance_to_home
        
        # Stop if:
        # 1. Robot has moved backwards past home position along the line (distance_along_line >= line_length)
        # 2. OR robot is within tolerance of home position (Euclidean distance)
        # 3. OR robot is close to home AND has moved backwards enough (within 0.5m of home along line)
        has_passed_home = distance_along_line >= line_length
        is_close_to_home = distance_to_home <= self.config.home_position_tolerance
        is_near_home_along_line = distance_along_line >= (line_length - 0.5)  # Within 0.5m of home along line
        
        if has_passed_home or is_close_to_home or (is_near_home_along_line and distance_to_home <= 0.6):
            if on_complete:
                on_complete()
            self._state = DockingState.IDLE
            self._debug.state = "IDLE"
            return 0.0, 0.0
        
        # Calculate signed lateral error (to stay on line)
        lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
        self._debug.lateral_error = lateral_error
        
        # Calculate how close we are to home (as fraction of line length)
        # Reduce corrections and speed when very close to home to avoid overshooting
        distance_remaining = line_length - distance_along_line
        close_to_home_factor = min(1.0, max(0.3, distance_remaining / 1.0))  # Reduce corrections when within 1m of home
        
        # Calculate heading error - robot should face backwards along line
        # Line direction is from charge to home, so backwards is opposite (180°)
        target_heading = self._normalize_angle(self._line_direction_rad + math.pi)
        line_heading_error = self._normalize_angle(target_heading - heading)
        line_heading_error_deg = math.degrees(line_heading_error)
        
        self._debug.target_yaw = math.degrees(target_heading)
        self._debug.heading_error = line_heading_error_deg
        
        # Use same line-following strategy as docking, but backwards with more aggressive gains
        # Strategy: Maintain heading parallel to line (backwards) and correct lateral drift
        
        now = time.monotonic()
        dt = 0.02
        if self._last_update_time is not None:
            dt = max(0.001, now - self._last_update_time)
        self._debug.control_dt = dt
        
        # Heading P term - keep parallel to line (backwards direction)
        # Use undocking-specific gains (higher than docking for more precision)
        heading_p = line_heading_error_deg * self.config.undock_steering_gain_heading_p
        
        # Heading D term - use undocking-specific gain
        heading_d = 0.0
        if self._last_heading_error is not None:
            heading_rate = (line_heading_error_deg - self._last_heading_error) / dt
            heading_d = heading_rate * self.config.undock_steering_gain_heading_d
            self._debug.heading_error_derivative = heading_rate
        
        # Lateral correction - convert lateral error to heading adjustment
        # Use undocking-specific gains and multiplier (more aggressive than docking)
        # Higher multiplier means more steering for same lateral error
        # Reduce corrections when close to home to avoid overshooting
        lateral_correction_deg = -lateral_error * self.config.undock_steering_gain_lateral_p * self.config.undock_lateral_multiplier * close_to_home_factor
        
        # Clamp lateral correction using undocking-specific max (larger than docking)
        # Also reduce max correction when close to home
        max_lateral_correction = self.config.undock_max_lateral_correction_deg * close_to_home_factor
        lateral_correction_deg = max(-max_lateral_correction, min(max_lateral_correction, lateral_correction_deg))
        
        # Lateral D term - dampen oscillation with undocking-specific gain
        lateral_d = 0.0
        if self._last_lateral_error is not None:
            lateral_rate = (lateral_error - self._last_lateral_error) / dt
            lateral_d = -lateral_rate * self.config.undock_steering_gain_lateral_d * self.config.undock_lateral_multiplier
            # Also clamp D term
            lateral_d = max(-max_lateral_correction, min(max_lateral_correction, lateral_d))
            self._debug.lateral_error_derivative = lateral_rate
        
        # Combine: heading alignment + lateral drift correction
        p_total = heading_p + lateral_correction_deg
        d_total = heading_d + lateral_d
        
        steering = p_total + d_total
        
        # Clamp to max steering
        steering = max(-self.config.max_steering, min(self.config.max_steering, steering))
        
        # Update tracking
        self._last_lateral_error = lateral_error
        self._last_heading_error = line_heading_error_deg
        self._last_update_time = now
        
        self._debug.steering_p_component = p_total
        self._debug.steering_d_component = d_total
        
        # Move backwards (negative speed)
        speed = -self.config.undock_speed_ratio * 100.0  # Negative for backwards
        
        self._debug.steering_command = steering
        self._debug.speed_command = speed
        
        return steering, speed
    
    def _compute_signed_lateral_error(self, robot_x: float, robot_y: float) -> float:
        """Compute signed lateral error from robot to line.
        
        Positive = robot is LEFT of line (when facing charge point)
        Negative = robot is RIGHT of line
        
        Uses cross product to determine side.
        """
        if not self._home_position_enu or not self._charge_point_enu:
            return 0.0
        
        # Line vector (from home to charge)
        line_dx = self._charge_point_enu[0] - self._home_position_enu[0]
        line_dy = self._charge_point_enu[1] - self._home_position_enu[1]
        line_length = math.sqrt(line_dx * line_dx + line_dy * line_dy)
        
        if line_length < 1e-6:
            return 0.0
        
        # Vector from home to robot
        robot_dx = robot_x - self._home_position_enu[0]
        robot_dy = robot_y - self._home_position_enu[1]
        
        # Cross product: positive if robot is left of line
        cross = line_dx * robot_dy - line_dy * robot_dx
        
        # Signed distance (positive = left, negative = right)
        signed_distance = cross / line_length
        
        return signed_distance
    
    def _compute_distance_along_line(self, robot_x: float, robot_y: float) -> float:
        """Compute distance along line from home position.
        
        Negative = before home (behind)
        Positive = past home (towards charge)
        """
        if not self._home_position_enu or not self._charge_point_enu:
            return 0.0
        
        # Line vector
        line_dx = self._charge_point_enu[0] - self._home_position_enu[0]
        line_dy = self._charge_point_enu[1] - self._home_position_enu[1]
        line_length = math.sqrt(line_dx * line_dx + line_dy * line_dy)
        
        if line_length < 1e-6:
            return 0.0
        
        # Vector from home to robot
        robot_dx = robot_x - self._home_position_enu[0]
        robot_dy = robot_y - self._home_position_enu[1]
        
        # Dot product gives projection
        dot = line_dx * robot_dx + line_dy * robot_dy
        distance_along = dot / line_length
        
        return distance_along
    
    def _find_closest_path_point(
        self,
        robot_x: float,
        robot_y: float
    ) -> Tuple[int, Tuple[float, float]]:
        """Find the closest point on the discretized path."""
        if not self._path_points:
            return 0, (robot_x, robot_y)
        
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(self._path_points):
            dx = point[0] - robot_x
            dy = point[1] - robot_y
            dist = dx * dx + dy * dy
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx, self._path_points[closest_idx]
    
    def _get_lookahead_point(
        self,
        current_idx: int,
        robot_x: float,
        robot_y: float
    ) -> Tuple[float, float]:
        """Get lookahead point on path ahead of current position."""
        if not self._path_points:
            return (robot_x, robot_y)
        
        # Calculate how many points to look ahead
        points_ahead = int(self.config.lookahead_distance / self.config.path_resolution)
        points_ahead = max(1, points_ahead)
        
        # Get lookahead index, clamped to path length
        lookahead_idx = min(current_idx + points_ahead, len(self._path_points) - 1)
        
        return self._path_points[lookahead_idx]
    
    def _compute_rotation_steering(self, error_deg: float, current_yaw_deg: float) -> float:
        """Compute steering for pure rotation with PD control and deadlock detection.
        
        Args:
            error_deg: Yaw error in degrees
            current_yaw_deg: Current yaw in degrees (for deadlock detection)
        """
        # If error is within tolerance, no steering needed
        if abs(error_deg) < self.config.alignment_tolerance:
            self._deadlock_steering_boost = 0.0
            return 0.0
        
        now = time.monotonic()
        
        # Deadlock detection: check if yaw is changing
        if self._deadlock_check_time is None:
            self._deadlock_check_time = now
            self._deadlock_check_yaw = current_yaw_deg
        elif (now - self._deadlock_check_time) >= self.config.deadlock_check_interval:
            if self._deadlock_check_yaw is not None:
                yaw_change = abs(current_yaw_deg - self._deadlock_check_yaw)
                
                if yaw_change < self.config.deadlock_yaw_threshold:
                    # Robot is stuck - increase steering
                    self._deadlock_steering_boost = min(
                        self._deadlock_steering_boost + self.config.deadlock_steering_increment,
                        self.config.deadlock_max_steering - self.config.min_steering_command
                    )
                    self._debug.warnings.append(
                        f"Deadlock: yaw_change={yaw_change:.2f}°, boost={self._deadlock_steering_boost:.1f}"
                    )
                else:
                    # Robot is moving - reduce boost gradually
                    self._deadlock_steering_boost = max(0.0, self._deadlock_steering_boost - self.config.deadlock_steering_increment)
            
            # Reset check
            self._deadlock_check_time = now
            self._deadlock_check_yaw = current_yaw_deg
        
        # Calculate derivative
        d_term = 0.0
        if self._last_heading_error is not None and self._last_update_time is not None:
            dt = now - self._last_update_time
            if dt > 0.001:
                error_rate = (error_deg - self._last_heading_error) / dt
                d_term = error_rate * self.config.rotation_gain_d
                self._debug.heading_error_derivative = error_rate
                self._debug.control_dt = dt
        
        # P term
        p_term = error_deg * self.config.rotation_gain_p
        
        # Combined PD
        steering = p_term + d_term
        
        # Apply minimum steering ONLY for large rotations (>10°)
        # For small corrections, allow smooth low-speed rotation
        if abs(error_deg) > 10.0:
            if abs(steering) < self.config.min_steering_command:
                steering = math.copysign(self.config.min_steering_command, steering)
        
        # Add deadlock boost (same sign as steering)
        steering = steering + math.copysign(self._deadlock_steering_boost, steering)
        
        # Clamp to max
        steering = max(-self.config.max_steering, min(self.config.max_steering, steering))
        
        # Update tracking
        self._last_heading_error = error_deg
        self._last_update_time = now
        
        self._debug.steering_p_component = p_term
        self._debug.steering_d_component = d_term
        self._debug.steering_deadlock_boost = self._deadlock_steering_boost
        
        return steering
    
    def _compute_pd_steering(
        self,
        lateral_error: float,
        heading_error_deg: float,
        weight_lateral: float
    ) -> float:
        """Compute steering with PD control for both lateral and heading errors.
        
        Args:
            lateral_error: Signed lateral error in meters
            heading_error_deg: Heading error in degrees
            weight_lateral: Current weight for lateral error (higher in critical zone)
            
        Returns:
            Steering command
        """
        now = time.monotonic()
        dt = 0.02  # Default 50Hz
        
        if self._last_update_time is not None:
            dt = now - self._last_update_time
            if dt < 0.001:
                dt = 0.02
        
        self._debug.control_dt = dt
        
        # Lateral PD
        lateral_p = -lateral_error * self.config.steering_gain_lateral_p * (weight_lateral / self.config.weight_lateral)
        lateral_d = 0.0
        
        if self._last_lateral_error is not None:
            lateral_error_rate = (lateral_error - self._last_lateral_error) / dt
            lateral_d = -lateral_error_rate * self.config.steering_gain_lateral_d
            self._debug.lateral_error_derivative = lateral_error_rate
        
        # Heading PD
        heading_p = heading_error_deg * self.config.steering_gain_heading_p
        heading_d = 0.0
        
        if self._last_heading_error is not None:
            heading_error_rate = (heading_error_deg - self._last_heading_error) / dt
            heading_d = heading_error_rate * self.config.steering_gain_heading_d
            self._debug.heading_error_derivative = heading_error_rate
        
        # Combine
        p_total = lateral_p * self.config.max_steering + heading_p
        d_total = lateral_d * self.config.max_steering + heading_d
        
        steering = p_total + d_total
        
        # Apply minimum steering if there's a significant error
        if abs(lateral_error) > 0.01 or abs(heading_error_deg) > 1.0:  # 1cm or 1°
            if abs(steering) < self.config.min_steering_command:
                steering = math.copysign(self.config.min_steering_command, steering)
        
        steering = max(-self.config.max_steering, min(self.config.max_steering, steering))
        
        # Update tracking
        self._last_lateral_error = lateral_error
        self._last_heading_error = heading_error_deg
        self._last_update_time = now
        
        self._debug.steering_p_component = p_total
        self._debug.steering_d_component = d_total
        
        return steering
    
    def _get_constant_speed(self) -> float:
        """Get constant docking speed (30% of max)."""
        return self.config.docking_speed_ratio * 100.0  # As percentage
    
    def _compensate_gps_offset(
        self,
        gps_x: float,
        gps_y: float,
        heading: float
    ) -> Tuple[float, float]:
        """Transform GPS position to robot rotation center."""
        offset_x = self.config.gps_offset_x
        offset_y = self.config.gps_offset_y
        
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        
        # GPS offset in world frame
        world_offset_x = offset_x * cos_h - offset_y * sin_h
        world_offset_y = offset_x * sin_h + offset_y * cos_h
        
        # Subtract offset to get rotation center
        center_x = gps_x - world_offset_x
        center_y = gps_y - world_offset_y
        
        return (center_x, center_y)
    
    def _extract_heading(self, pose: PoseStamped) -> float:
        """Extract heading from pose quaternion."""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle