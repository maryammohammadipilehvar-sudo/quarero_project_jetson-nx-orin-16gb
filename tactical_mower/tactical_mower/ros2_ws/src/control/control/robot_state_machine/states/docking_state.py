"""DOCKING state implementation with integrated docking controller logic."""

import math
import time
from enum import IntEnum
from typing import Optional, Set, Tuple, List, Callable, TYPE_CHECKING
from dataclasses import dataclass, field
import logging

if TYPE_CHECKING:
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


# Internal docking sub-states (moved from DockingController)
class DockingSubState(IntEnum):
    """Internal docking operation sub-states."""
    IDLE = 0
    APPROACH_LINE = 10      # Move to line at home position
    ALIGN_TO_LINE = 20      # Rotate to face charge point
    FOLLOW_LINE = 30        # Follow discretized line with corrections
    COMPLETED = 40


@dataclass
class DockingConfig:
    """Configuration parameters for docking."""
    # GPS Sensor Offset (meters)
    gps_offset_x: float = 0.18
    gps_offset_y: float = 0.0
    
    # Speed settings
    max_speed: float = 1.0
    docking_speed_ratio: float = 0.3
    
    # Steering settings
    max_steering: float = 100.0
    
    # Path discretization
    path_resolution: float = 0.10
    
    # Distance settings (meters)
    line_approach_tolerance: float = 0.03
    charge_position_tolerance: float = 0.05
    
    # Critical zone (last 50cm)
    critical_zone_distance: float = 0.50
    critical_zone_lateral_tolerance: float = 0.03
    
    # Final approach zone (last 30cm)
    final_approach_distance: float = 0.30
    
    # Alignment settings (degrees)
    alignment_tolerance: float = 2.0
    
    # Lookahead
    lookahead_distance: float = 0.30
    
    # Cost function weights
    weight_lateral: float = 2.0
    weight_heading: float = 1.0
    weight_lateral_critical: float = 5.0
    weight_lateral_final: float = 15.0
    
    # Steering limits
    max_lateral_correction_deg: float = 12.0
    max_lateral_correction_final_deg: float = 8.0
    
    # PD Controller gains
    steering_gain_lateral_p: float = 2.0
    steering_gain_lateral_d: float = 0.0
    steering_gain_heading_p: float = 1.5
    steering_gain_heading_d: float = 0.0
    rotation_gain_p: float = 1.5
    rotation_gain_d: float = 0.0
    
    # Minimum steering (deadband compensation)
    min_steering_command: float = 12.0
    
    # Deadlock detection
    deadlock_check_interval: float = 0.5
    deadlock_yaw_threshold: float = 0.3
    deadlock_steering_increment: float = 3.0
    deadlock_max_steering: float = 60.0


class DockingState(BaseState):
    """DOCKING state: Docking at charging station.
    
    This state implements the complete docking procedure:
    1. APPROACH_LINE: Move to line at home position
    2. ALIGN_TO_LINE: Rotate to face charge point
    3. FOLLOW_LINE: Follow discretized line with corrections
    4. DOCKED: Successfully docked
    
    The docking logic is integrated directly into this state class.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.DOCKING,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
        
        # Internal docking sub-state
        self._docking_sub_state = DockingSubState.IDLE
        
        # Target positions
        self._charge_point_enu: Optional[Tuple[float, float]] = None
        self._charge_yaw_rad: Optional[float] = None
        self._home_position_enu: Optional[Tuple[float, float]] = None
        
        # Discretized path
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
        
        # Configuration (will be set from node parameters or defaults)
        self._config = DockingConfig()
        
        # Coordinate transformer (will be passed via kwargs)
        self._coord_transformer = None
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if DOCKING can be entered.
        
        Args:
            context: Current robot context
            
        Returns:
            True if docking can be entered
        """
        # Docking can be entered when autonomous operation is enabled
        # and we're at/near home position (or explicitly triggered)
        # Use both is_at_home_pos and is_near_home_pos to handle GPS jitter
        return context.autonomous_operation_enabled and (context.is_at_home_pos or context.is_near_home_pos)
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        DOCKING can transition to:
        - DOCKED: When successfully docked
        - UNDOCKING: When undocking is initiated
        - ERROR: On error
        - MANUAL: Manual override

        
        Args:
            target_state: Target state
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        valid_targets = {
            RobotState.DOCKED,
            RobotState.UNDOCKING,
            RobotState.ERROR,
            RobotState.MANUAL
        }
        
        if target_state not in valid_targets:
            self._logger.info(
                f"DOCKING cannot transition to {target_state.name}: invalid target"
            )
            return False
        
        if target_state == RobotState.DOCKED:
            # Can transition to DOCKED when at charge position with RTK fix
            return context.rtk_fix and context.is_at_charge_pos
        
        # Other transitions are always allowed
        return True
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from DOCKING.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = {RobotState.UNDOCKING, RobotState.ERROR, RobotState.MANUAL}
        
        if context.rtk_fix and context.is_at_charge_pos:
            valid.add(RobotState.DOCKED)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        DOCKING has no automatic transitions - transitions are triggered
        by callbacks or explicit commands.
        
        Args:
            context: Current robot context
            
        Returns:
            None (no automatic transitions)
        """
        return None
    
    def on_enter(
        self, 
        previous_state: Optional[RobotState], 
        context: RobotStateContext
    ):
        """Called when entering DOCKING state.
        
        Initializes docking procedure. Expects docking parameters to be
        passed via kwargs in on_update().
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info("Entering DOCKING state")
        
        # Reset docking state
        self._docking_sub_state = DockingSubState.IDLE
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
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting DOCKING state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        self._logger.info(f"Exiting DOCKING state to {next_state.name}")
        
        # Reset docking state
        self._docking_sub_state = DockingSubState.IDLE
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        Executes docking procedure and returns steering/speed commands.
        
        Expected kwargs:
            - robot_pose: PoseStamped - Current robot pose in ENU frame
            - coordinate_transformer: CoordinateTransformerInterface - For GPS/ENU conversion
            - config: Optional[DockingConfig] - Docking configuration (uses defaults if not provided)
            - on_docked: Optional[Callable] - Callback when docking complete
            - target_gps: Optional[Tuple[float, float, float]] - Target charge position (lat, lon, yaw_deg)
            - home_gps: Optional[Tuple[float, float, float]] - Home position (lat, lon, yaw_deg)
            - origin_gps: Optional[Tuple[float, float, float]] - Origin for ENU (lat, lon, alt)
        
        Args:
            context: Current robot context
            **kwargs: Additional data (robot_pose, coordinate_transformer, etc.)
            
        Returns:
            Tuple of (steering, speed) commands, or None if no commands
        """
        # Get required parameters from kwargs
        robot_pose = kwargs.get('robot_pose')
        if robot_pose is None:
            self._logger.warning("DOCKING state: robot_pose not provided in kwargs")
            return None
        
        coord_transformer = kwargs.get('coordinate_transformer')
        if coord_transformer is None:
            self._logger.warning("DOCKING state: coordinate_transformer not provided in kwargs")
            return None
        
        # Store coordinate transformer
        self._coord_transformer = coord_transformer
        
        # Get config (use provided or defaults)
        config = kwargs.get('config')
        if config is not None:
            self._config = config
        
        # Initialize docking if not started
        if self._docking_sub_state == DockingSubState.IDLE:
            target_gps = kwargs.get('target_gps')
            home_gps = kwargs.get('home_gps')
            origin_gps = kwargs.get('origin_gps')
            
            if target_gps and home_gps and origin_gps:
                if not self._start_docking(target_gps, home_gps, origin_gps):
                    self._logger.error("Failed to start docking procedure")
                    return None
            else:
                self._logger.warning(
                    "DOCKING state: target_gps, home_gps, or origin_gps not provided"
                )
                return None
        
        # Extract position and heading from pose
        raw_x, raw_y = robot_pose.pose.position.x, robot_pose.pose.position.y
        heading = self._extract_heading(robot_pose)
        
        # Compensate GPS offset to get rotation center
        robot_x, robot_y = self._compensate_gps_offset(raw_x, raw_y, heading)
        
            # Check if docked - early check to prevent overshoot
        if self._charge_point_enu:
            dx = self._charge_point_enu[0] - robot_x
            dy = self._charge_point_enu[1] - robot_y
            distance_to_charge = math.sqrt(dx * dx + dy * dy)
            
            # Check if we've overshot (past charge point along the line)
            if self._docking_sub_state == DockingSubState.FOLLOW_LINE:
                distance_along_line = self._compute_distance_along_line(robot_x, robot_y)
                if self._home_position_enu and self._charge_point_enu:
                    line_dx = self._charge_point_enu[0] - self._home_position_enu[0]
                    line_dy = self._charge_point_enu[1] - self._home_position_enu[1]
                    line_length = math.sqrt(line_dx * line_dx + line_dy * line_dy)
                    # If we're past the charge point and very close, we've overshot
                    if distance_along_line > line_length and distance_to_charge < 0.15:
                        self._logger.warn(
                            f"Docking overshoot detected in on_update - stopping immediately "
                            f"(distance_along_line: {distance_along_line:.3f}m > line_length: {line_length:.3f}m, "
                            f"distance_to_charge: {distance_to_charge:.3f}m)"
                        )
                        self._docking_sub_state = DockingSubState.COMPLETED
                        on_docked = kwargs.get('on_docked')
                        if on_docked:
                            on_docked()
                        return 0.0, 0.0
            
        if distance_to_charge < self._config.charge_position_tolerance:
                self._docking_sub_state = DockingSubState.COMPLETED
                on_docked = kwargs.get('on_docked')
                if on_docked:
                    on_docked()
                self._logger.info(f"Docking complete - reached charge position (distance: {distance_to_charge:.3f}m)")
                return 0.0, 0.0
        
        # Execute current docking sub-state
        on_docked = kwargs.get('on_docked')
        on_home_reached = kwargs.get('on_home_reached')
        on_aligned = kwargs.get('on_aligned')
        
        if self._docking_sub_state == DockingSubState.APPROACH_LINE:
            return self._execute_approach_line(robot_x, robot_y, heading, on_home_reached)
        elif self._docking_sub_state == DockingSubState.ALIGN_TO_LINE:
            return self._execute_align_to_line(robot_x, robot_y, heading, on_aligned)
        elif self._docking_sub_state == DockingSubState.FOLLOW_LINE:
            return self._execute_follow_line(robot_x, robot_y, heading, on_docked)
        elif self._docking_sub_state == DockingSubState.COMPLETED:
            return 0.0, 0.0
        
        return None
    
    def _start_docking(
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
        if self._docking_sub_state != DockingSubState.IDLE:
            return False
        
        # Set coordinate origin
        self._coord_transformer.set_origin(origin_gps[0], origin_gps[1], origin_gps[2])
        
        # Convert charge point to ENU
        charge_enu = self._coord_transformer.gps_to_enu(target_gps[0], target_gps[1], 0.0)
        self._charge_yaw_rad = math.radians(target_gps[2])
        
        # Offset charge point forward by GPS offset
        offset_x = self._config.gps_offset_x * math.cos(self._charge_yaw_rad)
        offset_y = self._config.gps_offset_x * math.sin(self._charge_yaw_rad)
        self._charge_point_enu = (
            charge_enu[0] - offset_x,
            charge_enu[1] - offset_y
        )
        
        # Convert home position to ENU
        home_enu = self._coord_transformer.gps_to_enu(home_gps[0], home_gps[1], 0.0)
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
        
        # Start docking sub-state machine
        self._docking_sub_state = DockingSubState.APPROACH_LINE
        self._logger.info("Docking procedure started - APPROACH_LINE phase")
        
        return True
    
    def _discretize_path(self) -> None:
        """Discretize the line from home to charge into segments."""
        if not self._home_position_enu or not self._charge_point_enu:
            return
        
        self._path_points.clear()
        
        dx = self._charge_point_enu[0] - self._home_position_enu[0]
        dy = self._charge_point_enu[1] - self._home_position_enu[1]
        total_distance = math.sqrt(dx * dx + dy * dy)
        
        num_segments = max(1, int(total_distance / self._config.path_resolution))
        
        for i in range(num_segments + 1):
            t = i / num_segments
            x = self._home_position_enu[0] + t * dx
            y = self._home_position_enu[1] + t * dy
            self._path_points.append((x, y))
        
        # Ensure charge point is the last point
        if self._path_points[-1] != self._charge_point_enu:
            self._path_points.append(self._charge_point_enu)
    
    def _execute_approach_line(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]]
    ) -> Tuple[float, float]:
        """Move towards the line - drive perpendicular to reach line precisely."""
        if not self._home_position_enu or self._line_direction_rad is None:
            self._logger.error("DOCKING: Missing home position or line direction")
            return 0.0, 0.0
        
        # Calculate signed lateral error to the line
        lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
        
        # Check if we've reached the line (very close laterally)
        if abs(lateral_error) < self._config.line_approach_tolerance:
            if on_complete:
                on_complete()
            self._docking_sub_state = DockingSubState.ALIGN_TO_LINE
            self._last_lateral_error = None
            self._last_heading_error = None
            self._logger.info("Reached line - transitioning to ALIGN_TO_LINE")
            return 0.0, 0.0
        
        # TARGET: Drive perpendicular to line to reach it
        if lateral_error > 0:
            # Robot is LEFT of line, need to go RIGHT
            target_heading = self._line_direction_rad - math.pi / 2
        else:
            # Robot is RIGHT of line, need to go LEFT
            target_heading = self._line_direction_rad + math.pi / 2
        
        target_heading = self._normalize_angle(target_heading)
        heading_error = self._normalize_angle(target_heading - heading)
        heading_error_deg = math.degrees(heading_error)
        
        # If heading error is large, rotate first
        if abs(heading_error_deg) > 30.0:
            current_yaw_deg = math.degrees(heading)
            steering = self._compute_rotation_steering(heading_error_deg, current_yaw_deg)
            return steering, 0.0
        
        # Drive towards line with PD steering correction
        now = time.monotonic()
        dt = 0.02
        if self._last_update_time is not None:
            dt = max(0.001, now - self._last_update_time)
        
        # P term
        p_term = heading_error_deg * self._config.steering_gain_heading_p
        
        # D term
        d_term = 0.0
        if self._last_heading_error is not None:
            heading_rate = (heading_error_deg - self._last_heading_error) / dt
            d_term = heading_rate * self._config.steering_gain_heading_d
        
        steering = p_term + d_term
        steering = max(-self._config.max_steering, min(self._config.max_steering, steering))
        
        # Update tracking
        self._last_heading_error = heading_error_deg
        self._last_update_time = now
        
        speed = self._get_constant_speed()
        
        return steering, speed
    
    def _execute_align_to_line(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]]
    ) -> Tuple[float, float]:
        """Rotate to align with line direction (towards charge point)."""
        if self._line_direction_rad is None:
            self._logger.error("DOCKING: Missing line direction")
            return 0.0, 0.0
        
        # Calculate yaw error to line direction
        yaw_error = self._normalize_angle(self._line_direction_rad - heading)
        yaw_error_deg = math.degrees(yaw_error)
        
        # Check if aligned
        if abs(yaw_error_deg) < self._config.alignment_tolerance:
            if on_complete:
                on_complete()
            self._docking_sub_state = DockingSubState.FOLLOW_LINE
            self._last_lateral_error = None
            self._last_heading_error = None
            self._deadlock_steering_boost = 0.0
            self._logger.info("Aligned to line - transitioning to FOLLOW_LINE")
            return 0.0, 0.0
        
        # Pure rotation with PD control and deadlock detection
        current_yaw_deg = math.degrees(heading)
        steering = self._compute_rotation_steering(yaw_error_deg, current_yaw_deg)
        
        return steering, 0.0
    
    def _execute_follow_line(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]]
    ) -> Tuple[float, float]:
        """Follow the discretized line with PD control."""
        if not self._path_points or not self._charge_point_enu or self._line_direction_rad is None:
            self._logger.error("DOCKING: Missing path points or charge position")
            return 0.0, 0.0
        
        # Calculate distance to charge point
        dx = self._charge_point_enu[0] - robot_x
        dy = self._charge_point_enu[1] - robot_y
        distance_to_charge = math.sqrt(dx * dx + dy * dy)
        
        # Calculate signed lateral error
        lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
        
        # PRIMARY: Heading error relative to LINE DIRECTION
        line_heading_error = self._normalize_angle(self._line_direction_rad - heading)
        line_heading_error_deg = math.degrees(line_heading_error)
        
        # Determine zone and select parameters
        in_final_zone = distance_to_charge < self._config.final_approach_distance
        in_critical_zone = distance_to_charge < self._config.critical_zone_distance
        
        if in_final_zone:
            w_lateral = self._config.weight_lateral_final
            max_lateral_correction = self._config.max_lateral_correction_final_deg
        elif in_critical_zone:
            w_lateral = self._config.weight_lateral_critical
            max_lateral_correction = self._config.max_lateral_correction_deg
        else:
            w_lateral = self._config.weight_lateral
            max_lateral_correction = self._config.max_lateral_correction_deg
        
        # STEERING STRATEGY: Heading alignment + lateral drift correction
        now = time.monotonic()
        dt = 0.02
        if self._last_update_time is not None:
            dt = max(0.001, now - self._last_update_time)
        
        # Heading P term - keep parallel to line
        heading_p = line_heading_error_deg * self._config.steering_gain_heading_p
        
        # Heading D term
        heading_d = 0.0
        if self._last_heading_error is not None:
            heading_rate = (line_heading_error_deg - self._last_heading_error) / dt
            heading_d = heading_rate * self._config.steering_gain_heading_d
        
        # Lateral correction - convert lateral error to heading adjustment
        lateral_correction_deg = -lateral_error * self._config.steering_gain_lateral_p * 50.0
        lateral_correction_deg = max(-max_lateral_correction, min(max_lateral_correction, lateral_correction_deg))
        
        # Lateral D term - dampen oscillation
        lateral_d = 0.0
        if self._last_lateral_error is not None:
            lateral_rate = (lateral_error - self._last_lateral_error) / dt
            lateral_d = -lateral_rate * self._config.steering_gain_lateral_d * 50.0
            lateral_d = max(-max_lateral_correction, min(max_lateral_correction, lateral_d))
        
        # Combine: heading alignment + lateral drift correction
        p_total = heading_p + lateral_correction_deg
        d_total = heading_d + lateral_d
        
        steering = p_total + d_total
        steering = max(-self._config.max_steering, min(self._config.max_steering, steering))
        
        # Update tracking
        self._last_lateral_error = lateral_error
        self._last_heading_error = line_heading_error_deg
        self._last_update_time = now
        
        # Constant speed (no ramping - parameters are tuned for this)
        speed = self._get_constant_speed()
        
        # Check if docked - check BEFORE returning commands to prevent overshoot
        if distance_to_charge < self._config.charge_position_tolerance:
            self._docking_sub_state = DockingSubState.COMPLETED
            if on_complete:
                on_complete()
            self._logger.info(f"Docking complete - reached charge position (distance: {distance_to_charge:.3f}m)")
            return 0.0, 0.0
        
        # Additional safety check: if we're very close and moving forward, check for overshoot
        # Check if we're past the charge point (overshooting)
        # Hard overshoot stop: if we're past the charge point along the line, stop immediately
        # This prevents runaway if we missed the stop window (e.g., GPS noise or inertia).
        distance_along_line = self._compute_distance_along_line(robot_x, robot_y)
        if self._home_position_enu and self._charge_point_enu:
            line_dx = self._charge_point_enu[0] - self._home_position_enu[0]
            line_dy = self._charge_point_enu[1] - self._home_position_enu[1]
            line_length = math.sqrt(line_dx * line_dx + line_dy * line_dy)
            overshoot_margin = 0.05  # 5cm safety margin, not a docking tolerance change
            if distance_along_line > (line_length + overshoot_margin):
                self._logger.warn(
                    "Docking overshoot detected - stopping immediately "
                    f"(distance_along_line: {distance_along_line:.3f}m, line_length: {line_length:.3f}m)"
                )
                self._docking_sub_state = DockingSubState.COMPLETED
                if on_complete:
                    on_complete()
                return 0.0, 0.0
        
        return steering, speed
    
    def _compute_signed_lateral_error(self, robot_x: float, robot_y: float) -> float:
        """Compute signed lateral error from robot to line.
        
        Positive = robot is LEFT of line (when facing charge point)
        Negative = robot is RIGHT of line
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
        points_ahead = int(self._config.lookahead_distance / self._config.path_resolution)
        points_ahead = max(1, points_ahead)
        
        # Get lookahead index, clamped to path length
        lookahead_idx = min(current_idx + points_ahead, len(self._path_points) - 1)
        
        return self._path_points[lookahead_idx]
    
    def _compute_rotation_steering(self, error_deg: float, current_yaw_deg: float) -> float:
        """Compute steering for pure rotation with PD control and deadlock detection."""
        # If error is within tolerance, no steering needed
        if abs(error_deg) < self._config.alignment_tolerance:
            self._deadlock_steering_boost = 0.0
            return 0.0
        
        now = time.monotonic()
        
        # Deadlock detection: check if yaw is changing
        if self._deadlock_check_time is None:
            self._deadlock_check_time = now
            self._deadlock_check_yaw = current_yaw_deg
        elif (now - self._deadlock_check_time) >= self._config.deadlock_check_interval:
            if self._deadlock_check_yaw is not None:
                yaw_change = abs(current_yaw_deg - self._deadlock_check_yaw)
                
                if yaw_change < self._config.deadlock_yaw_threshold:
                    # Robot is stuck - increase steering
                    self._deadlock_steering_boost = min(
                        self._deadlock_steering_boost + self._config.deadlock_steering_increment,
                        self._config.deadlock_max_steering - self._config.min_steering_command
                    )
                    self._logger.info(
                        f"Deadlock detected: yaw_change={yaw_change:.2f}°, boost={self._deadlock_steering_boost:.1f}"
                    )
                else:
                    # Robot is moving - reduce boost gradually
                    self._deadlock_steering_boost = max(0.0, self._deadlock_steering_boost - self._config.deadlock_steering_increment)
            
            # Reset check
            self._deadlock_check_time = now
            self._deadlock_check_yaw = current_yaw_deg
        
        # Calculate derivative
        d_term = 0.0
        if self._last_heading_error is not None and self._last_update_time is not None:
            dt = now - self._last_update_time
            if dt > 0.001:
                error_rate = (error_deg - self._last_heading_error) / dt
                d_term = error_rate * self._config.rotation_gain_d
        
        # P term
        p_term = error_deg * self._config.rotation_gain_p
        
        # Combined PD
        steering = p_term + d_term
        
        # Apply minimum steering ONLY for large rotations (>10°)
        if abs(error_deg) > 10.0:
            if abs(steering) < self._config.min_steering_command:
                steering = math.copysign(self._config.min_steering_command, steering)
        
        # Add deadlock boost (same sign as steering)
        steering = steering + math.copysign(self._deadlock_steering_boost, steering)
        
        # Clamp to max
        steering = max(-self._config.max_steering, min(self._config.max_steering, steering))
        
        # Update tracking
        self._last_heading_error = error_deg
        self._last_update_time = now
        
        return steering
    
    def _get_constant_speed(self) -> float:
        """Get constant docking speed (30% of max)."""
        return self._config.docking_speed_ratio * 100.0  # As percentage
    
    def _compensate_gps_offset(
        self,
        gps_x: float,
        gps_y: float,
        heading: float
    ) -> Tuple[float, float]:
        """Transform GPS position to robot rotation center."""
        offset_x = self._config.gps_offset_x
        offset_y = self._config.gps_offset_y
        
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        
        # GPS offset in world frame
        world_offset_x = offset_x * cos_h - offset_y * sin_h
        world_offset_y = offset_x * sin_h + offset_y * cos_h
        
        # Subtract offset to get rotation center
        center_x = gps_x - world_offset_x
        center_y = gps_y - world_offset_y
        
        return (center_x, center_y)
    
    def _extract_heading(self, pose: 'PoseStamped') -> float:
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
