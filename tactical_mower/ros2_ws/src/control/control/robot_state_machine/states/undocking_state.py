"""UNDOCKING state implementation with integrated undocking controller logic.

Uses the same robust line-following approach as forward docking, but in reverse.
Includes path discretization, lookahead points, and zone-based corrections for
precise tracking even when wheels briefly block.
"""

import math
import time
from typing import Optional, Set, Tuple, List, TYPE_CHECKING, Callable
from dataclasses import dataclass
import logging

if TYPE_CHECKING:
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


@dataclass
class UndockingConfig:
    """Configuration parameters for undocking."""
    # GPS Sensor Offset (meters)
    gps_offset_x: float = 0.18
    gps_offset_y: float = 0.0
    
    # Speed settings
    max_speed: float = 1.0
    undock_speed_ratio: float = 0.5  # raised from 0.4 for grass traction at low battery
    
    # Undocking settings
    home_position_tolerance: float = 0.30  # Tolerance for reaching home position (30cm - robot drives closer to home)

    # Completion + fail-safe (RCA 2026-06-05). Undocking succeeds once the robot has
    # CLEARED the charge dock, not once it reaches home — a reverse drive that ends
    # off-line is still a valid undock if the contacts are clear, and the waypoint
    # follower drives to WP0 from wherever the robot ends up. undock_timeout bounds
    # the state so it can never hang silently commanding zero (the original deadlock).
    undock_clearance_distance: float = 0.5  # m from charge proving the robot left the dock
    undock_timeout: float = 30.0            # s; abort + notify operator if not cleared in time
    
    # Steering settings
    max_steering: float = 100.0
    
    # Path discretization (same as docking for consistent behavior)
    path_resolution: float = 0.10  # 10cm segments
    
    # Lookahead distance
    lookahead_distance: float = 0.20  # 30cm lookahead on line
    
    # Zone-based corrections (for robust tracking)
    critical_zone_distance: float = 0.40    # Last 50cm before home - tighter control
    final_approach_distance: float = 0.20    # Last 30cm - very precise
    
    # Undocking-specific control gains (more aggressive than docking for better correction)
    undock_steering_gain_lateral_p: float = 2.0    # Proportional gain for lateral correction
    undock_steering_gain_lateral_d: float = 0.3    # Derivative gain for lateral correction
    undock_steering_gain_heading_p: float = 2.0    # Proportional gain for heading
    undock_steering_gain_heading_d: float = 0.3    # Derivative gain for heading
    undock_max_lateral_correction_deg: float = 20.0  # Max steering from lateral error (degrees)
    undock_lateral_multiplier: float = 80.0         # Multiplier for lateral error to degrees
    
    # Zone-based lateral correction limits (reduce when close to home to avoid overshooting)
    undock_max_lateral_correction_final_deg: float = 15.0  # Reduced max in final zone


class UndockingState(BaseState):
    """UNDOCKING state: Undocking from charging station.
    
    This state implements the complete undocking procedure:
    - Move backwards along the line from charge point to home position
    - Use line-following logic similar to docking but in reverse
    - Maintain heading parallel to line (backwards direction)
    - Correct lateral errors to stay on line
    
    The undocking logic is integrated directly into this state class.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.UNDOCKING,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
        
        # Configuration (can be overridden via kwargs in on_update)
        self._config = UndockingConfig()
        
        # Internal state
        self._home_position_enu: Optional[Tuple[float, float]] = None
        self._charge_point_enu: Optional[Tuple[float, float]] = None
        self._line_direction_rad: Optional[float] = None
        
        # Discretized path (from charge to home, for reverse line following)
        self._path_points: List[Tuple[float, float]] = []
        
        # Derivative tracking for PD control
        self._last_lateral_error: Optional[float] = None
        self._last_heading_error: Optional[float] = None
        self._last_update_time: Optional[float] = None
        
        # Undocking state flag
        self._undocking_started: bool = False

        # Fail-safe bookkeeping (RCA 2026-06-05): monotonic time of state entry for
        # the undock timeout, and a latch so the failure callback fires at most once.
        self._undock_start_time: Optional[float] = None
        self._undock_failed_fired: bool = False

    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if UNDOCKING can be entered.
        
        UNDOCKING can be entered when:
        - Autonomous operation is enabled
        - Robot is docked (at charge position)
        - Route is active OR not need charge
        
        Args:
            context: Current robot context
            
        Returns:
            True if UNDOCKING can be entered
        """
        if not context.autonomous_operation_enabled:
            return False
        
        # Must be at charge position (docked) to undock
        if not context.is_at_charge_pos:
            return False
        
        # Can undock if route is active OR not need charge
        if not (context.route_active or not context.need_charge):
            return False
        
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        UNDOCKING can transition to:
        - UNDOCKED: When at (or near) home position
        - MANUAL: Always allowed
        - ERROR: Always allowed
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Manual override always allowed
        if target_state == RobotState.MANUAL:
            return True
        
        # Error state always allowed
        if target_state == RobotState.ERROR:
            return True
        
        # UNDOCKED: allowed once the robot has physically cleared the charge dock.
        # Undocking's goal is to leave the contacts so the route can start — not to
        # land precisely on home. Gating on "near home" deadlocked the machine when
        # the reverse drive ended off-line (RCA 2026-06-05): the controller declared
        # complete but this veto rejected the transition forever. Completion is now
        # decided authoritatively by the controller (_execute_undocking), which only
        # fires on_undocked after confirming clearance; this is the safety net.
        if target_state == RobotState.UNDOCKED:
            if context.is_at_charge_pos:
                self._logger.info(
                    "UNDOCKING cannot transition to UNDOCKED: still at charge position"
                )
                return False

            return True
        
        # Unknown target state
        self._logger.info(
            f"UNDOCKING cannot transition to {target_state.name}: invalid target state"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from UNDOCKING.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = {RobotState.MANUAL, RobotState.ERROR}

        # Can transition to UNDOCKED once the robot has cleared the charge dock.
        if not context.is_at_charge_pos:
            valid.add(RobotState.UNDOCKED)

        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        UNDOCKING automatically transitions to UNDOCKED when at (or near) home position
        AND not at charge position (robot has moved far enough from charge point).
        
        Args:
            context: Current robot context
            
        Returns:
            UNDOCKED if at/near home position and not at charge position, None otherwise
        """
        # Auto-transition to UNDOCKED when at (or near) home position AND not at charge position
        # Using is_near_home_pos (60cm) allows completion even with GPS drift
        # This ensures robot has moved far enough from charge point before transitioning
        if (context.is_at_home_pos or context.is_near_home_pos) and not context.is_at_charge_pos:
            return RobotState.UNDOCKED
        
        return None
    
    def on_enter(
        self, 
        previous_state: Optional[RobotState], 
        context: RobotStateContext
    ):
        """Called when entering UNDOCKING state.
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info("Entering UNDOCKING state")
        
        # Reset state tracking
        self._undocking_started = False
        self._last_lateral_error = None
        self._last_heading_error = None
        self._last_update_time = None
        self._undock_start_time = time.monotonic()
        self._undock_failed_fired = False

    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting UNDOCKING state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        self._logger.info(f"Exiting UNDOCKING state, transitioning to {next_state.name}")
        
        # Reset state
        self._undocking_started = False
        self._home_position_enu = None
        self._charge_point_enu = None
        self._line_direction_rad = None
        self._path_points.clear()
        self._undock_start_time = None
        self._undock_failed_fired = False
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        Executes the undocking procedure: move backwards along line from charge to home.
        
        Expected kwargs:
            - robot_pose: PoseStamped - Current robot pose
            - coordinate_transformer: CoordinateTransformer - For GPS/ENU conversion
            - target_gps: Optional[Tuple[float, float, float]] - Charge position (lat, lon, alt)
            - home_gps: Optional[Tuple[float, float, float]] - Home position (lat, lon, alt)
            - origin_gps: Optional[Tuple[float, float, float]] - Origin for ENU (lat, lon, alt)
            - config: Optional[UndockingConfig] - Configuration (uses defaults if not provided)
            - on_undocked: Optional[Callable] - Callback when undocking completes
        
        Args:
            context: Current robot context
            **kwargs: Additional data (robot_pose, coordinate_transformer, etc.)
            
        Returns:
            Tuple of (steering, speed) commands, or (None, None) if no commands
            are generated by this state.
        """
        # Get robot pose
        robot_pose = kwargs.get('robot_pose')
        if robot_pose is None:
            self._logger.warning("UNDOCKING: No robot_pose provided")
            return None, None
        
        # Extract position and heading from pose
        # Assume pose.position is already in ENU coordinates
        raw_x, raw_y = robot_pose.pose.position.x, robot_pose.pose.position.y
        heading = self._extract_heading(robot_pose)
        
        # Compensate GPS offset to get rotation center
        robot_x, robot_y = self._compensate_gps_offset(raw_x, raw_y, heading)
        
        on_undock_failed = kwargs.get('on_undock_failed')

        # Start undocking if not already started
        if not self._undocking_started:
            self._logger.info("UNDOCKING: Attempting to start undocking procedure...")
            if not self._start_undocking(kwargs):
                self._logger.error("UNDOCKING: Failed to start undocking procedure - aborting")
                self._fail_undock(on_undock_failed, "Start fehlgeschlagen")
                return 0.0, 0.0

        # Execute undocking
        on_undocked = kwargs.get('on_undocked')
        steering, speed = self._execute_undocking(robot_x, robot_y, heading, on_undocked, on_undock_failed)

        return steering, speed

    def _fail_undock(
        self,
        on_undock_failed: Optional[Callable[[str], None]],
        reason: str
    ) -> None:
        """Fire the undock-failure callback exactly once for this state visit.

        Routes to a safe terminal (operator notice + ERROR) in the node. Latched so
        a per-loop failure condition cannot spam the callback before the state flips.
        """
        if self._undock_failed_fired:
            return
        self._undock_failed_fired = True
        self._logger.error(f"UNDOCKING aborting: {reason}")
        if on_undock_failed is not None:
            on_undock_failed(reason)
    
    def _start_undocking(self, kwargs: dict) -> bool:
        """Start undocking procedure.
        
        Args:
            kwargs: Dictionary with target_gps, home_gps, origin_gps, coordinate_transformer
            
        Returns:
            True if undocking started successfully
        """
        target_gps = kwargs.get('target_gps')
        home_gps = kwargs.get('home_gps')
        origin_gps = kwargs.get('origin_gps')
        coord_transformer = kwargs.get('coordinate_transformer')
        
        if target_gps is None or home_gps is None or origin_gps is None:
            missing = []
            if target_gps is None:
                missing.append('target_gps')
            if home_gps is None:
                missing.append('home_gps')
            if origin_gps is None:
                missing.append('origin_gps')
            self._logger.error(
                f"UNDOCKING: Missing GPS positions: {', '.join(missing)}. "
                f"Available kwargs keys: {list(kwargs.keys())}"
            )
            return False
        
        if coord_transformer is None:
            self._logger.error("UNDOCKING: Missing coordinate_transformer")
            return False
        
        # Update config if provided
        config = kwargs.get('config')
        if config is not None:
            self._config = config
        
        try:
            # Set coordinate origin
            coord_transformer.set_origin(origin_gps[0], origin_gps[1], origin_gps[2])
            
            # Convert charge point to ENU
            charge_enu = coord_transformer.gps_to_enu(target_gps[0], target_gps[1], 0.0)
            charge_yaw_rad = math.radians(target_gps[2])
            
            # Offset charge point forward by GPS offset (same as docking)
            offset_x = self._config.gps_offset_x * math.cos(charge_yaw_rad)
            offset_y = self._config.gps_offset_x * math.sin(charge_yaw_rad)
            self._charge_point_enu = (
                charge_enu[0] - offset_x,
                charge_enu[1] - offset_y
            )
            
            # Convert home position to ENU
            home_enu = coord_transformer.gps_to_enu(home_gps[0], home_gps[1], 0.0)
            self._home_position_enu = (home_enu[0], home_enu[1])
            
            # Line direction (from charge to home - opposite of docking)
            dx = self._home_position_enu[0] - self._charge_point_enu[0]
            dy = self._home_position_enu[1] - self._charge_point_enu[1]
            self._line_direction_rad = math.atan2(dy, dx)
            
            # Discretize path (from charge to home, for reverse line following)
            self._discretize_path()
            
            # Reset derivative tracking
            self._last_lateral_error = None
            self._last_heading_error = None
            self._last_update_time = time.monotonic()
            
            # Mark as started
            self._undocking_started = True
            self._logger.info("Undocking procedure started with path discretization")
            
            return True
        except Exception as e:
            self._logger.error(f"UNDOCKING: Exception in start_undocking: {e}", exc_info=True)
            return False
    
    def _discretize_path(self) -> None:
        """Discretize the line from charge to home into segments (for reverse line following).
        
        Path points are ordered from charge (index 0) to home (last index).
        """
        if not self._home_position_enu or not self._charge_point_enu:
            return
        
        self._path_points.clear()
        
        # Line vector from charge to home (reverse direction)
        dx = self._home_position_enu[0] - self._charge_point_enu[0]
        dy = self._home_position_enu[1] - self._charge_point_enu[1]
        total_distance = math.sqrt(dx * dx + dy * dy)
        
        num_segments = max(1, int(total_distance / self._config.path_resolution))
        
        for i in range(num_segments + 1):
            t = i / num_segments
            x = self._charge_point_enu[0] + t * dx
            y = self._charge_point_enu[1] + t * dy
            self._path_points.append((x, y))
        
        # Ensure home point is the last point
        if self._path_points[-1] != self._home_position_enu:
            self._path_points.append(self._home_position_enu)
    
    def _find_closest_path_point(
        self,
        robot_x: float,
        robot_y: float
    ) -> Tuple[int, Tuple[float, float]]:
        """Find the closest point on the discretized path.
        
        Args:
            robot_x: Robot x position in ENU
            robot_y: Robot y position in ENU
            
        Returns:
            Tuple of (closest_index, closest_point)
        """
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
        """Get lookahead point on path behind current position (for reverse movement).
        
        For undocking, we look "behind" (towards home) since we're moving backwards.
        
        Args:
            current_idx: Current closest path point index
            robot_x: Robot x position in ENU
            robot_y: Robot y position in ENU
            
        Returns:
            Lookahead point (x, y)
        """
        if not self._path_points:
            return (robot_x, robot_y)
        
        # Calculate how many points to look behind (towards home)
        # Since we're moving backwards, lookahead is actually "lookbehind"
        points_behind = int(self._config.lookahead_distance / self._config.path_resolution)
        points_behind = max(1, points_behind)
        
        # Get lookbehind index (towards home, higher indices)
        # Clamp to path length
        lookbehind_idx = min(current_idx + points_behind, len(self._path_points) - 1)
        
        return self._path_points[lookbehind_idx]
    
    def _execute_undocking(
        self,
        robot_x: float,
        robot_y: float,
        heading: float,
        on_complete: Optional[Callable[[], None]],
        on_fail: Optional[Callable[[str], None]] = None
    ) -> Tuple[float, float]:
        """Execute undocking: move backwards along the line from charge to home.
        
        Uses the same robust line-following approach as forward docking:
        - Path discretization for precise tracking
        - Lookahead points for smooth trajectory
        - Zone-based corrections (critical zone, final approach)
        - PD control for heading and lateral error correction
        
        Args:
            robot_x: Robot x position (rotation center) in ENU
            robot_y: Robot y position (rotation center) in ENU
            heading: Current robot heading in radians
            on_complete: Callback when undocking is complete
            
        Returns:
            Tuple of (steering, speed) commands
        """
        if not self._home_position_enu or not self._charge_point_enu or self._line_direction_rad is None:
            self._logger.error("UNDOCKING: Missing required data (home_position, charge_point, line_direction)")
            return 0.0, 0.0
        
        if not self._path_points:
            self._logger.error("UNDOCKING: Path not discretized")
            return 0.0, 0.0

        # Fail-safe (RCA 2026-06-05): never hang in UNDOCKING. If we cannot reach a
        # terminal outcome within the timeout, abort to a safe state + notify operator.
        if self._undock_start_time is not None:
            elapsed = time.monotonic() - self._undock_start_time
            if elapsed > self._config.undock_timeout:
                self._fail_undock(on_fail, f"Zeitueberschreitung ({elapsed:.0f}s)")
                return 0.0, 0.0

        # Calculate distance to home (for zone detection and completion check)
        dx_to_home = robot_x - self._home_position_enu[0]
        dy_to_home = robot_y - self._home_position_enu[1]
        distance_to_home = math.sqrt(dx_to_home * dx_to_home + dy_to_home * dy_to_home)

        # Distance to the charge dock — the authoritative completion criterion: undock
        # is "done" only once the robot has cleared the contacts by a safe margin,
        # regardless of how close it got to home (handles off-line reverse drives).
        dx_to_charge = robot_x - self._charge_point_enu[0]
        dy_to_charge = robot_y - self._charge_point_enu[1]
        distance_to_charge = math.sqrt(dx_to_charge * dx_to_charge + dy_to_charge * dy_to_charge)
        cleared_dock = distance_to_charge >= self._config.undock_clearance_distance
        
        # Calculate distance along line from charge to robot (projection)
        dx_line = self._home_position_enu[0] - self._charge_point_enu[0]
        dy_line = self._home_position_enu[1] - self._charge_point_enu[1]
        line_length = math.sqrt(dx_line * dx_line + dy_line * dy_line)
        
        if line_length < 1e-6:
            # Invalid line, stop
            if on_complete:
                on_complete()
            self._logger.warning("UNDOCKING: Invalid line (zero length)")
            return 0.0, 0.0
        
        # Normalize line direction vector
        line_dir_x = dx_line / line_length
        line_dir_y = dy_line / line_length
        
        # Vector from charge to robot
        dx_robot = robot_x - self._charge_point_enu[0]
        dy_robot = robot_y - self._charge_point_enu[1]
        
        # Project robot position onto line (distance along line from charge)
        distance_along_line = dx_robot * line_dir_x + dy_robot * line_dir_y
        
        # Stop if:
        # 1. Robot has moved backwards past home position along the line
        # 2. OR robot is within tolerance of home position (Euclidean distance)
        # Robot should drive all the way to home position before stopping
        has_passed_home = distance_along_line >= line_length
        is_close_to_home = distance_to_home <= self._config.home_position_tolerance
        
        # Hard overshoot guard: past home by a small margin → stop now (prevents
        # runaway if home detection drifts due to pose/TF noise).
        overshoot_margin = 0.05  # 5cm safety margin, not a home tolerance change
        overshot = distance_along_line > (line_length + overshoot_margin)

        # The controller is the single authority on completion. Once it has finished
        # driving (reached home, passed home along the line, or overshot), the OUTCOME
        # is decided by whether the dock is actually clear — NOT by proximity to home.
        # An off-line reverse drive that ends >clearance from the contacts is a valid
        # undock; one that "finished" still on the contacts is a failure, not a dock.
        if overshot or has_passed_home or is_close_to_home:
            # Success if we cleared the dock OR genuinely reached the configured home
            # point (covers short undock lines where home sits inside the clearance).
            if cleared_dock or is_close_to_home:
                if on_complete:
                    on_complete()
                self._logger.info(
                    f"Undocking complete - dock cleared "
                    f"(to_charge={distance_to_charge:.2f}m, to_home={distance_to_home:.2f}m, "
                    f"along_line={distance_along_line:.2f}/{line_length:.2f}m, overshot={overshot})"
                )
            else:
                self._fail_undock(
                    on_fail,
                    f"Dock nicht verlassen (Abstand {distance_to_charge:.2f}m)"
                )
            return 0.0, 0.0
        
        # Find closest point on discretized path (for robust tracking)
        closest_idx, closest_point = self._find_closest_path_point(robot_x, robot_y)
        lookahead_point = self._get_lookahead_point(closest_idx, robot_x, robot_y)
        
        # Calculate signed lateral error (to stay on line)
        lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
        
        # Calculate heading error - robot should face backwards along line
        # Line direction is from charge to home, so backwards is opposite (180°)
        target_heading = self._normalize_angle(self._line_direction_rad + math.pi)
        line_heading_error = self._normalize_angle(target_heading - heading)
        line_heading_error_deg = math.degrees(line_heading_error)
        
        # Determine zone and select parameters (same approach as forward docking)
        in_final_zone = distance_to_home < self._config.final_approach_distance
        in_critical_zone = distance_to_home < self._config.critical_zone_distance
        
        # Select correction parameters based on zone
        if in_final_zone:
            # Final 30cm - very precise, reduced corrections to avoid overshooting
            max_lateral_correction = self._config.undock_max_lateral_correction_final_deg
            close_to_home_factor = 0.5  # Reduce corrections more aggressively
        elif in_critical_zone:
            # Critical zone 50-30cm - tighter control
            max_lateral_correction = self._config.undock_max_lateral_correction_deg * 0.8
            close_to_home_factor = 0.7
        else:
            # Normal following - full corrections
            max_lateral_correction = self._config.undock_max_lateral_correction_deg
            close_to_home_factor = 1.0
        
        # Use same line-following strategy as forward docking: Maintain heading parallel to line (backwards) and correct lateral drift
        now = time.monotonic()
        dt = 0.02
        if self._last_update_time is not None:
            dt = max(0.001, now - self._last_update_time)
        
        # Heading P term - keep parallel to line (backwards direction)
        heading_p = line_heading_error_deg * self._config.undock_steering_gain_heading_p
        
        # Heading D term
        heading_d = 0.0
        if self._last_heading_error is not None:
            heading_rate = (line_heading_error_deg - self._last_heading_error) / dt
            heading_d = heading_rate * self._config.undock_steering_gain_heading_d
        
        # Lateral correction - convert lateral error to heading adjustment
        # Use same approach as forward docking: lateral_error * gain * multiplier
        lateral_correction_deg = -lateral_error * self._config.undock_steering_gain_lateral_p * 50.0 * close_to_home_factor
        
        # Clamp lateral correction based on zone
        lateral_correction_deg = max(-max_lateral_correction, min(max_lateral_correction, lateral_correction_deg))
        
        # Lateral D term - dampen oscillation (same approach as forward docking)
        lateral_d = 0.0
        if self._last_lateral_error is not None:
            lateral_rate = (lateral_error - self._last_lateral_error) / dt
            lateral_d = -lateral_rate * self._config.undock_steering_gain_lateral_d * 50.0
            lateral_d = max(-max_lateral_correction, min(max_lateral_correction, lateral_d))
        
        # Combine: heading alignment + lateral drift correction
        p_total = heading_p + lateral_correction_deg
        d_total = heading_d + lateral_d
        
        steering = p_total + d_total
        
        # Clamp to max steering
        steering = max(-self._config.max_steering, min(self._config.max_steering, steering))
        
        # Update tracking
        self._last_lateral_error = lateral_error
        self._last_heading_error = line_heading_error_deg
        self._last_update_time = now
        
        # Move backwards (negative speed)
        speed = -self._config.undock_speed_ratio * 100.0  # Negative for backwards
        
        return steering, speed
    
    def _compute_signed_lateral_error(self, robot_x: float, robot_y: float) -> float:
        """Compute signed lateral error from robot to line.
        
        Positive = robot is LEFT of line (when facing charge point)
        Negative = robot is RIGHT of line
        
        Args:
            robot_x: Robot x position in ENU
            robot_y: Robot y position in ENU
            
        Returns:
            Signed lateral error in meters
        """
        if not self._home_position_enu or not self._charge_point_enu:
            return 0.0
        
        # Line vector (from charge to home)
        line_dx = self._home_position_enu[0] - self._charge_point_enu[0]
        line_dy = self._home_position_enu[1] - self._charge_point_enu[1]
        line_length = math.sqrt(line_dx * line_dx + line_dy * line_dy)
        
        if line_length < 1e-6:
            return 0.0
        
        # Vector from charge to robot
        robot_dx = robot_x - self._charge_point_enu[0]
        robot_dy = robot_y - self._charge_point_enu[1]
        
        # Cross product: positive if robot is left of line
        cross = line_dx * robot_dy - line_dy * robot_dx
        
        # Signed distance (positive = left, negative = right)
        signed_distance = cross / line_length
        
        return signed_distance
    
    def _compensate_gps_offset(
        self,
        gps_x: float,
        gps_y: float,
        heading: float
    ) -> Tuple[float, float]:
        """Transform GPS position to robot rotation center.
        
        Args:
            gps_x: GPS x position in ENU
            gps_y: GPS y position in ENU
            heading: Current heading in radians
            
        Returns:
            Tuple of (center_x, center_y) in ENU
        """
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
        """Extract heading from pose quaternion.
        
        Args:
            pose: Robot pose
            
        Returns:
            Heading in radians
        """
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi].
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
