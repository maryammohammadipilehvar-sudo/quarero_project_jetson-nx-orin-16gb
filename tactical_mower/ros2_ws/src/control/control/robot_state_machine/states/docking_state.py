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
    VISUAL_SERVO = 35       # ArUco-marker visual servo for the final precise approach
    COMPLETED = 40
    FAILED = 41             # Could not reach charge point within lateral tolerance


@dataclass
class DockingConfig:
    """Configuration parameters for docking."""
    # GPS Sensor Offset (meters)
    gps_offset_x: float = 0.18
    gps_offset_y: float = 0.0
    
    # Speed settings
    max_speed: float = 1.0
    docking_speed_ratio: float = 0.5  # raised from 0.3 for grass traction at low battery
    approach_speed_ratio: float = 0.7  # raised from 0.3 for grass traction at low battery
    
    # Steering settings
    max_steering: float = 100.0
    
    # Path discretization
    path_resolution: float = 0.10
    
    # Distance settings (meters)
    line_approach_tolerance: float = 0.03
    # Longitudinal "docked" acceptance. With no contact sensor, this is set from the
    # measured residual where the robot physically comes to rest against the dock:
    # it parks a consistent 10-15 cm short of the saved charge GPS (final creep can't
    # push past the dock). 0.18 covers the observed ~14.6 cm rest point with margin.
    # Lateral stays tight (charge_lateral_tolerance) so a robot stopped *beside* the
    # plates is still rejected. Tune on-robot with pose_diff_measure.py.
    charge_position_tolerance: float = 0.18
    
    # Critical zone (last 50cm)
    critical_zone_distance: float = 0.50
    critical_zone_lateral_tolerance: float = 0.03
    
    # Final approach zone (last 30cm)
    final_approach_distance: float = 0.30

    # Creep zone (last 15cm). creep_speed_ratio must stay above the drivetrain
    # stiction floor: speed% maps to duty ≈ speed%·0.286, and below ~10% duty the
    # robot does not move on grass — it stalled at 0.12 (≈3.4% duty), parking 10-15 cm
    # short. 0.35 (≈10% duty) lets the creep actually close the gap and seat into the
    # dock. On-robot tuning knob: raise if it still stalls, lower if it overshoots.
    creep_zone_distance: float = 0.15
    creep_speed_ratio: float = 0.35

    # Consecutive docking confirmation (filter GPS noise)
    docked_confirm_readings: int = 3

    # GPS-only-mode "grossly off the line" sanity gate. With no contact sensor we must
    # not declare docked if the robot finished well beside the plates — but the GPS path
    # cannot hit a tight (3 cm) lateral on grass, so a tight gate chronically false-FAILs
    # a perfectly good dock. This is therefore a GENEROUS sanity bound: complete the GPS
    # dock at charge distance, and only FAIL if the robot is grossly sideways (something
    # went wrong upstream). Tight cm-precision is the ArUco visual servo's job, not GPS's.
    charge_lateral_tolerance: float = 0.15

    # Safety: abort FOLLOW_LINE if travel exceeds this multiple of line length
    max_travel_ratio: float = 2.0

    # Alignment settings (degrees)
    # Widened from 2.0: a 2° in-place-pivot target on grass with a heavy diff-drive
    # caused hunting/never-settling (worst-traction maneuver + undamped rotation).
    # FOLLOW_LINE corrects residual heading continuously while driving, so ALIGN only
    # needs to get roughly onto the line heading.
    alignment_tolerance: float = 6.0
    # Fail-safe: if ALIGN can't settle within this time (poor grass traction), hand
    # off to FOLLOW_LINE instead of pivoting forever. FOLLOW drives the line with
    # heading PD, finishing the alignment dynamically (with traction). This is the
    # direct fix for the observed "spins in place, never docks" failure.
    align_timeout: float = 12.0
    
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
    
    # PD Controller gains. The D terms damp the approach so the line-follower
    # settles instead of weaving/overshooting (they were previously 0.0 = pure-P).
    steering_gain_lateral_p: float = 2.0
    steering_gain_lateral_d: float = 0.5
    steering_gain_heading_p: float = 1.5
    steering_gain_heading_d: float = 0.3
    rotation_gain_p: float = 1.5
    # Damping for in-place rotation. Was 0.0 (pure-P) → overshoot/hunting around the
    # align target with nothing to settle it. Non-zero D term damps the approach.
    rotation_gain_d: float = 0.4
    
    # Minimum steering (deadband compensation)
    min_steering_command: float = 12.0
    
    # Deadlock detection
    deadlock_check_interval: float = 0.5
    deadlock_yaw_threshold: float = 0.3
    deadlock_steering_increment: float = 3.0
    deadlock_max_steering: float = 60.0


@dataclass
class ArucoDockConfig:
    """ArUco-marker precision-docking config.

    Optional final-approach refinement layered on top of the GPS line-follow.
    Defaults keep it OFF (enabled=False, setpoint_valid=False) so docking is
    byte-for-byte the existing GPS behaviour until an operator explicitly
    enables it and teaches a setpoint (see docking_aruco/). All fields are
    populated at runtime from /routen/settings/aruco_dock.yaml.
    """
    enabled: bool = False
    # Engage the visual servo only inside this range (m) to the GPS charge point,
    # so the bulk of the approach stays on the proven GPS line-follow.
    engage_distance: float = 1.5
    # Marker pose older than this (s) → treat as lost; revert to GPS FOLLOW_LINE.
    stale_timeout: float = 1.0
    # Teach-and-repeat setpoint: marker pose (camera optical frame) recorded when
    # the robot is perfectly docked. lateral = x (m), range = z (m), yaw = marker
    # bearing (rad, normal in the camera XZ plane). setpoint_valid gates engagement.
    setpoint_lateral: float = 0.0
    setpoint_range: float = 0.0
    setpoint_yaw: float = 0.0
    setpoint_valid: bool = False
    # Control gains (steering is the same -100..100 convention as the line-follower).
    gain_lateral: float = 120.0   # DEPRECATED (metres-based; over-reacted at range)
    gain_yaw: float = 40.0        # steering per radian of bearing error (taught mode)
    invert_steering: bool = False  # flip if bench test shows it steers away from centre
    # NORMAL-ALIGNED control (line-follower in the dock frame): align to the dock normal
    # (heading) while steering onto the dock centre-line (cross-track). Forward motion
    # removes the lateral offset; cross is a bounded metre offset so this stays stable to
    # the dock (unlike bearing-centring, which blows up at close range and spins).
    gain_heading_deg: float = 1.5        # steering per degree of heading-vs-normal error
    gain_heading_d: float = 0.2          # damping on heading rate (deg/s)
    gain_cross: float = 45.0             # steering per metre of cross-track offset
    creep_heading_gate_deg: float = 30.0 # creep forward while heading within this of the normal
    dock_heading_tolerance_deg: float = 6.0  # docked only when squared to the normal within this
    # ANTI-WALL SAFETY (RCA 2026-06-06 wall-crash). NEVER creep toward the standoff while
    # off-axis — you'd drive into the wall BESIDE the markers. Only creep when cross is
    # within this; if it can't get on-axis it stops and times out → GPS fallback (which
    # stops at its own charge point), never a crash.
    align_cross_tol: float = 0.20        # m: max cross-track offset to allow forward creep
    # Overshoot guard: the ~13 Hz marker makes creep near-continuous; it overshot the stop
    # by ~11 cm into the wall. Stop creeping this far BEFORE the standoff so momentum
    # coasts to it instead of past it.
    creep_stop_margin: float = 0.12      # m
    # Deprecated bearing-centring gains (kept so older yamls don't error; unused).
    gain_bearing: float = 3.0
    gain_bearing_d: float = 0.4
    creep_bearing_gate_deg: float = 7.0
    # Operator request 2026-06-05: keep the marker in the MIDDLE of the camera (lateral
    # target = optical centre, 0) and drive that centred line to the charge point,
    # instead of servoing to the taught setpoint_lateral. Pure-centring also drops the
    # bearing term so the single steering DOF is spent only on centring the marker.
    center_marker: bool = True
    # Markerless-reference mode (operator request 2026-06-05): use the marker ONLY as a
    # live centring reference (no taught pose at all) and let the SAVED GPS charge point
    # own the stop. When False, ArUco engages on a fresh marker alone — no setpoint_* is
    # required or used, and the marker is always centred. When True, the legacy
    # teach-and-repeat behaviour (servo to the taught setpoint) is used.
    use_taught_setpoint: bool = False
    # Target standoff (m) for MARKERLESS mode: the camera→marker range at which the
    # robot is docked. Operator measures this once at the perfectly-mated position. In
    # taught mode setpoint_range plays this role instead. The MARKER — not GPS — owns the
    # longitudinal stop and the "docked" decision during VISUAL_SERVO. This removes the
    # GPS-vs-marker dual-authority that caused the VISUAL_SERVO⇄FOLLOW_LINE livelock
    # (the marker range floor and the GPS charge point fighting over the stop).
    stop_range_m: float = 0.85
    # DEPRECATED: GPS no longer owns the stop in marker mode (kept so older yamls that
    # still set it don't error). The marker range owns the stop now; see stop_range_m.
    gps_stop_tolerance: float = 0.12
    # Anti-ping-pong: a brief marker dropout mid-servo HOLDS position (steer/creep paused)
    # for up to this long, waiting to re-acquire, instead of instantly reverting to GPS.
    reacquire_grace: float = 2.0
    # After a real revert to GPS FOLLOW_LINE (lost beyond grace, or servo timeout), block
    # re-engaging the servo for this long so the two controllers cannot oscillate — GPS
    # drives the last stretch to its own charge-point stop.
    reengage_cooldown: float = 6.0
    creep_speed_ratio: float = 0.35  # forward creep while closing range (% of max speed)
    max_steering: float = 60.0    # gentler than GPS max — we are cm from the dock
    # Docked acceptance (tight — precision is the whole point of using vision).
    dock_lateral_tolerance: float = 0.02   # 2 cm vs taught lateral
    dock_range_tolerance: float = 0.03     # 3 cm vs taught range
    dock_confirm_readings: int = 3
    # Give up the visual servo after this long without docking → hand back to GPS.
    servo_timeout: float = 25.0

    # Anti-collision safety (RCA 2026-06-05 wall-crash). The marker is mounted ON the
    # wall, so its range IS the distance to the wall. These bound the final creep
    # INDEPENDENTLY of the taught setpoint, so a bad setpoint or a pose glitch can
    # never drive the robot into the wall. min_range_m is an absolute physical floor
    # the operator should set just below the true docked standoff; range_floor_margin
    # also forbids creeping more than that past the taught setpoint range.
    # Hard anti-wall abort floor (m): if the marker range ever falls below this the dock
    # is aborted (never push into the wall). MUST be set below stop_range_m / the taught
    # standoff. Independent of the stop logic — the last-resort backstop.
    min_range_m: float = 0.70         # never let the marker range fall below this (m)
    range_floor_margin: float = 0.05  # also never below setpoint_range minus this (m)
    max_range_jump: float = 0.3       # ignore (don't creep on) a frame whose range jumps > this (m)


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

        # Consecutive docking confirmation counter
        self._docked_confirm_counter: int = 0

        # ALIGN_TO_LINE fail-safe timeout start (monotonic); None = not aligning.
        self._align_start_time: Optional[float] = None

        # Safety: track FOLLOW_LINE entry position for travel distance check
        self._follow_line_start_pos: Optional[Tuple[float, float]] = None
        self._follow_line_log_counter: int = 0

        # Failure callback (set from kwargs each update); fired when docking gives up
        self._on_dock_failed: Optional[Callable[[str], None]] = None

        # ArUco precision-docking (optional, off by default). Config + latest marker
        # pose are pushed in via kwargs each update; see ArucoDockConfig.
        self._aruco = ArucoDockConfig()
        self._aruco_marker: Optional[Tuple[float, float, float, float]] = None  # (lateral, range, yaw, age_s)
        self._visual_servo_start_time: Optional[float] = None
        self._visual_dock_confirm_counter: int = 0
        self._last_servo_rng: Optional[float] = None  # previous marker range, for jump rejection
        self._last_servo_bearing_deg: Optional[float] = None  # previous bearing, for servo damping
        self._last_processed_marker: Optional[Tuple[float, float]] = None  # (lat,rng) of last NEW frame acted on
        self._last_servo_proc_time: Optional[float] = None  # monotonic time of last processed frame (for dt)
        self._marker_lost_since: Optional[float] = None  # monotonic; servo hold-to-reacquire
        self._reengage_block_until: float = 0.0  # monotonic; blocks FOLLOW_LINE→VISUAL re-entry

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
        self._docked_confirm_counter = 0
        self._align_start_time = None
        self._follow_line_start_pos = None
        self._follow_line_log_counter = 0
        self._visual_servo_start_time = None
        self._visual_dock_confirm_counter = 0
        self._last_servo_rng = None
        self._marker_lost_since = None
        self._reengage_block_until = 0.0

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
        
        # Callbacks (fetched once per update so the recovery helper can reach them)
        on_docked = kwargs.get('on_docked')
        on_home_reached = kwargs.get('on_home_reached')
        on_aligned = kwargs.get('on_aligned')
        self._on_dock_failed = kwargs.get('on_dock_failed')

        # ArUco precision-docking inputs (optional). Both default to keeping it off.
        aruco_config = kwargs.get('aruco_config')
        if aruco_config is not None:
            for key, value in aruco_config.items():
                if hasattr(self._aruco, key):
                    setattr(self._aruco, key, value)
        self._aruco_marker = kwargs.get('aruco_marker')

        # GPS-mode docked check — runs ONLY in FOLLOW_LINE (the GPS line-follow).
        # VISUAL_SERVO owns its own stop + "docked" decision from the MARKER range, so
        # GPS and the marker never fight over the stop (that dual-authority was the
        # VISUAL_SERVO⇄FOLLOW_LINE livelock — RCA 2026-06-06). Authority is staged:
        # GPS coarse-approaches and is the fallback; the marker owns the precise finish.
        #
        # No contact sensor → never fake a dock. The line-follower's PD keeps lateral
        # small on the way in; here we complete at charge distance (or on a committed
        # overshoot) and only FAIL if the robot finished GROSSLY sideways
        # (charge_lateral_tolerance, generous — GPS can't hit cm precision on grass;
        # that's the marker's job).
        if self._charge_point_enu and self._docking_sub_state == DockingSubState.FOLLOW_LINE:
            dx = self._charge_point_enu[0] - robot_x
            dy = self._charge_point_enu[1] - robot_y
            distance_to_charge = math.sqrt(dx * dx + dy * dy)
            lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
            lateral_tolerance = self._config.charge_lateral_tolerance
            charge_tolerance = self._config.charge_position_tolerance

            # Overshoot: past the charge point along the line (GPS noise / inertia).
            past_charge = False
            if self._home_position_enu:
                distance_along_line = self._compute_distance_along_line(robot_x, robot_y)
                line_dx = self._charge_point_enu[0] - self._home_position_enu[0]
                line_dy = self._charge_point_enu[1] - self._home_position_enu[1]
                line_length = math.sqrt(line_dx * line_dx + line_dy * line_dy)
                past_charge = distance_along_line > line_length

            if distance_to_charge < charge_tolerance or past_charge:
                if abs(lateral_error) <= lateral_tolerance:
                    self._docked_confirm_counter += 1
                    # Overshoot is committed (we can't reverse here), so accept the
                    # moment we're on-target; otherwise require N consecutive readings.
                    if (self._docked_confirm_counter >= self._config.docked_confirm_readings
                            or past_charge):
                        self._docking_sub_state = DockingSubState.COMPLETED
                        if on_docked:
                            on_docked()
                        self._logger.info(
                            f"GPS docking complete at charge position "
                            f"(distance: {distance_to_charge:.3f}m, "
                            f"lateral: {lateral_error*100:.1f}cm)"
                        )
                        return 0.0, 0.0
                    self._logger.info(
                        f"Docking proximity {self._docked_confirm_counter}/"
                        f"{self._config.docked_confirm_readings} "
                        f"(distance: {distance_to_charge:.3f}m, lateral: {lateral_error*100:.1f}cm)"
                    )
                else:
                    # At charge distance but grossly sideways: don't fake a dock and
                    # don't loop re-approaches. Stop once and report failure.
                    self._docked_confirm_counter = 0
                    self._logger.error(
                        f"Docking stopped off-target: lateral {lateral_error*100:.1f}cm "
                        f"(> {lateral_tolerance*100:.0f}cm) at "
                        f"{distance_to_charge:.3f}m. Not docked (no false success)."
                    )
                    self._docking_sub_state = DockingSubState.FAILED
                    if self._on_dock_failed:
                        self._on_dock_failed(
                            f"off-target lateral {lateral_error*100:.1f}cm"
                        )
                    return 0.0, 0.0
            else:
                self._docked_confirm_counter = 0

        # Execute current docking sub-state
        if self._docking_sub_state == DockingSubState.APPROACH_LINE:
            return self._execute_approach_line(robot_x, robot_y, heading, on_home_reached)
        elif self._docking_sub_state == DockingSubState.ALIGN_TO_LINE:
            return self._execute_align_to_line(robot_x, robot_y, heading, on_aligned)
        elif self._docking_sub_state == DockingSubState.FOLLOW_LINE:
            return self._execute_follow_line(robot_x, robot_y, heading, on_docked)
        elif self._docking_sub_state == DockingSubState.VISUAL_SERVO:
            return self._execute_visual_servo(robot_x, robot_y, on_docked)
        elif self._docking_sub_state == DockingSubState.COMPLETED:
            return 0.0, 0.0
        elif self._docking_sub_state == DockingSubState.FAILED:
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
        self._follow_line_start_pos = None
        self._follow_line_log_counter = 0

        ldx = self._charge_point_enu[0] - self._home_position_enu[0]
        ldy = self._charge_point_enu[1] - self._home_position_enu[1]
        ll = math.sqrt(ldx * ldx + ldy * ldy)
        self._logger.info(
            f"Docking started: home_enu=({self._home_position_enu[0]:.3f},{self._home_position_enu[1]:.3f}), "
            f"charge_enu=({self._charge_point_enu[0]:.3f},{self._charge_point_enu[1]:.3f}), "
            f"line_length={ll:.3f}m, line_dir={math.degrees(self._line_direction_rad):.1f}°, "
            f"origin=({origin_gps[0]:.8f},{origin_gps[1]:.8f})"
        )
        
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
            self._align_start_time = time.monotonic()
            self._deadlock_check_time = None
            self._deadlock_steering_boost = 0.0
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

        # Fail-safe: never pivot in place forever. On poor traction the in-place turn
        # can hunt/stall; after align_timeout, hand off to FOLLOW_LINE, which drives
        # toward the charge point while correcting heading (with traction). This is the
        # direct guard against the observed "spins in place, never docks" failure.
        if (self._align_start_time is not None
                and (time.monotonic() - self._align_start_time) > self._config.align_timeout):
            self._logger.warn(
                f"ALIGN timeout (> {self._config.align_timeout:.0f}s) — handing off to "
                f"FOLLOW_LINE; heading PD finishes alignment while driving"
            )
            self._docking_sub_state = DockingSubState.FOLLOW_LINE
            self._last_lateral_error = None
            self._last_heading_error = None
            self._deadlock_steering_boost = 0.0
            self._align_start_time = None
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
            self._align_start_time = None
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

        # Track entry position for travel-distance safety check
        if self._follow_line_start_pos is None:
            self._follow_line_start_pos = (robot_x, robot_y)
            self._logger.info(
                f"FOLLOW_LINE started: robot=({robot_x:.3f},{robot_y:.3f}), "
                f"charge_enu=({self._charge_point_enu[0]:.3f},{self._charge_point_enu[1]:.3f}), "
                f"home_enu=({self._home_position_enu[0]:.3f},{self._home_position_enu[1]:.3f})"
            )

        # Calculate distance to charge point
        dx = self._charge_point_enu[0] - robot_x
        dy = self._charge_point_enu[1] - robot_y
        distance_to_charge = math.sqrt(dx * dx + dy * dy)

        # Safety: abort if robot has traveled too far (coordinate mismatch protection)
        sdx = robot_x - self._follow_line_start_pos[0]
        sdy = robot_y - self._follow_line_start_pos[1]
        travel_distance = math.sqrt(sdx * sdx + sdy * sdy)
        line_length = math.sqrt(
            (self._charge_point_enu[0] - self._home_position_enu[0]) ** 2
            + (self._charge_point_enu[1] - self._home_position_enu[1]) ** 2
        )
        max_travel = max(line_length * self._config.max_travel_ratio, 1.0)
        if travel_distance > max_travel:
            self._logger.error(
                f"DOCKING ABORT: traveled {travel_distance:.2f}m > "
                f"limit {max_travel:.2f}m (line={line_length:.2f}m). "
                f"Possible coordinate mismatch. Stopping."
            )
            self._docking_sub_state = DockingSubState.COMPLETED
            if on_complete:
                on_complete()
            return 0.0, 0.0

        # Periodic diagnostic log (every 10 iterations = 1 s)
        self._follow_line_log_counter += 1
        if self._follow_line_log_counter % 10 == 0:
            self._logger.info(
                f"FOLLOW_LINE: dist_charge={distance_to_charge:.3f}m, "
                f"traveled={travel_distance:.2f}m/{max_travel:.2f}m, "
                f"robot=({robot_x:.3f},{robot_y:.3f}), heading={math.degrees(heading):.1f}°"
            )

        # Hand off to the ArUco visual servo for the final, precise approach when
        # enabled, the marker is fresh, and we are close enough. In markerless mode no
        # taught setpoint is needed (the marker is just a centring reference); in legacy
        # mode a taught setpoint must be valid. If any precondition is false we stay on
        # the GPS line-follow, so nothing changes when ArUco docking is off. The
        # reengage cooldown blocks immediate re-entry right after a servo→GPS revert so
        # the two controllers can't oscillate.
        setpoint_ready = (not self._aruco.use_taught_setpoint) or self._aruco.setpoint_valid
        # [DOCKDIAG] TEMP: log every engage-gate input while within engage range so a
        # missed/late VISUAL_SERVO engage is explained in the data, not guessed. Remove
        # once docking is validated.
        if distance_to_charge <= self._aruco.engage_distance and (self._follow_line_log_counter % 3 == 0):
            _m = self._aruco_marker
            self._logger.info(
                f"[DOCKDIAG] engage-gate: enabled={self._aruco.enabled} "
                f"dist={distance_to_charge:.2f}<=engage{self._aruco.engage_distance} "
                f"fresh={self._marker_is_fresh()} "
                f"marker={'None' if _m is None else f'x={_m[0]:.3f} z={_m[1]:.3f} age={_m[3]:.2f}'} "
                f"setpoint_ready={setpoint_ready} "
                f"cooldown_ok={time.monotonic() >= self._reengage_block_until}"
            )
        if (self._aruco.enabled and setpoint_ready
                and distance_to_charge <= self._aruco.engage_distance
                and self._marker_is_fresh()
                and time.monotonic() >= self._reengage_block_until):
            self._logger.info(
                f"FOLLOW_LINE → VISUAL_SERVO (dist_charge={distance_to_charge:.2f}m, "
                f"marker fresh)"
            )
            self._docking_sub_state = DockingSubState.VISUAL_SERVO
            self._visual_servo_start_time = time.monotonic()
            self._visual_dock_confirm_counter = 0
            self._marker_lost_since = None
            self._last_servo_rng = None
            self._last_servo_bearing_deg = None
            self._last_processed_marker = None
            self._last_servo_proc_time = None
            return 0.0, 0.0

        # Calculate signed lateral error
        lateral_error = self._compute_signed_lateral_error(robot_x, robot_y)
        
        # PRIMARY: Heading error relative to LINE DIRECTION
        line_heading_error = self._normalize_angle(self._line_direction_rad - heading)
        line_heading_error_deg = math.degrees(line_heading_error)
        
        # Determine zone and select parameters
        in_creep_zone = distance_to_charge < self._config.creep_zone_distance
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
        
        # Distance-based speed: approach → critical ramp → final → creep
        if in_creep_zone:
            speed = self._config.creep_speed_ratio * 100.0
        elif in_final_zone:
            ramp_range = self._config.final_approach_distance - self._config.creep_zone_distance
            if ramp_range > 0:
                t = (distance_to_charge - self._config.creep_zone_distance) / ramp_range
                ratio = self._config.creep_speed_ratio + t * (self._config.docking_speed_ratio - self._config.creep_speed_ratio)
            else:
                ratio = self._config.creep_speed_ratio
            speed = ratio * 100.0
        elif in_critical_zone:
            ramp_range = self._config.critical_zone_distance - self._config.final_approach_distance
            if ramp_range > 0:
                t = (distance_to_charge - self._config.final_approach_distance) / ramp_range
                ratio = self._config.docking_speed_ratio + t * (self._config.approach_speed_ratio - self._config.docking_speed_ratio)
            else:
                ratio = self._config.docking_speed_ratio
            speed = ratio * 100.0
        else:
            speed = self._config.approach_speed_ratio * 100.0
        
        # The "docked" decision (distance AND lateral within tolerance, plus the
        # past-charge overshoot stop) is owned by the early-check at the top of
        # on_update, so FOLLOW_LINE just drives the line with PD here.
        return steering, speed

    def _marker_is_fresh(self) -> bool:
        """True if a dock-marker pose was received within stale_timeout."""
        return (self._aruco_marker is not None
                and self._aruco_marker[3] <= self._aruco.stale_timeout)

    def _execute_visual_servo(
        self,
        robot_x: float,
        robot_y: float,
        on_complete: Optional[Callable[[], None]],
    ) -> Tuple[float, float]:
        """Final precise approach servoing on the ArUco dock marker.

        Authority model (RCA 2026-06-06): once engaged, the MARKER owns BOTH the
        steering (centre the marker) AND the longitudinal stop + "docked" decision (via
        the camera→marker range vs the target standoff). GPS does NOT participate here —
        that removes the GPS-vs-marker dual-authority that previously made the robot
        ping-pong VISUAL_SERVO⇄FOLLOW_LINE and never seat. GPS already did its job
        (coarse approach + engage gate) and remains only as the fallback if the marker
        is lost.

        Target standoff = setpoint_range (taught mode) or stop_range_m (markerless).
        Docked = marker centred within dock_lateral_tolerance AND range within
        dock_range_tolerance of the standoff, for dock_confirm_readings consecutive
        FRESH frames → on_complete() (same DOCKED transition the GPS path uses).

        Fail-soft, in priority order:
          - hard min-range floor breached → FAILED (anti-wall; never push into the wall),
          - range-jump frame → steer but hold forward motion (ArUco close-range glitch),
          - marker briefly lost → HOLD position up to reacquire_grace, then revert to GPS
            FOLLOW_LINE with a reengage cooldown (no instant bounce-back),
          - servo_timeout → revert to GPS FOLLOW_LINE (with cooldown).
        Creep only ever happens on a FRESH, trustworthy frame above the floor.
        """
        now = time.monotonic()

        # Safety: never servo forever → hand back to GPS line-follow (with cooldown so
        # FOLLOW_LINE doesn't immediately re-engage and oscillate).
        if (self._visual_servo_start_time is not None
                and (now - self._visual_servo_start_time) > self._aruco.servo_timeout):
            self._logger.warn(
                f"VISUAL_SERVO timeout (> {self._aruco.servo_timeout:.0f}s) → "
                f"reverting to FOLLOW_LINE (GPS finishes)"
            )
            self._revert_to_follow_line(now)
            return 0.0, 0.0

        # Marker briefly lost → HOLD (do not coast toward the wall, do not instantly
        # revert). Wait up to reacquire_grace for it to come back; only then fall back to
        # GPS. This hold-to-reacquire is what kills the old ping-pong.
        if not self._marker_is_fresh():
            if self._marker_lost_since is None:
                self._marker_lost_since = now
                self._logger.info("VISUAL_SERVO: marker lost → holding for re-acquire")
            if (now - self._marker_lost_since) > self._aruco.reacquire_grace:
                self._logger.warn(
                    "VISUAL_SERVO: marker not re-acquired within grace → "
                    "reverting to FOLLOW_LINE (GPS finishes)"
                )
                self._revert_to_follow_line(now)
            return 0.0, 0.0   # hold position while waiting
        # Fresh again — clear the lost timer.
        self._marker_lost_since = None

        # NORMAL-ALIGNED control. Instead of just centring the marker (which only nulls
        # the bearing and lets a diff-drive spiral in off-axis — the spin-around-the-dock
        # seen 2026-06-06), we use the robot's pose in the DOCK frame:
        #   cross_track  = sideways offset from the dock centre-line (m)
        #   range_perp   = perpendicular distance to the marker plane (m)
        #   heading_err  = robot heading vs the dock normal (rad), 0 = head-on
        # and run a line-follower IN THE DOCK FRAME: align to the normal (heading) while
        # steering onto the centre-line (cross). Forward motion is what removes the lateral
        # offset (you cannot null it by rotating in place), so we creep while heading is
        # roughly aligned and let the curve carry us onto the axis — arriving head-on.
        lateral, rng, _yaw, _age, cross, range_perp, heading_err = self._aruco_marker
        target_range = (self._aruco.setpoint_range if self._aruco.use_taught_setpoint
                        else self._aruco.stop_range_m)
        range_err = range_perp - target_range                  # +: still too far to go
        heading_deg = math.degrees(heading_err)

        # HARD ANTI-WALL FLOOR on the perpendicular distance to the marker plane.
        if self._aruco.use_taught_setpoint:
            min_range = max(self._aruco.min_range_m,
                            self._aruco.setpoint_range - self._aruco.range_floor_margin)
        else:
            min_range = self._aruco.min_range_m
        if range_perp < min_range:
            self._logger.error(
                f"VISUAL_SERVO abort: perp range {range_perp:.2f}m < safety floor "
                f"{min_range:.2f}m (too close to marker/wall)"
            )
            self._docking_sub_state = DockingSubState.FAILED
            self._last_servo_rng = None
            if self._on_dock_failed:
                self._on_dock_failed(f"zu nah am Marker ({range_perp:.2f}m)")
            return 0.0, 0.0

        # Act ONLY on a NEW marker frame (move-measure-move). The 10 Hz loop outruns the
        # ~5 Hz marker (which freezes ~1 s under motion); steering on a stale frame
        # over-drives blind between updates. On a duplicate, HOLD STILL.
        marker_key = (lateral, rng)
        if marker_key == self._last_processed_marker:
            return 0.0, 0.0
        self._last_processed_marker = marker_key

        # Range-jump reject (ArUco close-range instability) → steer, don't creep.
        range_jump = (self._last_servo_rng is not None
                      and abs(range_perp - self._last_servo_rng) > self._aruco.max_range_jump)
        if range_jump:
            self._logger.warn(
                f"VISUAL_SERVO: range jump {self._last_servo_rng:.2f}→{range_perp:.2f}m "
                f"(> {self._aruco.max_range_jump:.2f}m) — holding forward motion"
            )
        self._last_servo_rng = range_perp

        # DOCKED: on the centre-line AND square to the normal AND at the standoff, for N
        # consecutive fresh frames. No contact sensor → tight geometric proxy.
        on_target = (abs(cross) <= self._aruco.dock_lateral_tolerance
                     and abs(heading_deg) <= self._aruco.dock_heading_tolerance_deg
                     and abs(range_err) <= self._aruco.dock_range_tolerance
                     and not range_jump)
        if on_target:
            self._visual_dock_confirm_counter += 1
            if self._visual_dock_confirm_counter >= self._aruco.dock_confirm_readings:
                self._docking_sub_state = DockingSubState.COMPLETED
                if on_complete:
                    on_complete()
                self._logger.info(
                    f"VISUAL dock complete (perp {range_perp:.3f}m vs {target_range:.3f}m, "
                    f"cross {cross*100:.1f}cm, heading {heading_deg:.1f}°) — marker-confirmed"
                )
                return 0.0, 0.0
        else:
            self._visual_dock_confirm_counter = 0

        # Steering: align to the normal (heading) + steer onto the centre-line (cross),
        # with heading damping. cross is a bounded metre offset (it does NOT blow up at
        # close range the way a bearing angle does), so this stays stable to the dock.
        now_proc = time.monotonic()
        d_heading = 0.0
        if self._last_servo_bearing_deg is not None and self._last_servo_proc_time is not None:
            dt_proc = max(0.05, now_proc - self._last_servo_proc_time)
            d_heading = (heading_deg - self._last_servo_bearing_deg) / dt_proc
        self._last_servo_bearing_deg = heading_deg
        self._last_servo_proc_time = now_proc
        # Line-follower sign rule: the heading term and the cross-track term must have
        # OPPOSITE signs to converge (offset-right wants steer-left; nose-left wants
        # steer-right). Verified by a geometry derivation 2026-06-06 and matching the
        # proven GPS FOLLOW_LINE law (+heading − lateral). Same-sign reinforces → diverges.
        # invert_steering only flips the absolute convention (both terms together).
        steering = (-(self._aruco.gain_heading_deg * heading_deg
                      + self._aruco.gain_heading_d * d_heading)
                    + self._aruco.gain_cross * cross)
        if self._aruco.invert_steering:
            steering = -steering
        steering = max(-self._aruco.max_steering, min(self._aruco.max_steering, steering))

        # Creep forward ONLY when on-axis (cross small) AND roughly heading-aligned. The
        # anti-wall rule: never drive toward the standoff while off-axis — that drives into
        # the wall beside the markers (RCA 2026-06-06). If it can't get on-axis it simply
        # won't creep → servo_timeout → GPS fallback (safe), never a crash. The overshoot
        # margin stops creep early so the ~13 Hz near-continuous creep coasts to the
        # standoff instead of past it into the wall.
        heading_ok = abs(heading_deg) <= self._aruco.creep_heading_gate_deg
        cross_ok = abs(cross) <= self._aruco.align_cross_tol
        at_or_past_standoff = range_perp <= (target_range + self._aruco.creep_stop_margin)
        stop_forward = (range_jump or range_perp <= min_range or at_or_past_standoff
                        or not heading_ok or not cross_ok)
        speed = 0.0 if stop_forward else self._aruco.creep_speed_ratio * 100.0

        # [DOCKDIAG] TEMP: full servo internals (dock-frame errors).
        self._logger.info(
            f"[DOCKDIAG] servo: cross={cross:.3f} heading={heading_deg:.1f} perp={range_perp:.3f} "
            f"target={target_range:.2f} range_err={range_err:.3f} steer={steering:.1f} "
            f"speed={speed:.1f} head_ok={heading_ok} cross_ok={cross_ok} stop_fwd={stop_forward} jump={range_jump} "
            f"confirm={self._visual_dock_confirm_counter}/{self._aruco.dock_confirm_readings} "
            f"min_range={min_range:.2f}"
        )

        return steering, speed

    def _revert_to_follow_line(self, now: float) -> None:
        """Hand authority back to the GPS line-follow and block immediate re-engagement
        (so the marker servo and GPS cannot oscillate)."""
        self._docking_sub_state = DockingSubState.FOLLOW_LINE
        self._last_lateral_error = None
        self._last_heading_error = None
        self._visual_servo_start_time = None
        self._visual_dock_confirm_counter = 0
        self._last_servo_rng = None
        self._last_servo_bearing_deg = None
        self._last_processed_marker = None
        self._last_servo_proc_time = None
        self._marker_lost_since = None
        self._reengage_block_until = now + self._aruco.reengage_cooldown

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
        
        # Apply minimum steering whenever we're still outside the alignment window.
        # Previously gated at >10°, which left a 2-10° band where the command could be
        # too weak to break grass stiction: rotation stalled, the deadlock booster
        # ramped, then jerked past the target — the hunting that prevented settling.
        if abs(error_deg) > self._config.alignment_tolerance:
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
