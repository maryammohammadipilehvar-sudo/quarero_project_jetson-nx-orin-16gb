"""IDLE state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging
import time

if TYPE_CHECKING:
    from rclpy.node import Node

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class IdleState(BaseState):
    """IDLE state: Waiting for commands.
    
    In this state, the robot is idle and waiting for commands. It can transition to:
    - DOCKED: If already at charge position
    - NAVIGATING: If schedule is active OR route is active and conditions are met
    - RETURNING_TO_HOME: If need to return home or route is not active
    - UNDOCKING: When undocking is initiated
    - MANUAL: Manual override (always allowed)
    
    Auto-transition priorities in determine_next_state():
    Priority 1: DOCKED - If at charge position with RTK fix
    Priority 2: RETURNING_TO_HOME - If charging was requested (need_charge=True)
    Priority 3: DOCKING - If at home position with no active route/schedule (with grace period)
    Priority 4: NAVIGATING - If route is active (resume after stop/restart)
    
    Grace period: When entering IDLE from NAVIGATING or RETURNING_TO_HOME (i.e., after
    completing a route), a grace period prevents auto-docking (Priority 3) to allow
    the scheduler time to load the next route for repetition or continuation.
    """
    
    # Grace period after entering IDLE from NAVIGATING/RETURNING_TO_HOME before allowing DOCKING
    # This prevents race conditions where the scheduler hasn't loaded the next route yet
    # during route repetition (Pingpong/Loop mode) or multi-route schedules
    GRACE_PERIOD_SECONDS = 5.0
    
    def __init__(
        self, 
        state: RobotState = RobotState.IDLE,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
        self._entry_time: Optional[float] = None
        self._entered_from_navigation: bool = False
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """IDLE can be entered from most states.
        
        Args:
            context: Current robot context
            
        Returns:
            True (always allowed)
        """
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        IDLE can transition to:
        - DOCKED: If at charge position with RTK fix
        - DOCKING: If autonomous operation enabled and at home position
        - NAVIGATING: If (schedule OR route) active, not docked, not need charge, RTK fix, sensors valid
        - RETURNING_TO_HOME: If not docked, (route not active or need charge), RTK fix, sensors valid
        - UNDOCKING: Always allowed (when docking controller initiates)
        - MANUAL: Always allowed (manual override)
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Manual override always allowed
        if target_state == RobotState.MANUAL:
            return True
        
        # UNDOCKING always allowed (when docking controller starts undocking)
        if target_state == RobotState.UNDOCKING:
            return True
        
        # DOCKING: Requires autonomous operation enabled and at/near home position
        # Use both is_at_home_pos and is_near_home_pos to handle GPS jitter
        if target_state == RobotState.DOCKING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "IDLE cannot transition to DOCKING: autonomous operation not enabled"
                )
                return False
            if not context.is_at_home_pos and not context.is_near_home_pos:
                self._logger.info(
                    "IDLE cannot transition to DOCKING: not at/near home position"
                )
                return False
            return True
        
        # DOCKED: Must be at (or near) charge position with RTK fix
        # Using is_near_charge_pos allows recovery after MANUAL mode when GPS has slight drift
        if target_state == RobotState.DOCKED:
            if not context.rtk_fix:
                self._logger.info(
                    "IDLE cannot transition to DOCKED: RTK fix not available"
                )
                return False
            if not context.is_at_charge_pos and not context.is_near_charge_pos:
                self._logger.info(
                    "IDLE cannot transition to DOCKED: not at or near charge position"
                )
                return False
            return True
        
        # NAVIGATING: Requires multiple conditions
        if target_state == RobotState.NAVIGATING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "IDLE cannot transition to NAVIGATING: autonomous operation not enabled"
                )
                return False
            if context.docked:
                self._logger.info(
                    "IDLE cannot transition to NAVIGATING: robot is docked"
                )
                return False
            # Allow transition if schedule is active OR route is active
            # This enables resuming navigation after stopping (autonomous off) 
            # and reactivating (autonomous on) while a route is still active
            if not context.schedule_active and not context.route_active:
                self._logger.info(
                    "IDLE cannot transition to NAVIGATING: neither schedule nor route active"
                )
                return False
            if context.need_charge:
                self._logger.info(
                    "IDLE cannot transition to NAVIGATING: need to charge"
                )
                return False
            if not context.rtk_fix:
                self._logger.info(
                    "IDLE cannot transition to NAVIGATING: RTK fix not available"
                )
                return False
            if not context.sensor_info_valid:
                self._logger.info(
                    "IDLE cannot transition to NAVIGATING: sensor info not valid"
                )
                return False
            return True
        
        # RETURNING_TO_HOME: Requires multiple conditions
        if target_state == RobotState.RETURNING_TO_HOME:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "IDLE cannot transition to RETURNING_TO_HOME: autonomous operation not enabled"
                )
                return False
            if context.docked:
                self._logger.info(
                    "IDLE cannot transition to RETURNING_TO_HOME: robot is docked"
                )
                return False
            # Must have route not active OR need charge
            if context.route_active and not context.need_charge:
                self._logger.info(
                    "IDLE cannot transition to RETURNING_TO_HOME: route is active and no need to charge"
                )
                return False
            if not context.rtk_fix:
                self._logger.info(
                    "IDLE cannot transition to RETURNING_TO_HOME: RTK fix not available"
                )
                return False
            if not context.sensor_info_valid:
                self._logger.info(
                    "IDLE cannot transition to RETURNING_TO_HOME: sensor info not valid"
                )
                return False
            return True
        
        # Unknown target state
        self._logger.info(
            f"IDLE cannot transition to {target_state.name}: invalid target state"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from IDLE.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = set()
        
        # MANUAL always allowed
        valid.add(RobotState.MANUAL)
        
        # UNDOCKING always allowed (when docking controller initiates)
        valid.add(RobotState.UNDOCKING)
        
        # DOCKED: If at (or near) charge position with RTK fix
        if context.rtk_fix and (context.is_at_charge_pos or context.is_near_charge_pos):
            valid.add(RobotState.DOCKED)
        
        # DOCKING: If autonomous operation enabled and at/near home position
        # Use both is_at_home_pos and is_near_home_pos to handle GPS jitter
        if context.autonomous_operation_enabled and (context.is_at_home_pos or context.is_near_home_pos):
            valid.add(RobotState.DOCKING)
        
        # NAVIGATING: If all conditions are met
        # Allow if schedule is active OR route is active (for resuming after stop)
        if (context.autonomous_operation_enabled and 
            not context.docked and 
            (context.schedule_active or context.route_active) and 
            not context.need_charge and 
            context.rtk_fix and 
            context.sensor_info_valid):
            valid.add(RobotState.NAVIGATING)
        
        # RETURNING_TO_HOME: If all conditions are met
        if (context.autonomous_operation_enabled and 
            not context.docked and 
            (not context.route_active or context.need_charge) and 
            context.rtk_fix and 
            context.sensor_info_valid):
            valid.add(RobotState.RETURNING_TO_HOME)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        IDLE handles priority-based automatic transitions:
        Priority 1: DOCKED - If at charge position (or near it) with RTK fix
        Priority 2: RETURNING_TO_HOME - If charging was requested (need_charge=True)
        Priority 3: DOCKING - If at home position with no active route/schedule (auto-dock)
                    NOTE: This is skipped during grace period after route completion
        Priority 4: NAVIGATING - If route is active (resume after stop/restart)
        
        This ensures that when the robot is at the charge position (e.g., after
        startup recovery or after MANUAL mode), it properly transitions to DOCKED
        before any other state decisions are made. The is_near_charge_pos check
        prevents the robot from trying to navigate when it's physically at the
        charging station but GPS has slight drift.
        
        Priority 4 (NAVIGATING) is the lowest priority, enabling the robot to resume
        navigation after being stopped (autonomous off) and reactivated (autonomous on)
        while a route is still active.
        
        Grace period: When entering IDLE from NAVIGATING/RETURNING_TO_HOME (after a
        route completes), Priority 3 (DOCKING) is skipped for GRACE_PERIOD_SECONDS
        to allow the scheduler to load the next route for repetition. This prevents
        the robot from auto-docking between route repetitions in Pingpong/Loop mode.
        
        Args:
            context: Current robot context
            
        Returns:
            Next state based on priority, or None if no auto-transition
        """
        # Priority 1: If at charge position (or near it) with RTK fix -> DOCKED
        # This is critical for:
        # - Startup recovery
        # - Proper docking behavior
        # - Recovery after MANUAL mode (when GPS may have slight drift)
        # Using is_near_charge_pos (40cm tolerance) ensures the robot doesn't
        # try to navigate when it's physically on the charging station
        if (context.is_at_charge_pos or context.is_near_charge_pos) and context.rtk_fix:
            return RobotState.DOCKED
        
        # Priority 2: If charging was requested -> DOCKING (if at home) or RETURNING_TO_HOME
        # This handles the "go to charge" command from scheduler
        # IMPORTANT: Use is_near_home_pos (0.6m) instead of is_at_home_pos (0.5m) to handle GPS jitter
        # The GPS position can fluctuate by 10-20cm, causing is_at_home_pos to flip between True/False
        if (context.need_charge and 
            context.autonomous_operation_enabled and 
            not context.docked and 
            context.rtk_fix and 
            context.sensor_info_valid):
            # If already at home position (or near it), go directly to DOCKING
            # This prevents unnecessary RETURNING_TO_HOME when robot is already at home
            if context.is_at_home_pos or context.is_near_home_pos:
                self._logger.info(
                    f"IDLE: Charging requested and robot at home position "
                    f"(at_home={context.is_at_home_pos}, near_home={context.is_near_home_pos}), "
                    f"transitioning directly to DOCKING"
                )
                return RobotState.DOCKING
            # Otherwise, need to return to home first
            if not context.route_active or context.need_charge:
                return RobotState.RETURNING_TO_HOME
        
        # Check if we're in the grace period after route completion
        in_grace_period = False
        if self._entered_from_navigation and self._entry_time is not None:
            time_since_entry = time.monotonic() - self._entry_time
            in_grace_period = time_since_entry < self.GRACE_PERIOD_SECONDS
            if in_grace_period:
                self._logger.debug(
                    f"IDLE grace period active: {time_since_entry:.1f}s / {self.GRACE_PERIOD_SECONDS}s - "
                    f"skipping auto-dock and auto-navigate checks (waiting for scheduler)"
                )
        
        # Priority 3: Auto-dock when idle at home with no active route/schedule
        # This ensures the robot automatically docks and charges when no mission is active
        # SKIP during grace period to allow scheduler to load next route for repetition
        # IMPORTANT: Use is_near_home_pos (0.6m) in addition to is_at_home_pos (0.5m) 
        # to handle GPS jitter that can cause the position to fluctuate
        if not in_grace_period:
            if (context.autonomous_operation_enabled and 
                (context.is_at_home_pos or context.is_near_home_pos) and 
                not context.docked and 
                not context.route_active and 
                not context.schedule_active and 
                context.rtk_fix and 
                context.sensor_info_valid):
                self._logger.info(
                    f"IDLE: No active route/schedule at home position "
                    f"(at_home={context.is_at_home_pos}, near_home={context.is_near_home_pos}), "
                    f"transitioning to DOCKING"
                )
                return RobotState.DOCKING
        
        # Priority 4: Resume navigation when route is active
        # This is the lowest priority transition, allowing the robot to resume
        # navigation after being stopped (autonomous off) and reactivated (autonomous on)
        # while a route is still active. This prevents the robot from getting stuck
        # in IDLE when it should continue navigating.
        # SKIP during grace period to prevent race condition with stale context
        # (route_active may be True in old context even though route just completed)
        if not in_grace_period:
            if (context.autonomous_operation_enabled and 
                context.route_active and 
                not context.docked and 
                not context.need_charge and 
                context.rtk_fix and 
                context.sensor_info_valid):
                return RobotState.NAVIGATING
        
        # Other transitions are handled by control loop or external triggers
        return None
    
    def on_enter(self, previous_state: Optional[RobotState], context: RobotStateContext):
        """Called when entering IDLE state.
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        
        # Track if we entered from a navigation state (route just completed)
        # In this case, we need a grace period before allowing DOCKING
        # to give the scheduler time to load the next route for repetition
        self._entered_from_navigation = previous_state in [
            RobotState.NAVIGATING, 
            RobotState.RETURNING_TO_HOME
        ]
        
        if self._entered_from_navigation:
            self._entry_time = time.monotonic()
            self._logger.info(
                f"Robot in IDLE state (from {previous_state.name}), "
                f"grace period {self.GRACE_PERIOD_SECONDS}s before auto-dock"
            )
        else:
            self._entry_time = None
            self._logger.info("Robot in IDLE state, waiting for commands")
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting IDLE state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        # Reset grace period tracking
        self._entry_time = None
        self._entered_from_navigation = False
        self._logger.info(f"Exiting IDLE state, transitioning to {next_state.name}")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        In IDLE state, the robot should not perform any actions.
        Just wait for conditions to trigger a transition.
        
        Args:
            context: Current robot context
            **kwargs: Additional data
            
        Returns:
            None (no commands from this state)
        """
        # No actions in IDLE state
        # Just wait for conditions to trigger transitions
        return None

