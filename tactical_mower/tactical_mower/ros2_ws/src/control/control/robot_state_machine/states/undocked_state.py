"""UNDOCKED state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class UndockedState(BaseState):
    """UNDOCKED state: Successfully undocked.
    
    In this state, the robot has successfully undocked from the charging station
    and is at the home position (but not at charge position). The robot can
    transition to:
    - DOCKING: When need to dock (autonomous enabled, need charge or route not active)
    - NAVIGATING: When ready to navigate (autonomous enabled, route active, not need charge, schedule active, RTK fix, sensors valid)
    - IDLE: Always allowed
    - MANUAL: Manual override (always allowed)
    
    UNDOCKED has no automatic transitions - transitions are explicitly triggered
    in the control loop based on conditions.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.UNDOCKED,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if UNDOCKED can be entered.
        
        UNDOCKED can be entered when robot is at home position.
        If robot is also at charge position (due to overlapping tolerances), allow transition
        if robot is at home position (indicates successful undocking to home).
        
        Args:
            context: Current robot context
            
        Returns:
            True if UNDOCKED can be entered
        """
        # UNDOCKED can be entered when at home position
        # If robot is at home, it has successfully undocked, even if still within charge tolerance
        if not context.is_at_home_pos:
            # Only log once per state change attempt (avoid spam)
            if not hasattr(self, '_last_can_enter_log') or self._last_can_enter_log != 'not_at_home':
                self._logger.debug(
                    "Cannot enter UNDOCKED: not at home position"
                )
                self._last_can_enter_log = 'not_at_home'
            return False
        
        # If robot is at home position, allow transition to UNDOCKED
        # Even if is_at_charge_pos is True (due to overlapping tolerances when charge and home are close)
        # The key indicator is that robot reached home position, which means undocking was successful
        if context.is_at_charge_pos:
            # Log that we're allowing transition despite being at charge position
            # This happens when charge and home positions are very close (< 1m apart)
            if not hasattr(self, '_last_can_enter_log') or self._last_can_enter_log != 'at_charge_allowed':
                self._logger.debug(
                    "Allowing UNDOCKED transition: robot at home position (charge position overlap due to close proximity)"
                )
                self._last_can_enter_log = 'at_charge_allowed'
        
        # Reset log flag on successful check
        if hasattr(self, '_last_can_enter_log'):
            delattr(self, '_last_can_enter_log')
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        UNDOCKED can transition to:
        - DOCKING: If autonomous enabled and (need charge or route not active)
        - NAVIGATING: If autonomous enabled, route active, not need charge, schedule active, RTK fix, sensors valid
        - IDLE: Always allowed
        - MANUAL: Always allowed
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Manual override always allowed
        if target_state == RobotState.MANUAL:
            return True
        
        # IDLE always allowed
        if target_state == RobotState.IDLE:
            return True
        
        # DOCKING: Requires autonomous operation and conditions
        if target_state == RobotState.DOCKING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "UNDOCKED cannot transition to DOCKING: autonomous operation not enabled"
                )
                return False
            
            # Can dock if need charge OR route not active
            if not (context.need_charge or not context.route_active):
                self._logger.info(
                    "UNDOCKED cannot transition to DOCKING: route is active and no need to charge"
                )
                return False
            
            return True
        
        # NAVIGATING: Requires multiple conditions
        if target_state == RobotState.NAVIGATING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "UNDOCKED cannot transition to NAVIGATING: autonomous operation not enabled"
                )
                return False
            
            if not context.route_active:
                self._logger.info(
                    "UNDOCKED cannot transition to NAVIGATING: route not active"
                )
                return False
            
            if context.need_charge:
                self._logger.info(
                    "UNDOCKED cannot transition to NAVIGATING: need to charge"
                )
                return False
            
            if not context.schedule_active:
                self._logger.info(
                    "UNDOCKED cannot transition to NAVIGATING: schedule not active"
                )
                return False
            
            if not context.rtk_fix:
                self._logger.info(
                    "UNDOCKED cannot transition to NAVIGATING: RTK fix not available"
                )
                return False
            
            if not context.sensor_info_valid:
                self._logger.info(
                    "UNDOCKED cannot transition to NAVIGATING: sensor info not valid"
                )
                return False
            
            return True
        
        # Unknown target state
        self._logger.info(
            f"UNDOCKED cannot transition to {target_state.name}: invalid target state"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from UNDOCKED.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = {RobotState.MANUAL, RobotState.IDLE}
        
        if context.autonomous_operation_enabled:
            # Can transition to DOCKING if need charge or route not active
            if context.need_charge or not context.route_active:
                valid.add(RobotState.DOCKING)
            
            # Can transition to NAVIGATING if all conditions are met
            if (context.route_active and 
                not context.need_charge and 
                context.schedule_active and 
                context.rtk_fix and 
                context.sensor_info_valid):
                valid.add(RobotState.NAVIGATING)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        UNDOCKED has no automatic transitions. All transitions are explicitly
        triggered in the control loop based on conditions.
        
        Args:
            context: Current robot context
            
        Returns:
            None (no automatic transitions)
        """
        # No automatic transitions from UNDOCKED
        # Transitions should be explicitly triggered in control loop
        return None
    
    def on_enter(
        self, 
        previous_state: Optional[RobotState], 
        context: RobotStateContext
    ):
        """Called when entering UNDOCKED state.
        
        Args:
            previous_state: Previous state (typically UNDOCKING)
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info("Robot successfully undocked - ready for next operation")
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting UNDOCKED state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        self._logger.info(f"Exiting UNDOCKED state, transitioning to {next_state.name}")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        In UNDOCKED state, the robot should not perform any actions.
        Just wait for conditions to trigger a transition.
        
        Args:
            context: Current robot context
            **kwargs: Additional data
            
        Returns:
            None (no commands from this state)
        """
        # No actions in UNDOCKED state
        # Just wait for conditions to trigger transitions
        return None

