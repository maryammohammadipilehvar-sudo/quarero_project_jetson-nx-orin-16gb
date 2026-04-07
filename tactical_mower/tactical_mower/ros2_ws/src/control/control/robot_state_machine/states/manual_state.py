"""MANUAL state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class ManualState(BaseState):
    """MANUAL state: Manual control mode.
    
    In this state, the robot is under manual control (e.g., via joystick).
    The robot transitions to MANUAL when autonomous_operation_enabled is False.
    The robot can transition to IDLE when autonomous_operation_enabled is True.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.MANUAL,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """MANUAL can be entered from any state (manual override).
        
        Manual mode has highest priority and can always be entered.
        This happens when autonomous_operation_enabled is False.
        
        Args:
            context: Current robot context
            
        Returns:
            True (always allowed)
        """
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        MANUAL can only transition to IDLE. IDLE then handles priority-based
        transitions to other states (DOCKED, NAVIGATING, etc.).
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Can only transition to IDLE
        if target_state == RobotState.IDLE:
            # For startup recovery and recovery after MANUAL mode:
            # allow transition even without autonomous enabled
            # if we're at (or near) charge position with RTK fix
            if (context.is_at_charge_pos or context.is_near_charge_pos) and context.rtk_fix:
                return True
            
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "MANUAL cannot transition to IDLE: autonomous operation not enabled"
                )
                return False
            
            return True
        
        # Cannot transition to other states from MANUAL
        self._logger.info(
            f"MANUAL can only transition to IDLE, not {target_state.name}"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from MANUAL.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = set()
        
        # Can transition to IDLE if autonomous operation is enabled
        # OR if at (or near) charge position with RTK fix (startup recovery / recovery after MANUAL)
        if context.autonomous_operation_enabled or ((context.is_at_charge_pos or context.is_near_charge_pos) and context.rtk_fix):
            valid.add(RobotState.IDLE)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        Automatically transitions to IDLE when autonomous operation is enabled.
        
        Args:
            context: Current robot context
            
        Returns:
            IDLE if autonomous operation is enabled, None otherwise
        """
        # Exit MANUAL mode when autonomous operation is enabled
        if context.autonomous_operation_enabled:
            self._logger.info(
                "Autonomous operation enabled, transitioning from MANUAL to IDLE"
            )
            return RobotState.IDLE
        
        return None
    
    def on_enter(self, previous_state: Optional[RobotState], context: RobotStateContext):
        """Called when entering MANUAL state.
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info("Robot in MANUAL control mode")
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting MANUAL state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        if next_state == RobotState.IDLE:
            self._logger.info("Exiting MANUAL mode, entering autonomous operation")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        In MANUAL state, the robot should accept manual control commands
        (e.g., from joystick). The state itself doesn't need to do anything
        here as the control commands are handled elsewhere.
        
        Args:
            context: Current robot context
            **kwargs: Additional data
            
        Returns:
            None (no commands from this state)
        """
        # Manual control is handled by the joystick controller
        # This state just needs to exist to allow manual operation
        return None

