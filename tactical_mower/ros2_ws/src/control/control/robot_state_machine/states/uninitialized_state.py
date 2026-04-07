"""UNINITIALIZED state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class UninitializedState(BaseState):
    """UNINITIALIZED state: Waiting for configuration.
    
    This is the initial state when the robot starts up. The robot remains
    in this state until the charge position is configured. Once configured,
    it automatically transitions to MANUAL state.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.UNINITIALIZED,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """UNINITIALIZED can only be entered at startup.
        
        Args:
            context: Current robot context
            
        Returns:
            True (only entered at startup)
        """
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        UNINITIALIZED can only transition to MANUAL, and only when
        the charge position has been set.
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Can only transition to MANUAL
        if target_state != RobotState.MANUAL:
            self._logger.info(
                f"UNINITIALIZED can only transition to MANUAL, not {target_state.name}"
            )
            return False
        
        # Charge position must be set
        if not context.charge_pos_set:
            self._logger.info(
                "UNINITIALIZED cannot transition to MANUAL: charge position not set"
            )
            return False
        
        return True
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from UNINITIALIZED.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = set()
        
        # Can transition to MANUAL if charge position is set
        if context.charge_pos_set:
            valid.add(RobotState.MANUAL)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        Automatically transitions to MANUAL when charge position is configured.
        
        Args:
            context: Current robot context
            
        Returns:
            MANUAL if charge position is set, None otherwise
        """
        if context.charge_pos_set:
            self._logger.info("Charge position set, transitioning to MANUAL")
            return RobotState.MANUAL
        
        return None
    
    def on_enter(self, previous_state: Optional[RobotState], context: RobotStateContext):
        """Called when entering UNINITIALIZED state.
        
        Args:
            previous_state: Previous state (None if initial)
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info(
            "Robot in UNINITIALIZED state. Waiting for charge position configuration..."
        )
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        In UNINITIALIZED state, the robot should not perform any actions.
        Just wait for configuration.
        
        Args:
            context: Current robot context
            **kwargs: Additional data
            
        Returns:
            None (no commands from this state)
        """
        # No actions in UNINITIALIZED state
        # Just wait for charge position to be set
        return None

