"""ERROR state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class ErrorState(BaseState):
    """ERROR state: Error condition requiring immediate stop.
    
    In this state, the robot has encountered an error condition and must
    immediately stop all movement. The robot can transition to:
    - IDLE: When error is resolved and system is ready
    - MANUAL: Manual override (always allowed)
    
    ERROR state requires explicit transition - no automatic transitions.
    The robot should remain stopped until an external command or condition
    triggers a transition to IDLE or MANUAL.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.ERROR,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """ERROR can be entered from any state (emergency override).
        
        Args:
            context: Current robot context
            
        Returns:
            True (always allowed)
        """
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        ERROR can only exit to IDLE or MANUAL states.
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition to IDLE or MANUAL, False otherwise
        """
        return target_state in [RobotState.IDLE, RobotState.MANUAL]
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from ERROR.
        
        Args:
            context: Current robot context
            
        Returns:
            Set containing IDLE and MANUAL states
        """
        return {RobotState.IDLE, RobotState.MANUAL}
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        ERROR state has no automatic transitions - requires explicit reset.
        
        Args:
            context: Current robot context
            
        Returns:
            None (no auto-transitions)
        """
        return None
    
    def on_enter(self, previous_state: Optional[RobotState], context: RobotStateContext):
        """Called when entering ERROR state.
        
        Logs the error state entry and ensures robot stops.
        
        Args:
            previous_state: Previous state before transition
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.error(
            f"Entered ERROR state from {previous_state.name if previous_state else 'None'}"
        )
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting ERROR state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        self._logger.info(f"Exiting ERROR state to {next_state.name}")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        Returns stop commands (0.0, 0.0) to ensure robot remains stationary
        in error state.
        
        Args:
            context: Current robot context
            **kwargs: Additional data (not used in ERROR state)
            
        Returns:
            Tuple of (steering=0.0, speed=0.0) to stop the robot
        """
        # Return stop commands
        return 0.0, 0.0

