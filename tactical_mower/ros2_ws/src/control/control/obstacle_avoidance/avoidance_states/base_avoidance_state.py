"""Abstract base class for all avoidance states."""

from abc import ABC, abstractmethod
from typing import Optional, Set, Tuple
import logging

from ..avoidance_state import AvoidanceState
from ..avoidance_context import AvoidanceContext


class BaseAvoidanceState(ABC):
    """Abstract base class for all avoidance states.
    
    Each state class implements the behavior and transition logic for a specific
    avoidance state. This follows the State pattern for better organization.
    """
    
    def __init__(
        self,
        state: AvoidanceState,
        logger: Optional[logging.Logger] = None
    ):
        """Initialize state.
        
        Args:
            state: The AvoidanceState enum value this state represents
            logger: Optional logger instance
        """
        self._state = state
        self._logger = logger or logging.getLogger(f"{__name__}.{self.__class__.__name__}")
    
    @property
    def state(self) -> AvoidanceState:
        """Get the state enum value."""
        return self._state
    
    @property
    def name(self) -> str:
        """Get state name."""
        return self._state.name
    
    @property
    def logger(self) -> logging.Logger:
        """Get logger instance."""
        return self._logger
    
    @abstractmethod
    def can_enter(self, context: AvoidanceContext) -> bool:
        """Check if this state can be entered from current context.
        
        Args:
            context: Current avoidance context
            
        Returns:
            True if state can be entered
        """
        pass
    
    @abstractmethod
    def can_exit_to(self, target_state: AvoidanceState, context: AvoidanceContext) -> bool:
        """Check if transition to target state is allowed.
        
        Args:
            target_state: Target state to transition to
            context: Current avoidance context
            
        Returns:
            True if transition is allowed
        """
        pass
    
    @abstractmethod
    def get_valid_transitions(self, context: AvoidanceContext, enabled: bool = True) -> Set[AvoidanceState]:
        """Get set of valid states that can be transitioned to.
        
        Args:
            context: Current avoidance context
            enabled: Whether obstacle avoidance is enabled (default: True)
            
        Returns:
            Set of valid target states
        """
        pass
    
    def on_enter(self, previous_state: Optional[AvoidanceState], context: AvoidanceContext):
        """Called when entering this state.
        
        Args:
            previous_state: Previous state (None if initial)
            context: Current avoidance context
        """
        self._logger.debug(
            f"Entering avoidance state {self.name} from "
            f"{previous_state.name if previous_state else 'None'}"
        )
    
    def on_exit(self, next_state: AvoidanceState, context: AvoidanceContext):
        """Called when exiting this state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current avoidance context
        """
        self._logger.debug(f"Exiting avoidance state {self.name} to {next_state.name}")
    
    @abstractmethod
    def on_update(
        self,
        context: AvoidanceContext
    ) -> Tuple[Optional[float], Optional[float]]:
        """Called during state update (called in control loop).
        
        This method is called repeatedly while the state machine is in this state.
        Use this to execute state-specific behavior (e.g., modify commands).
        
        Args:
            context: Current avoidance context
            
        Returns:
            Tuple of (steering, speed) commands, or (None, None) if no modification.
            Both steering and speed can be None to indicate no command modification.
        """
        pass

