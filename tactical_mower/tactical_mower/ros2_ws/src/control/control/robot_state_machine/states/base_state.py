"""Abstract base class for all robot states."""

from abc import ABC, abstractmethod
from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class BaseState(ABC):
    """Abstract base class for all robot states.
    
    Each state class implements the behavior and transition logic for a specific
    robot operational state. This follows the State pattern for better organization.
    
    States have access to a ROS node for publishing/subscribing to topics,
    calling services, and other ROS operations.
    """
    
    def __init__(
        self, 
        state: RobotState, 
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        """Initialize state.
        
        Args:
            state: The RobotState enum value this state represents
            node: ROS node instance for ROS operations (publishers, subscribers, etc.)
            logger: Optional logger instance. If None, uses node's logger or creates a new one.
        """
        self._state = state
        self._node = node
        
        # Use node's logger if available, otherwise create one
        if logger is not None:
            self._logger = logger
        elif node is not None:
            self._logger = node.get_logger()
        else:
            self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
    
    @property
    def state(self) -> RobotState:
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
    
    @property
    def node(self) -> Optional['Node']:
        """Get ROS node instance."""
        return self._node
    
    @abstractmethod
    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if this state can be entered from current context.
        
        Args:
            context: Current robot context
            
        Returns:
            True if state can be entered
        """
        pass
    
    @abstractmethod
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        pass
    
    @abstractmethod
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get set of valid states that can be transitioned to.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        pass
    
    @abstractmethod
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine the next state based on context (auto-transitions).
        
        This method is called by the state machine to determine if an automatic
        transition should occur. Return None to stay in current state.
        
        Args:
            context: Current robot context
            
        Returns:
            Next state to transition to, or None to stay in current state
        """
        pass
    
    def on_enter(self, previous_state: Optional[RobotState], context: RobotStateContext):
        """Called when entering this state.
        
        Args:
            previous_state: Previous state (None if initial)
            context: Current robot context
        """
        self._logger.info(f"Entering state {self.name} from {previous_state.name if previous_state else 'None'}")
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting this state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        self._logger.info(f"Exiting state {self.name} to {next_state.name}")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        This method is called repeatedly while the state machine is in this state.
        Use this to execute state-specific behavior (e.g., control commands).
        
        Args:
            context: Current robot context
            **kwargs: Additional data (e.g., robot_pose, callbacks, etc.)
            
        Returns:
            Tuple of (steering, speed) commands, or None if no commands.
            Both steering and speed can be None to indicate no command.
        """
        return None

