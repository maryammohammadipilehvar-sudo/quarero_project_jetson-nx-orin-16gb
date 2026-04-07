"""Factory for creating state instances."""

from typing import Dict, Optional, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .robot_state import RobotState
from .states.base_state import BaseState

# Import state classes (will be implemented)
from .states.uninitialized_state import UninitializedState
from .states.manual_state import ManualState
from .states.idle_state import IdleState
from .states.navigating_state import NavigatingState
from .states.returning_to_home_state import ReturningToHomeState
from .states.docking_state import DockingState
from .states.docked_state import DockedState
from .states.charging_state import ChargingState
from .states.undocking_state import UndockingState
from .states.undocked_state import UndockedState
from .states.error_state import ErrorState


class StateFactory:
    """Factory for creating state instances (singleton pattern per state)."""
    
    _state_classes: Dict[RobotState, type[BaseState]] = {
        RobotState.UNINITIALIZED: UninitializedState,
        RobotState.MANUAL: ManualState,
        RobotState.IDLE: IdleState,
        RobotState.NAVIGATING: NavigatingState,
        RobotState.RETURNING_TO_HOME: ReturningToHomeState,
        RobotState.DOCKING: DockingState,
        RobotState.DOCKED: DockedState,
        RobotState.CHARGING: ChargingState,
        RobotState.UNDOCKING: UndockingState,
        RobotState.UNDOCKED: UndockedState,
        RobotState.ERROR: ErrorState,
    }
    
    _instances: Dict[RobotState, BaseState] = {}
    _logger: logging.Logger = logging.getLogger(__name__)
    
    @classmethod
    def get_state(
        cls, 
        state: RobotState, 
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ) -> BaseState:
        """Get state instance (singleton per state).
        
        Args:
            state: State enum value
            node: ROS node instance to pass to state
            logger: Optional logger instance to pass to state (if node not provided)
            
        Returns:
            State instance
            
        Raises:
            ValueError: If state is unknown
        """
        if state not in cls._instances:
            state_class = cls._state_classes.get(state)
            if state_class is None:
                raise ValueError(f"Unknown state: {state}")
            
            # Create state with node and logger
            cls._instances[state] = state_class(
                state=state,
                node=node,
                logger=logger
            )
            cls._logger.debug(f"Created state instance for {state.name}")
        
        return cls._instances[state]
    
    @classmethod
    def reset(cls):
        """Reset all state instances (for testing)."""
        cls._instances.clear()
        cls._logger.debug("Reset all state instances")

