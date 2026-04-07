"""Robot State Machine package for managing high-level robot operational states.

This package implements a state machine using the State pattern where each state
is represented by its own class, providing better organization and maintainability.
"""

from .robot_state import RobotState
from .robot_state_context import RobotStateContext
from .robot_state_machine import RobotStateMachine

__all__ = [
    'RobotState',
    'RobotStateContext',
    'RobotStateMachine',
]

