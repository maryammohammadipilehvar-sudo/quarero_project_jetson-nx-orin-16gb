"""Robot state enumeration."""

from enum import IntEnum


class RobotState(IntEnum):
    """Robot operational states.
    
    States:
        UNINITIALIZED: Waiting for configuration
        MANUAL: Manual control mode
        IDLE: Waiting for commands
        NAVIGATING: Following waypoints
        RETURNING_TO_HOME: Returning to home position
        DOCKING: Docking at charging station
        DOCKED: Successfully docked (physical connection)
        CHARGING: Actively charging
        UNDOCKING: Undocking from charging station
        UNDOCKED: Successfully undocked
        ERROR: Error state
    """
    UNINITIALIZED = 0
    MANUAL = 1
    IDLE = 2
    NAVIGATING = 3
    RETURNING_TO_HOME = 4
    DOCKING = 5
    DOCKED = 6
    CHARGING = 7
    UNDOCKING = 8
    UNDOCKED = 9
    ERROR = 10

