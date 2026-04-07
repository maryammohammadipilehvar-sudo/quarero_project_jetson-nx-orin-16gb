"""Avoidance state enumeration."""

from enum import IntEnum


class AvoidanceState(IntEnum):
    """Obstacle avoidance states.
    
    States:
        FREE_DRIVE: No obstacles detected, normal driving
        SLOW_APPROACH: Obstacle in 1.0-2.0m range, reduced speed
        EMERGENCY_STOP: Critical situation < 0.3m, immediate stop
    """
    FREE_DRIVE = 0
    SLOW_APPROACH = 1
    EMERGENCY_STOP = 2

