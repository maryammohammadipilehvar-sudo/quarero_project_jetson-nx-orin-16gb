"""SLOW_APPROACH state implementation."""

from typing import Optional, Set, Tuple
import logging

from .base_avoidance_state import BaseAvoidanceState
from ..avoidance_state import AvoidanceState
from ..avoidance_context import (
    AvoidanceContext,
    has_any_blocked_sectors,
    has_blocked_stop_sectors,
    has_blocked_slow_sectors
)


class SlowApproachState(BaseAvoidanceState):
    """SLOW_APPROACH state: Obstacle in 1.0-2.0m range, reduced speed.
    
    In this state, the robot reduces speed proportionally to the distance
    to the obstacle, but maintains the original steering direction.
    """
    
    def __init__(
        self,
        state: AvoidanceState = AvoidanceState.SLOW_APPROACH,
        logger: Optional[logging.Logger] = None,
        speed_reduction: float = 0.6
    ):
        """Initialize SLOW_APPROACH state.
        
        Args:
            state: State enum value
            logger: Optional logger
            speed_reduction: Speed factor to apply (0.0-1.0)
        """
        super().__init__(state, logger)
        self.speed_reduction = speed_reduction
    
    def can_enter(self, context: AvoidanceContext) -> bool:
        """SLOW_APPROACH can be entered when only SLOW sectors are blocked.
        
        Args:
            context: Current avoidance context
            
        Returns:
            True if only SLOW sectors are blocked (no STOP sectors blocked)
        """
        return has_blocked_slow_sectors(context) and not has_blocked_stop_sectors(context)
    
    def can_exit_to(self, target_state: AvoidanceState, context: AvoidanceContext) -> bool:
        """Check if transition to target state is allowed.
        
        SLOW_APPROACH can transition to:
        - FREE_DRIVE: When no sectors are blocked
        - EMERGENCY_STOP: When at least one STOP sector is blocked
        
        Args:
            target_state: Target state to transition to
            context: Current avoidance context
            
        Returns:
            True if transition is allowed
        """
        if target_state == AvoidanceState.FREE_DRIVE:
            return not has_any_blocked_sectors(context)
        
        if target_state == AvoidanceState.EMERGENCY_STOP:
            return has_blocked_stop_sectors(context)
        
        return False
    
    def get_valid_transitions(self, context: AvoidanceContext, enabled: bool = True) -> Set[AvoidanceState]:
        """Get valid transitions from SLOW_APPROACH.
        
        Args:
            context: Current avoidance context
            enabled: Whether obstacle avoidance is enabled (default: True)
            
        Returns:
            Set of valid target states
        """
        valid = set()
        
        # EMERGENCY_STOP has highest priority: at least one STOP sector blocked
        if has_blocked_stop_sectors(context):
            valid.add(AvoidanceState.EMERGENCY_STOP)
        # FREE_DRIVE: no sectors blocked
        elif not has_any_blocked_sectors(context):
            valid.add(AvoidanceState.FREE_DRIVE)
        
        return valid
    
    def on_update(self, context: AvoidanceContext) -> Tuple[Optional[float], Optional[float]]:
        """Reduce speed in SLOW_APPROACH state.
        
        Args:
            context: Current avoidance context
            
        Returns:
            (steering, modified_speed) where steering is unchanged, speed is reduced
        """
        if context.original_speed is None:
            return None, None
        
        # Apply speed reduction
        modified_speed = context.original_speed * self.speed_reduction
        
        # Return original steering (None = no change) and modified speed
        return context.original_steering, modified_speed

