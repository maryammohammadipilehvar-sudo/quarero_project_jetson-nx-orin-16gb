"""FREE_DRIVE state implementation."""

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


class FreeDriveState(BaseAvoidanceState):
    """FREE_DRIVE state: No obstacles detected, normal driving.
    
    In this state, no obstacle avoidance is applied. The robot drives normally
    with the original commands from the waypoint follower.
    """
    
    def __init__(
        self,
        state: AvoidanceState = AvoidanceState.FREE_DRIVE,
        logger: Optional[logging.Logger] = None
    ):
        """Initialize FREE_DRIVE state.
        
        Args:
            state: State enum value
            logger: Optional logger
        """
        super().__init__(state, logger)
    
    def can_enter(self, context: AvoidanceContext) -> bool:
        """FREE_DRIVE can be entered when no sectors are blocked.
        
        Args:
            context: Current avoidance context
            
        Returns:
            True if no sectors are blocked
        """
        return not has_any_blocked_sectors(context)
    
    def can_exit_to(self, target_state: AvoidanceState, context: AvoidanceContext) -> bool:
        """Check if transition to target state is allowed.
        
        FREE_DRIVE can transition to:
        - SLOW_APPROACH: When only SLOW sectors are blocked
        - EMERGENCY_STOP: When at least one STOP sector is blocked
        
        Args:
            target_state: Target state to transition to
            context: Current avoidance context
            
        Returns:
            True if transition is allowed
        """
        if target_state == AvoidanceState.SLOW_APPROACH:
            return has_blocked_slow_sectors(context) and not has_blocked_stop_sectors(context)
        
        if target_state == AvoidanceState.EMERGENCY_STOP:
            return has_blocked_stop_sectors(context)
        
        return False
    
    def get_valid_transitions(self, context: AvoidanceContext, enabled: bool = True) -> Set[AvoidanceState]:
        """Get valid transitions from FREE_DRIVE.
        
        Args:
            context: Current avoidance context
            enabled: Whether obstacle avoidance is enabled (default: True)
            
        Returns:
            Set of valid target states
        """
        # If obstacle avoidance is disabled, no transitions allowed
        if not enabled:
            return set()
        
        valid = set()
        
        # EMERGENCY_STOP has highest priority: at least one STOP sector blocked
        if has_blocked_stop_sectors(context):
            valid.add(AvoidanceState.EMERGENCY_STOP)
        # SLOW_APPROACH: only SLOW sectors blocked (no STOP sectors)
        elif has_blocked_slow_sectors(context):
            valid.add(AvoidanceState.SLOW_APPROACH)
        
        return valid
    
    def on_update(self, context: AvoidanceContext) -> Tuple[Optional[float], Optional[float]]:
        """No modification in FREE_DRIVE state - pass through original commands.
        
        Args:
            context: Current avoidance context
            
        Returns:
            (None, None) to indicate no modification
        """
        # Return None, None to indicate no modification (pass through)
        return None, None

