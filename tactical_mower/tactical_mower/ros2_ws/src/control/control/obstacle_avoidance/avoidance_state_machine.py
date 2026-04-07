"""State machine for obstacle avoidance."""

import logging
from typing import Optional, Tuple

from .avoidance_state import AvoidanceState
from .avoidance_context import AvoidanceContext
from .avoidance_states.base_avoidance_state import BaseAvoidanceState
from .avoidance_states.free_drive_state import FreeDriveState
from .avoidance_states.slow_approach_state import SlowApproachState
from .avoidance_states.emergency_stop_state import EmergencyStopState


class AvoidanceStateMachine:
    """State machine for obstacle avoidance using State pattern."""
    
    def __init__(
        self,
        logger: Optional[logging.Logger] = None,
        speed_reduction_slow: float = 0.6,
        initial_stop_duration: float = 3.0
    ):
        """Initialize avoidance state machine.
        
        Args:
            logger: Optional logger instance
            speed_reduction_slow: Speed factor for SLOW_APPROACH
            initial_stop_duration: Duration to fully stop (steering=0, speed=0) when entering EMERGENCY_STOP (s)
        """
        self._logger = logger or logging.getLogger(__name__)
        
        # Create state instances with parameters
        self._states: dict[AvoidanceState, BaseAvoidanceState] = {
            AvoidanceState.FREE_DRIVE: FreeDriveState(
                logger=self._logger
            ),
            AvoidanceState.SLOW_APPROACH: SlowApproachState(
                speed_reduction=speed_reduction_slow,
                logger=self._logger
            ),
            AvoidanceState.EMERGENCY_STOP: EmergencyStopState(
                initial_stop_duration=initial_stop_duration,
                logger=self._logger
            ),
        }
        
        # Start in FREE_DRIVE state
        self._current_state: BaseAvoidanceState = self._states[AvoidanceState.FREE_DRIVE]
        self._previous_state: Optional[AvoidanceState] = None
    
    def get_state(self) -> AvoidanceState:
        """Get current state.
        
        Returns:
            Current avoidance state
        """
        return self._current_state.state
    
    def update(self, context: AvoidanceContext, enabled: bool = True) -> Tuple[Optional[float], Optional[float]]:
        """Update state machine and get commands from current state.
        
        Args:
            context: Current avoidance context
            enabled: Whether obstacle avoidance is enabled (default: True)
            
        Returns:
            Tuple of (steering, speed) commands from current state
        """
        # Determine next state based on context and enabled flag
        next_state = self._determine_next_state(context, enabled)
        
        # Transition if needed
        if next_state is not None and next_state != self._current_state.state:
            self._transition_to(next_state, context)
        
        # Get commands from current state
        return self._current_state.on_update(context)
    
    def _determine_next_state(self, context: AvoidanceContext, enabled: bool = True) -> Optional[AvoidanceState]:
        """Determine next state based on context.
        
        Args:
            context: Current avoidance context
            enabled: Whether obstacle avoidance is enabled (default: True)
            
        Returns:
            Next state to transition to, or None to stay in current state
        """
        # If disabled, only allow transition to FREE_DRIVE
        if not enabled:
            if self._current_state.state != AvoidanceState.FREE_DRIVE:
                return AvoidanceState.FREE_DRIVE
            return None
        
        # Get valid transitions from current state
        valid_transitions = self._current_state.get_valid_transitions(context, enabled)
        
        if not valid_transitions:
            return None
        
        # Priority-based state selection
        # EMERGENCY_STOP has highest priority
        if AvoidanceState.EMERGENCY_STOP in valid_transitions:
            return AvoidanceState.EMERGENCY_STOP
        
        # Then SLOW_APPROACH
        if AvoidanceState.SLOW_APPROACH in valid_transitions:
            return AvoidanceState.SLOW_APPROACH
        
        # Then FREE_DRIVE
        if AvoidanceState.FREE_DRIVE in valid_transitions:
            return AvoidanceState.FREE_DRIVE
        
        return None
    
    def _transition_to(self, new_state: AvoidanceState, context: AvoidanceContext):
        """Transition to new state.
        
        Args:
            new_state: Target state
            context: Current avoidance context
        """
        target_state = self._states[new_state]
        
        # Check if transition is allowed
        if not self._current_state.can_exit_to(new_state, context):
            self._logger.debug(
                f"Transition from {self._current_state.name} to {new_state.name} not allowed"
            )
            return
        
        if not target_state.can_enter(context):
            self._logger.debug(
                f"Cannot enter {new_state.name} from current context"
            )
            return
        
        # Perform transition
        previous_state_enum = self._current_state.state
        self._previous_state = previous_state_enum
        
        # Call exit
        self._current_state.on_exit(new_state, context)
        
        # Update state
        self._current_state = target_state
        
        # Call enter
        self._current_state.on_enter(previous_state_enum, context)
        
        self._logger.debug(
            f"Avoidance state transition: {previous_state_enum.name} -> {new_state.name}"
        )
    
    def force_transition_to(self, new_state: AvoidanceState, context: AvoidanceContext):
        """Force transition to new state.
        
        Args:
            new_state: Target state
            context: Current avoidance context
        """
        if new_state not in self._states:
            self._logger.warning(f"Invalid state: {new_state}")
            return
        
        target_state = self._states[new_state]
        previous_state_enum = self._current_state.state
        self._previous_state = previous_state_enum
        
        # Call exit
        self._current_state.on_exit(new_state, context)
        
        # Update state
        self._current_state = target_state
        
        # Call enter
        self._current_state.on_enter(previous_state_enum, context)
        
        self._logger.info(
            f"Forced avoidance state transition: {previous_state_enum.name} -> {new_state.name}"
        )
    
    def force_transition_to_free_drive(self):
        """Force transition to FREE_DRIVE (e.g., when obstacle avoidance is disabled).
        
        This method creates an empty context and forces a transition to FREE_DRIVE.
        """
        if self._current_state.state != AvoidanceState.FREE_DRIVE:
            # Create empty context for transition
            empty_context = AvoidanceContext(
                sectors=[],
                occupancy_grid=None,
                robot_pose=None,
                original_steering=0.0,
                original_speed=0.0
            )
            self.force_transition_to(AvoidanceState.FREE_DRIVE, empty_context)
            self._logger.info("Forced transition to FREE_DRIVE (obstacle avoidance disabled)")

