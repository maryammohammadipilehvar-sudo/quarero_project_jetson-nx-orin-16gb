"""State machine for robot operational states using State pattern."""

from typing import Optional, Callable, TYPE_CHECKING
from dataclasses import asdict
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .robot_state import RobotState
from .robot_state_context import RobotStateContext
from .state_factory import StateFactory
from .states.base_state import BaseState


class RobotStateMachine:
    """State machine for robot operational states using State pattern.
    
    This state machine uses the State pattern where each state is represented
    by its own class. This provides better organization, maintainability, and
    testability compared to a monolithic state machine implementation.
    
    States have access to a ROS node for ROS operations (publishers, subscribers, etc.).
    """
    
    def __init__(self, node: Optional['Node'] = None, logger: Optional[logging.Logger] = None):
        """Initialize robot state machine.
        
        Args:
            node: ROS node instance to pass to states
            logger: Optional logger instance. If None, uses node's logger or creates a new one.
        """
        self._node = node
        
        # Use node's logger if available, otherwise create one
        if logger is not None:
            self._logger = logger
        elif node is not None:
            self._logger = node.get_logger()
        else:
            self._logger = logging.getLogger(__name__)
        
        self._current_state: BaseState = StateFactory.get_state(
            RobotState.UNINITIALIZED,
            node=node,
            logger=self._logger.getChild("states") if node is None else None
        )
        self._previous_state: Optional[RobotState] = None
        self._entry_callbacks: dict[RobotState, list[Callable]] = {}
        self._exit_callbacks: dict[RobotState, list[Callable]] = {}
        
        self._logger.info(f"State machine initialized in state: {self._current_state.name}")
    
    def get_state(self) -> RobotState:
        """Get current state.
        
        Returns:
            Current RobotState enum value
        """
        return self._current_state.state
    
    def get_previous_state(self) -> Optional[RobotState]:
        """Get previous state.
        
        Returns:
            Previous RobotState or None
        """
        return self._previous_state
    
    def get_state_name(self) -> str:
        """Get current state name.
        
        Returns:
            Current state name as string
        """
        return self._current_state.name
    
    def is_state(self, state: RobotState) -> bool:
        """Check if currently in specific state.
        
        Args:
            state: State to check
            
        Returns:
            True if in specified state
        """
        return self._current_state.state == state
    
    def can_transition_to(self, new_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to new state is allowed.
        
        Args:
            new_state: Target state
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Manual override: if autonomous operation is disabled, must be in MANUAL
        if not context.autonomous_operation_enabled:
            if new_state == RobotState.MANUAL:
                self._logger.debug(f"Autonomous operation disabled, allowing transition to MANUAL")
                return True
            # If not transitioning to MANUAL, check if we're already in MANUAL
            if self._current_state.state != RobotState.MANUAL:
                self._logger.debug(
                    f"Autonomous operation disabled, cannot transition to {new_state.name} "
                    f"(must be in MANUAL)"
                )
                return False
        
        # Error state special handling
        if self._current_state.state == RobotState.ERROR:
            allowed = new_state in [RobotState.IDLE, RobotState.MANUAL]
            if not allowed:
                self._logger.debug(f"ERROR state can only transition to IDLE or MANUAL, not {new_state.name}")
            return allowed
        
        # Delegate to current state
        can_exit = self._current_state.can_exit_to(new_state, context)
        if not can_exit:
            self._logger.debug(
                f"Cannot exit {self._current_state.name} to {new_state.name} "
                f"(can_exit_to returned False)"
            )
            return False
        
        # Check if target state can be entered
        target_state = StateFactory.get_state(
            new_state,
            node=self._node,
            logger=self._logger.getChild("states") if self._node is None else None
        )
        can_enter = target_state.can_enter(context)
        if not can_enter:
            self._logger.debug(
                f"Cannot enter {new_state.name} from {self._current_state.name} "
                f"(can_enter returned False)"
            )
            return False
        
        return True
    
    def _get_transition_rejection_reason(self, new_state: RobotState, context: RobotStateContext) -> str:
        """Get detailed reason why a transition was rejected.
        
        Args:
            new_state: Target state that was rejected
            context: Current robot context
            
        Returns:
            Human-readable reason for rejection
        """
        # Check autonomous operation first
        if not context.autonomous_operation_enabled:
            if new_state != RobotState.MANUAL:
                return f"autonomous_operation_enabled=False, can only go to MANUAL"
        
        # Check ERROR state restrictions
        if self._current_state.state == RobotState.ERROR:
            if new_state not in [RobotState.IDLE, RobotState.MANUAL]:
                return "ERROR state can only transition to IDLE or MANUAL"
        
        # Check if current state can exit
        can_exit = self._current_state.can_exit_to(new_state, context)
        if not can_exit:
            return f"current state {self._current_state.name} cannot exit to {new_state.name} (context: {asdict(context)})"
        
        # Check if target state can be entered
        target_state = StateFactory.get_state(
            new_state,
            node=self._node,
            logger=self._logger.getChild("states") if self._node is None else None
        )
        can_enter = target_state.can_enter(context)
        if not can_enter:
            return f"target state {new_state.name} cannot be entered (context: {asdict(context)})"
        
        return "unknown reason"
    
    def transition_to(self, new_state: RobotState, context: RobotStateContext) -> bool:
        """Transition to new state if allowed.
        
        Args:
            new_state: Target state
            context: Current robot context
            
        Returns:
            True if transition was successful
        """
        if not self.can_transition_to(new_state, context):
            # Build detailed reason for rejection
            reason = self._get_transition_rejection_reason(new_state, context)
            self._logger.warning(
                f"Transition from {self._current_state.name} to {new_state.name} not allowed: {reason}"
            )
            return False
        
        # Get target state instance
        target_state = StateFactory.get_state(
            new_state,
            node=self._node,
            logger=self._logger.getChild("states") if self._node is None else None
        )
        
        # Call exit callbacks (registered callbacks)
        self._call_exit_callbacks(self._current_state.state, new_state)
        
        # Call state's on_exit
        self._current_state.on_exit(new_state, context)
        
        # Update state
        previous_state_enum = self._current_state.state
        self._previous_state = previous_state_enum
        self._current_state = target_state
        
        # Call state's on_enter
        target_state.on_enter(previous_state_enum, context)
        
        # Call entry callbacks (registered callbacks)
        self._call_entry_callbacks(previous_state_enum, new_state)
        
        self._logger.info(
            f"State transition: {previous_state_enum.name} -> {new_state.name}"
        )
        
        return True
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on current context (auto-transitions).
        
        This method implements automatic state transitions based on context.
        Manual override has highest priority.
        
        Args:
            context: Current robot context
            
        Returns:
            Next state to transition to, or None to stay in current state
        """
        # Manual override has highest priority (using autonomous_operation_enabled)
        # If autonomous operation is disabled, must be in MANUAL state
        if not context.autonomous_operation_enabled:
            if self._current_state.state == RobotState.ERROR:
                return None
            if self._current_state.state != RobotState.MANUAL:
                self._logger.debug("Autonomous operation disabled, transitioning to MANUAL")
                return RobotState.MANUAL
            return None
        
        # Error state: can only exit via manual or explicit reset
        if self._current_state.state == RobotState.ERROR:
            return None
        
        # Delegate to current state
        next_state = self._current_state.determine_next_state(context)
        if next_state is not None:
            self._logger.debug(
                f"State {self._current_state.name} determined next state: {next_state.name}"
            )
        return next_state
    
    def update(self, context: RobotStateContext, **kwargs):
        """Update the state machine (call in control loop).
        
        This method should be called repeatedly in the control loop. It:
        1. Calls the current state's on_update method
        2. Checks for automatic transitions
        
        Args:
            context: Current robot context
            **kwargs: Additional data to pass to state's on_update (e.g., robot_pose, callbacks)
            
        Returns:
            Tuple of (steering, speed) commands from state, or None if no commands
        """
        # Call state's update method
        commands = None
        try:
            commands = self._current_state.on_update(context, **kwargs)
        except Exception as e:
            self._logger.error(
                f"Error in state {self._current_state.name}.on_update: {e}",
                exc_info=True
            )
        
        # Check for automatic transitions
        next_state = self.determine_next_state(context)
        if next_state is not None:
            self.transition_to(next_state, context)
        
        return commands
    
    def register_state_entry_callback(self, state: RobotState, callback: Callable):
        """Register callback for state entry.
        
        Args:
            state: State to register callback for
            callback: Callback function(state_from, state_to)
        """
        if state not in self._entry_callbacks:
            self._entry_callbacks[state] = []
        self._entry_callbacks[state].append(callback)
        self._logger.debug(f"Registered entry callback for state {state.name}")
    
    def register_state_exit_callback(self, state: RobotState, callback: Callable):
        """Register callback for state exit.
        
        Args:
            state: State to register callback for
            callback: Callback function(state_from, state_to)
        """
        if state not in self._exit_callbacks:
            self._exit_callbacks[state] = []
        self._exit_callbacks[state].append(callback)
        self._logger.debug(f"Registered exit callback for state {state.name}")
    
    def _call_entry_callbacks(self, from_state: RobotState, to_state: RobotState):
        """Call registered entry callbacks.
        
        Args:
            from_state: Previous state
            to_state: New state
        """
        if to_state in self._entry_callbacks:
            for callback in self._entry_callbacks[to_state]:
                try:
                    callback(from_state, to_state)
                except Exception as e:
                    self._logger.error(
                        f"Error in entry callback for {to_state.name}: {e}",
                        exc_info=True
                    )
    
    def _call_exit_callbacks(self, from_state: RobotState, to_state: RobotState):
        """Call registered exit callbacks.
        
        Args:
            from_state: Previous state
            to_state: New state
        """
        if from_state in self._exit_callbacks:
            for callback in self._exit_callbacks[from_state]:
                try:
                    callback(from_state, to_state)
                except Exception as e:
                    self._logger.error(
                        f"Error in exit callback for {from_state.name}: {e}",
                        exc_info=True
                    )
    
    def set_error(self):
        """Set error state (bypasses normal transition checks)."""
        self._logger.warning(f"Setting error state from {self._current_state.name}")
        self._previous_state = self._current_state.state
        self._current_state = StateFactory.get_state(
            RobotState.ERROR,
            node=self._node,
            logger=self._logger.getChild("states") if self._node is None else None
        )
    
    def reset(self):
        """Reset state machine to IDLE."""
        self._logger.info(f"Resetting state machine from {self._current_state.name} to IDLE")
        self._previous_state = self._current_state.state
        self._current_state = StateFactory.get_state(
            RobotState.IDLE,
            node=self._node,
            logger=self._logger.getChild("states") if self._node is None else None
        )
    
    def to_dict(self) -> dict:
        """Serialize state to dict for persistence.
        
        Returns:
            Dictionary with state information
        """
        return {
            'state': int(self._current_state.state),
            'state_name': self._current_state.name,
            'previous_state': int(self._previous_state) if self._previous_state else None
        }
    
    @classmethod
    def from_dict(
        cls, 
        data: dict, 
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ) -> 'RobotStateMachine':
        """Create RobotStateMachine from serialized dict.
        
        Args:
            data: Dictionary with state information
            node: ROS node instance to pass to states
            logger: Optional logger instance
            
        Returns:
            RobotStateMachine instance
        """
        machine = cls(node=node, logger=logger)
        try:
            state_int = int(data.get('state', RobotState.IDLE))
            machine._current_state = StateFactory.get_state(
                RobotState(state_int),
                node=machine._node,
                logger=machine._logger.getChild("states") if machine._node is None else None
            )
            prev_state_int = data.get('previous_state')
            if prev_state_int is not None:
                machine._previous_state = RobotState(int(prev_state_int))
            machine._logger.info(
                f"State machine restored from dict: {machine._current_state.name}"
            )
        except Exception as e:
            machine._logger.warning(f"Failed to restore state from dict: {e}, using defaults")
        return machine

