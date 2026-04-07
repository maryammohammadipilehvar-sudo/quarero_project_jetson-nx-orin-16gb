"""DOCKED state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class DockedState(BaseState):
    """DOCKED state: Successfully docked (physical connection).
    
    In this state, the robot is physically docked at the charging station.
    The robot should remain stationary. It can transition to:
    - CHARGING: When ready to charge (autonomous enabled, route not active or need charge)
    - UNDOCKING: When need to undock (autonomous enabled, route active or not need charge)
    - MANUAL: Manual override (always allowed)
    - ERROR: Error state (always allowed)
    
    DOCKED cannot transition directly to UNDOCKED - must go through UNDOCKING first.
    
    DOCKED has no automatic transitions - transitions are explicitly triggered
    in the control loop based on conditions.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.DOCKED,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if DOCKED can be entered.
        
        DOCKED can be entered when robot is at charge position with RTK fix.
        
        Args:
            context: Current robot context
            
        Returns:
            True if DOCKED can be entered
        """
        # DOCKED can be entered when at charge position with RTK fix
        return context.rtk_fix and context.is_at_charge_pos
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        DOCKED can transition to:
        - CHARGING: If autonomous enabled and (route not active or need charge)
        - UNDOCKING: If autonomous enabled and (route active or not need charge)
        - MANUAL: Always allowed
        - ERROR: Always allowed
        
        DOCKED cannot transition directly to UNDOCKED - must go through UNDOCKING.
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Manual override always allowed
        if target_state == RobotState.MANUAL:
            return True
        
        # Error state always allowed
        if target_state == RobotState.ERROR:
            return True
        
        # CHARGING: Requires autonomous operation and conditions
        if target_state == RobotState.CHARGING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "DOCKED cannot transition to CHARGING: autonomous operation not enabled"
                )
                return False
            
            # Can charge if route is not active OR need charge
            if not (not context.route_active or context.need_charge):
                self._logger.info(
                    "DOCKED cannot transition to CHARGING: route is active and no need to charge"
                )
                return False
            
            return True
        
        # UNDOCKING: Requires autonomous operation and conditions
        if target_state == RobotState.UNDOCKING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "DOCKED cannot transition to UNDOCKING: autonomous operation not enabled"
                )
                return False
            
            # Can undock if route is active OR not need charge
            if not (context.route_active or not context.need_charge):
                self._logger.info(
                    "DOCKED cannot transition to UNDOCKING: route not active and need charge"
                )
                return False
            
            return True
        
        # UNDOCKED: Not allowed directly from DOCKED (must go through UNDOCKING)
        if target_state == RobotState.UNDOCKED:
            self._logger.info(
                "DOCKED cannot transition directly to UNDOCKED: must go through UNDOCKING first"
            )
            return False
        
        # Unknown target state
        self._logger.info(
            f"DOCKED cannot transition to {target_state.name}: invalid target state"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from DOCKED.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = {RobotState.MANUAL, RobotState.ERROR}
        
        if context.autonomous_operation_enabled:
            # Can transition to CHARGING if route not active or need charge
            if not context.route_active or context.need_charge:
                valid.add(RobotState.CHARGING)
            
            # Can transition to UNDOCKING if route active or not need charge
            if context.route_active or not context.need_charge:
                valid.add(RobotState.UNDOCKING)
        
        # Cannot transition directly to UNDOCKED from DOCKED (must go through UNDOCKING)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        DOCKED automatically transitions to CHARGING when:
        - Autonomous operation is enabled
        - Robot is at charge position
        - Route is not active OR need charge
        
        Args:
            context: Current robot context
            
        Returns:
            CHARGING state if conditions are met, None otherwise
        """
        # Automatic transition to CHARGING when conditions are met
        if context.autonomous_operation_enabled:
            if context.is_at_charge_pos:
                # Can charge if route is not active OR need charge
                if not context.route_active or context.need_charge:
                    return RobotState.CHARGING
        
        return None
    
    def on_enter(
        self, 
        previous_state: Optional[RobotState], 
        context: RobotStateContext
    ):
        """Called when entering DOCKED state.
        
        Stops the robot when docked.
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info("Robot successfully docked - stopping movement")
        
        # Publish stop command if node is available
        if self._node:
            try:
                from interfaces.msg import CommandDrive
                cmd = CommandDrive()
                cmd.left_vel = 0.0
                cmd.right_vel = 0.0
                
                # Try to get the command publisher
                # This assumes the publisher exists, otherwise we'll log a warning
                if hasattr(self._node, '_cmd_pub'):
                    self._node._cmd_pub.publish(cmd)
                else:
                    self._logger.info(
                        "DOCKED state: Command publisher not available, "
                        "commands will be handled by node"
                    )
            except Exception as e:
                self._logger.warning(
                    f"Failed to publish stop command in DOCKED state: {e}"
                )
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting DOCKED state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        self._logger.info(f"Exiting DOCKED state, transitioning to {next_state.name}")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        In DOCKED state, the robot should remain stationary (no movement).
        Returns zero commands to ensure robot stays stopped.
        
        Args:
            context: Current robot context
            **kwargs: Additional data
            
        Returns:
            Tuple of (0.0, 0.0) to keep robot stopped
        """
        # Keep robot stopped when docked
        return 0.0, 0.0

