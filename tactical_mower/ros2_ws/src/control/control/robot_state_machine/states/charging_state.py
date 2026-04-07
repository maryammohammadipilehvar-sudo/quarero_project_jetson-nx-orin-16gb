"""CHARGING state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node

from .base_state import BaseState
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class ChargingState(BaseState):
    """CHARGING state: Actively charging.
    
    In this state, the robot is actively charging at the charging station.
    The robot should remain stationary. The charging relay is enabled.
    It can transition to:
    - DOCKED: When charging relay is disabled (charging stopped)
    - UNDOCKING: When need to undock (autonomous enabled, route active, not need charge)
    - MANUAL: Manual override (always allowed)
    - ERROR: Error state (always allowed)
    
    Note: CHARGING cannot transition directly to UNDOCKED - must go through UNDOCKING.
    
    CHARGING has no automatic transitions - transitions are explicitly triggered
    in the control loop based on conditions.
    
    The charging relay is enabled when entering this state and disabled when exiting.
    This is the ONLY place where the charging relay should be enabled.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.CHARGING,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
        
        # Charging relay publisher (created lazily when node is available)
        self._charging_relay_pub = None
        self._charging_relay_topic = '/control/enable_charging'
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if CHARGING can be entered.
        
        CHARGING can be entered when:
        - Autonomous operation is enabled
        - Robot is docked (at charge position)
        - Route is not active OR need charge
        
        Args:
            context: Current robot context
            
        Returns:
            True if CHARGING can be entered
        """
        if not context.autonomous_operation_enabled:
            return False
        
        # Must be at charge position (docked)
        if not context.is_at_charge_pos:
            return False
        
        # Can charge if route is not active OR need charge
        if not (not context.route_active or context.need_charge):
            return False
        
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        CHARGING can transition to:
        - DOCKED: When charging relay is disabled (charging stopped)
        - UNDOCKING: When autonomous enabled, route active, and not need charge
        - MANUAL: Always allowed
        - ERROR: Always allowed
        
        CHARGING cannot transition directly to UNDOCKED - must go through UNDOCKING.
        
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
        
        # DOCKED: When charging relay is disabled (charging stopped)
        if target_state == RobotState.DOCKED:
            if context.charging_relay_enabled:
                self._logger.info(
                    "CHARGING cannot transition to DOCKED: charging relay still enabled"
                )
                return False
            
            return True
        
        # UNDOCKING: When need to undock (route active OR schedule active, and not need charge)
        # schedule_active=True indicates a schedule wants to start, which requires undocking first
        if target_state == RobotState.UNDOCKING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "CHARGING cannot transition to UNDOCKING: autonomous operation not enabled"
                )
                return False
            
            # Allow transition if route is active OR schedule is active (schedule wants to start)
            if not (context.route_active or context.schedule_active):
                self._logger.info(
                    "CHARGING cannot transition to UNDOCKING: route not active and schedule not active"
                )
                return False
            
            if context.need_charge:
                self._logger.info(
                    "CHARGING cannot transition to UNDOCKING: still need charge"
                )
                return False
            
            return True
        
        # UNDOCKED: Not allowed directly from CHARGING (must go through UNDOCKING)
        if target_state == RobotState.UNDOCKED:
            self._logger.info(
                "CHARGING cannot transition directly to UNDOCKED: must go through UNDOCKING first"
            )
            return False
        
        # Unknown target state
        self._logger.info(
            f"CHARGING cannot transition to {target_state.name}: invalid target state"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from CHARGING.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = {RobotState.MANUAL, RobotState.ERROR}
        
        # Can transition to DOCKED when charging relay is disabled
        if not context.charging_relay_enabled:
            valid.add(RobotState.DOCKED)
        
        # Can transition to UNDOCKING when (route active OR schedule active) and not need charge
        # schedule_active indicates a schedule wants to start, requiring undocking first
        if (context.autonomous_operation_enabled and 
            (context.route_active or context.schedule_active) and 
            not context.need_charge):
            valid.add(RobotState.UNDOCKING)
        
        # Cannot transition directly to UNDOCKED from CHARGING (must go through UNDOCKING)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        CHARGING automatically transitions to UNDOCKING when:
        - Autonomous operation is enabled
        - A schedule is active (schedule_active=True)
        - Robot doesn't need to charge (need_charge=False)
        
        This ensures the robot undocks and starts scheduled routes automatically
        without requiring an explicit undock command.
        
        Args:
            context: Current robot context
            
        Returns:
            UNDOCKING if conditions are met, None otherwise
        """
        # Auto-transition to UNDOCKING when schedule wants to start and battery is sufficient
        if (context.autonomous_operation_enabled and
            context.schedule_active and
            not context.need_charge):
            self._logger.info(
                "CHARGING auto-transition to UNDOCKING: schedule_active=True, need_charge=False"
            )
            return RobotState.UNDOCKING
        
        return None
    
    def _get_charging_relay_publisher(self):
        """Get or create the charging relay publisher."""
        if self._node is None:
            return None
        
        if self._charging_relay_pub is None:
            try:
                from std_msgs.msg import Bool
                self._charging_relay_pub = self._node.create_publisher(
                    Bool,
                    self._charging_relay_topic,
                    10
                )
                self._logger.info(
                    f"Created charging relay publisher on topic {self._charging_relay_topic}"
                )
            except Exception as e:
                self._logger.error(
                    f"Failed to create charging relay publisher: {e}",
                    exc_info=True
                )
                return None
        
        return self._charging_relay_pub
    
    def _enable_charging_relay(self, enable: bool):
        """Enable or disable the charging relay.
        
        This is the ONLY place where the charging relay should be enabled/disabled.
        
        Args:
            enable: True to enable, False to disable
        """
        pub = self._get_charging_relay_publisher()
        if pub is None:
            self._logger.warning(
                "Cannot enable/disable charging relay: publisher not available"
            )
            return
        
        try:
            from std_msgs.msg import Bool
            msg = Bool()
            msg.data = enable
            pub.publish(msg)
            self._logger.info(
                f"Charging relay {'ENABLED' if enable else 'DISABLED'}"
            )
        except Exception as e:
            self._logger.error(
                f"Failed to publish charging relay command: {e}",
                exc_info=True
            )
    
    def on_enter(
        self, 
        previous_state: Optional[RobotState], 
        context: RobotStateContext
    ):
        """Called when entering CHARGING state.
        
        Stops the robot and enables charging relay.
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info("Robot entering CHARGING state - stopping movement")
        
        # Publish stop command if node is available
        if self._node:
            try:
                from interfaces.msg import CommandDrive
                cmd = CommandDrive()
                cmd.left_vel = 0.0
                cmd.right_vel = 0.0
                
                # Try to get the command publisher
                if hasattr(self._node, '_cmd_pub'):
                    self._node._cmd_pub.publish(cmd)
                else:
                    self._logger.info(
                        "CHARGING state: Command publisher not available, "
                        "commands will be handled by node"
                    )
            except Exception as e:
                self._logger.warning(
                    f"Failed to publish stop command in CHARGING state: {e}"
                )
        
        # Enable charging relay (THIS IS THE ONLY PLACE WHERE RELAY IS ENABLED)
        self._enable_charging_relay(True)
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting CHARGING state.
        
        Disables charging relay when leaving CHARGING state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        self._logger.info(f"Exiting CHARGING state, transitioning to {next_state.name}")
        
        # Disable charging relay when leaving CHARGING state
        self._enable_charging_relay(False)
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        In CHARGING state, the robot should remain stationary (no movement).
        Returns zero commands to ensure robot stays stopped.
        
        Args:
            context: Current robot context
            **kwargs: Additional data (e.g., for handling undocking if needed)
            
        Returns:
            Tuple of (0.0, 0.0) to keep robot stopped
        """
        # Keep robot stopped when charging
        return 0.0, 0.0

