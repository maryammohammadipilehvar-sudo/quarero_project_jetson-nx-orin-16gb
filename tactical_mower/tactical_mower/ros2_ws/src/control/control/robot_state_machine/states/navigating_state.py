"""NAVIGATING state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging
import time

if TYPE_CHECKING:
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

from .base_state import BaseState
from .navigation_helper import execute_waypoint_following
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class NavigatingState(BaseState):
    """NAVIGATING state: Following waypoints.
    
    In this state, the robot follows waypoints along an active route.
    It uses the WaypointFollower to generate steering and speed commands.
    When the route is completed, it transitions to IDLE.
    """
    
    # Grace period after entering NAVIGATING state before allowing transition to RETURNING_TO_HOME
    # This prevents race conditions where route_active might be False briefly during state transition
    GRACE_PERIOD_SECONDS = 6.0
    
    def __init__(
        self, 
        state: RobotState = RobotState.NAVIGATING,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
        self._entry_time: Optional[float] = None
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if NAVIGATING can be entered.
        
        NAVIGATING can be entered when:
        - Autonomous operation is enabled
        - Route is active
        - Schedule is active
        - RTK fix is available
        - Sensor info is valid
        - Not need charge (if need charge, should return home instead)
        
        Args:
            context: Current robot context
            
        Returns:
            True if NAVIGATING can be entered
        """
        if not context.autonomous_operation_enabled:
            return False
        
        if not context.route_active:
            return False
        
        if not context.schedule_active:
            return False
        
        if not context.rtk_fix:
            return False
        
        if not context.sensor_info_valid:
            return False
        
        if context.need_charge:
            return False
        
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        NAVIGATING can transition to:
        - RETURNING_TO_HOME: When need charge or route inactive
        - MANUAL: Always allowed
        - IDLE: Always allowed (when route completes)
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Manual override always allowed
        if target_state == RobotState.MANUAL:
            return True
        
        # IDLE always allowed (route completion)
        if target_state == RobotState.IDLE:
            return True
        
        # RETURNING_TO_HOME: When need charge or route inactive
        if target_state == RobotState.RETURNING_TO_HOME:
            if not context.autonomous_operation_enabled:
                return False
            
            if not (context.need_charge or not context.route_active):
                return False
            
            if not context.rtk_fix:
                return False
            
            if not context.sensor_info_valid:
                return False
            
            return True
        
        # Unknown target state
        self._logger.info(
            f"NAVIGATING cannot transition to {target_state.name}: invalid target state"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from NAVIGATING.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = {RobotState.MANUAL, RobotState.IDLE}
        
        # Can transition to RETURNING_TO_HOME if need charge or route inactive
        if (context.autonomous_operation_enabled and
            (context.need_charge or not context.route_active) and
            context.rtk_fix and
            context.sensor_info_valid):
            valid.add(RobotState.RETURNING_TO_HOME)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        NAVIGATING automatically transitions to RETURNING_TO_HOME when:
        - Need charge OR route inactive
        - All other conditions for RETURNING_TO_HOME are met
        - Grace period has elapsed (prevents race conditions)
        
        Args:
            context: Current robot context
            
        Returns:
            RETURNING_TO_HOME if conditions met, None otherwise
        """
        # Check if grace period has elapsed
        current_time = time.monotonic()
        if self._entry_time is not None:
            time_since_entry = current_time - self._entry_time
            if time_since_entry < self.GRACE_PERIOD_SECONDS:
                # Still in grace period - don't transition based on route_active
                # Only allow transition if need_charge (which is more critical)
                if (context.autonomous_operation_enabled and
                    context.need_charge and  # Only allow charge-based transition during grace period
                    context.rtk_fix and
                    context.sensor_info_valid):
                    self._logger.info(
                        f"NAVIGATING: Need charge during grace period, transitioning to RETURNING_TO_HOME "
                        f"(time_since_entry={time_since_entry:.2f}s < {self.GRACE_PERIOD_SECONDS}s)"
                    )
                    return RobotState.RETURNING_TO_HOME
                # During grace period, ignore route_active=False to prevent race conditions
                return None
        
        # Grace period has elapsed - normal transition logic
        # Auto-transition to RETURNING_TO_HOME when need charge or route inactive
        if (context.autonomous_operation_enabled and
            (context.need_charge or not context.route_active) and
            context.rtk_fix and
            context.sensor_info_valid):
            # Only transition if route is truly inactive (not just a brief state change)
            # If route_active is False, we should transition, but log it for debugging
            if not context.route_active:
                self._logger.info(
                    f"NAVIGATING: Route inactive, transitioning to RETURNING_TO_HOME "
                    f"(route_active={context.route_active}, need_charge={context.need_charge})"
                )
            return RobotState.RETURNING_TO_HOME
        
        return None
    
    def on_enter(
        self, 
        previous_state: Optional[RobotState], 
        context: RobotStateContext
    ):
        """Called when entering NAVIGATING state.
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        # Record entry time for grace period
        self._entry_time = time.monotonic()
        self._logger.info("Entering NAVIGATING state - following waypoints")
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting NAVIGATING state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        # Reset entry time when exiting
        self._entry_time = None
        self._logger.info(f"Exiting NAVIGATING state, transitioning to {next_state.name}")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        Executes waypoint following using the WaypointFollower.
        
        Expected kwargs:
            - robot_pose: PoseStamped - Current robot pose
            - waypoint_follower: WaypointFollower - Waypoint follower instance
            - on_route_completed: Optional[Callable] - Callback when route completes
            - on_waypoint_reached: Optional[Callable[[int]]] - Callback when waypoint reached
            - obstacle_avoidance: Optional[Any] - Obstacle avoidance controller/state machine
            - occupancy_grid: Optional[OccupancyGrid] - Current obstacle grid data
        
        Args:
            context: Current robot context
            **kwargs: Additional data (robot_pose, waypoint_follower, etc.)
            
        Returns:
            Tuple of (steering, speed) commands, or (None, None) if no commands
            are generated by this state.
        """
        # Get required parameters from kwargs
        robot_pose_map = kwargs.get('robot_pose')
        if robot_pose_map is None:
            self._logger.warning("NAVIGATING: robot_pose not provided in kwargs")
            return None, None
        
        waypoint_follower = kwargs.get('waypoint_follower')
        if waypoint_follower is None:
            self._logger.warning("NAVIGATING: waypoint_follower not provided in kwargs")
            return None, None
        
        # Callbacks
        on_route_completed = kwargs.get('on_route_completed')
        on_waypoint_reached = kwargs.get('on_waypoint_reached')
        
        # Optional obstacle avoidance
        obstacle_avoidance = kwargs.get('obstacle_avoidance')
        obstacle_sectors = kwargs.get('obstacle_sectors')
        
        # Execute waypoint following using shared helper
        steering, speed = execute_waypoint_following(
            waypoint_follower,
            robot_pose_map,
            on_waypoint_reached=on_waypoint_reached,
            on_route_completed=on_route_completed,
            obstacle_avoidance=obstacle_avoidance,
            obstacle_sectors=obstacle_sectors
        )
        
        return steering, speed
