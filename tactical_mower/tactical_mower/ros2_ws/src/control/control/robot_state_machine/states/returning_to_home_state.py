"""RETURNING_TO_HOME state implementation."""

from typing import Optional, Set, Tuple, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

from .base_state import BaseState
from .navigation_helper import execute_waypoint_following
from ..robot_state import RobotState
from ..robot_state_context import RobotStateContext


class ReturningToHomeState(BaseState):
    """RETURNING_TO_HOME state: Returning to home position.
    
    In this state, the robot returns to the home position using a reverse route.
    It uses the WaypointFollower to follow waypoints backwards along the route.
    When the robot reaches the home position, it transitions to DOCKING.
    """
    
    def __init__(
        self, 
        state: RobotState = RobotState.RETURNING_TO_HOME,
        node: Optional['Node'] = None,
        logger: Optional[logging.Logger] = None
    ):
        super().__init__(state, node=node, logger=logger)
    
    def can_enter(self, context: RobotStateContext) -> bool:
        """Check if RETURNING_TO_HOME can be entered.
        
        RETURNING_TO_HOME can be entered when:
        - Autonomous operation is enabled
        - Not docked
        - (Route not active OR need charge)
        - RTK fix is available
        - Sensor info is valid
        
        Args:
            context: Current robot context
            
        Returns:
            True if RETURNING_TO_HOME can be entered
        """
        if not context.autonomous_operation_enabled:
            return False
        
        if context.docked:
            return False
        
        if not (not context.route_active or context.need_charge):
            return False
        
        if not context.rtk_fix:
            return False
        
        if not context.sensor_info_valid:
            return False
        
        return True
    
    def can_exit_to(self, target_state: RobotState, context: RobotStateContext) -> bool:
        """Check if transition to target state is allowed.
        
        RETURNING_TO_HOME can transition to:
        - DOCKING: When at home position
        - MANUAL: Always allowed
        - IDLE: Always allowed
        
        Args:
            target_state: Target state to transition to
            context: Current robot context
            
        Returns:
            True if transition is allowed
        """
        # Manual override always allowed
        if target_state == RobotState.MANUAL:
            return True
        
        # IDLE always allowed
        if target_state == RobotState.IDLE:
            return True
        
        # DOCKING: When at/near home position
        # Use both is_at_home_pos and is_near_home_pos to handle GPS jitter
        if target_state == RobotState.DOCKING:
            if not context.autonomous_operation_enabled:
                self._logger.info(
                    "RETURNING_TO_HOME cannot transition to DOCKING: autonomous operation not enabled"
                )
                return False
            
            if not context.is_at_home_pos and not context.is_near_home_pos:
                self._logger.info(
                    "RETURNING_TO_HOME cannot transition to DOCKING: not at/near home position"
                )
                return False
            
            return True
        
        # Unknown target state
        self._logger.info(
            f"RETURNING_TO_HOME cannot transition to {target_state.name}: invalid target state"
        )
        return False
    
    def get_valid_transitions(self, context: RobotStateContext) -> Set[RobotState]:
        """Get valid transitions from RETURNING_TO_HOME.
        
        Args:
            context: Current robot context
            
        Returns:
            Set of valid target states
        """
        valid = {RobotState.MANUAL, RobotState.IDLE}
        
        # Can transition to DOCKING when at/near home position
        # Use both is_at_home_pos (0.5m) and is_near_home_pos (0.6m) to handle GPS jitter
        if context.autonomous_operation_enabled and (context.is_at_home_pos or context.is_near_home_pos):
            valid.add(RobotState.DOCKING)
        
        return valid
    
    def determine_next_state(self, context: RobotStateContext) -> Optional[RobotState]:
        """Determine next state based on context (auto-transitions).
        
        RETURNING_TO_HOME automatically transitions to DOCKING when at home position.
        
        Args:
            context: Current robot context
            
        Returns:
            DOCKING if at home position, None otherwise
        """
        # Auto-transition to DOCKING when at/near home position
        # Use both is_at_home_pos (0.5m) and is_near_home_pos (0.6m) to handle GPS jitter
        # The GPS position can fluctuate by 10-20cm, causing is_at_home_pos to flip
        if context.autonomous_operation_enabled and (context.is_at_home_pos or context.is_near_home_pos):
            return RobotState.DOCKING
        
        return None
    
    def on_enter(
        self, 
        previous_state: Optional[RobotState], 
        context: RobotStateContext
    ):
        """Called when entering RETURNING_TO_HOME state.
        
        Args:
            previous_state: Previous state
            context: Current robot context
        """
        super().on_enter(previous_state, context)
        self._logger.info("Entering RETURNING_TO_HOME state - returning to home position")
        
        # Safety check: warn if robot is already at home position
        # This should not happen normally - indicates a race condition or logic error
        if context.is_at_home_pos or context.is_near_home_pos:
            self._logger.warning(
                f"RETURNING_TO_HOME entered but robot is already at/near home position! "
                f"This may indicate a state machine race condition. "
                f"(previous_state={previous_state.name if previous_state else 'None'}, "
                f"at_home={context.is_at_home_pos}, near_home={context.is_near_home_pos}, "
                f"route_active={context.route_active}, schedule_active={context.schedule_active})"
            )
    
    def on_exit(self, next_state: RobotState, context: RobotStateContext):
        """Called when exiting RETURNING_TO_HOME state.
        
        Args:
            next_state: Next state being transitioned to
            context: Current robot context
        """
        super().on_exit(next_state, context)
        self._logger.info(f"Exiting RETURNING_TO_HOME state, transitioning to {next_state.name}")
    
    def on_update(
        self, 
        context: RobotStateContext,
        **kwargs
    ) -> Optional[Tuple[Optional[float], Optional[float]]]:
        """Called during state update (called in control loop).
        
        Executes waypoint following using the WaypointFollower (for reverse route).
        
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
            self._logger.warning("RETURNING_TO_HOME: robot_pose not provided in kwargs")
            return None, None
        
        waypoint_follower = kwargs.get('waypoint_follower')
        if waypoint_follower is None:
            self._logger.warning("RETURNING_TO_HOME: waypoint_follower not provided in kwargs")
            return None, None
        
        # Callbacks
        on_route_completed = kwargs.get('on_route_completed')
        on_waypoint_reached = kwargs.get('on_waypoint_reached')
        
        # Optional obstacle avoidance
        obstacle_avoidance = kwargs.get('obstacle_avoidance')
        obstacle_sectors = kwargs.get('obstacle_sectors')
        
        # Execute waypoint following using shared helper
        # The waypoint_follower should be configured with reverse route by the node
        steering, speed = execute_waypoint_following(
            waypoint_follower,
            robot_pose_map,
            on_waypoint_reached=on_waypoint_reached,
            on_route_completed=on_route_completed,
            obstacle_avoidance=obstacle_avoidance,
            obstacle_sectors=obstacle_sectors
        )
        
        return steering, speed
