"""Obstacle avoidance controller - main interface for obstacle avoidance."""

from typing import Optional, Tuple, Callable, List, Union, TYPE_CHECKING
import logging

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from interfaces.msg import ObstacleSectors, SectorInfo as SectorInfoMsg
from std_msgs.msg import Bool

from .avoidance_state_machine import AvoidanceStateMachine
from .avoidance_context import AvoidanceContext, SectorInfo
from .deadlock_detector import DeadlockDetector
from .avoidance_state import AvoidanceState

if TYPE_CHECKING:
    from rclpy.node import Node


class ObstacleAvoidanceController:
    """Main controller for obstacle avoidance.
    
    This class provides the `apply()` method interface expected by
    navigation_helper.execute_waypoint_following().
    """
    
    def __init__(
        self,
        logger: Optional[logging.Logger],
        speed_reduction_slow: float,
        initial_stop_duration: float = 3.0,
        deadlock_timeout: float = 30.0,
        deadlock_min_distance: float = 0.5,
        deadlock_callback: Optional[Callable[[str], None]] = None,
        node: Optional['Node'] = None,
        initial_enabled: bool = True
    ):
        """Initialize obstacle avoidance controller.
        
        Args:
            logger: Optional logger instance
            speed_reduction_slow: Speed factor for SLOW_APPROACH
            initial_stop_duration: Duration to fully stop (steering=0, speed=0) when entering EMERGENCY_STOP (s)
            deadlock_timeout: Timeout for deadlock detection (s)
            deadlock_min_distance: Minimum distance traveled to avoid deadlock (m)
            deadlock_callback: Optional callback function called when deadlock is detected.
                              Receives warning message as string parameter.
            node: Optional ROS node for topic subscription
            initial_enabled: Initial enabled state (default: True)
        """
        self._logger = logger or logging.getLogger(__name__)
        
        # Single Source of Truth: enabled flag
        self._enabled = initial_enabled
        
        # Initialize state machine
        self._state_machine = AvoidanceStateMachine(
            logger=self._logger,
            speed_reduction_slow=speed_reduction_slow,
            initial_stop_duration=initial_stop_duration
        )
        
        self._deadlock_detector = DeadlockDetector(
            timeout=deadlock_timeout,
            min_distance=deadlock_min_distance
        )
        
        self._deadlock_callback = deadlock_callback
        
        # Simplified deadlock tracking (only 2 variables)
        self._deadlock_target_waypoint: Optional[PoseStamped] = None  # Saved waypoint at deadlock (None = no deadlock)
        self._deadlock_warning_sent: bool = False  # Whether warning has been sent
        
        # Store current position for deadlock detection
        self._last_position: Optional[tuple] = None
        
        # Subscribe to topic if node provided
        if node is not None:
            node.create_subscription(
                Bool,
                '/control/obstacle_avoidance_enabled',
                self._enabled_callback,
                10
            )
            self._logger.info(f"Subscribed to /control/obstacle_avoidance_enabled (initial: {initial_enabled})")
    
    def _enabled_callback(self, msg: Bool):
        """Handle enable/disable topic message.
        
        Args:
            msg: Bool message with enabled state
        """
        self.set_enabled(msg.data)
    
    def set_enabled(self, enabled: bool):
        """Set obstacle avoidance enabled state.
        
        Args:
            enabled: True to enable, False to disable
        """
        if self._enabled != enabled:
            self._enabled = enabled
            self._logger.info(f"Obstacle avoidance {'enabled' if enabled else 'disabled'}")
            
            # Force FREE_DRIVE if disabling
            if not enabled and self._state_machine.get_state() != AvoidanceState.FREE_DRIVE:
                self._state_machine.force_transition_to_free_drive()
    
    def is_enabled(self) -> bool:
        """Get current enabled state.
        
        Returns:
            True if obstacle avoidance is enabled, False otherwise
        """
        return self._enabled
    
    @staticmethod
    def _convert_sectors_to_dataclass(sectors: List[Union[SectorInfoMsg, SectorInfo]]) -> List[SectorInfo]:
        """Convert ROS message sectors to dataclass sectors.
        
        Args:
            sectors: List of SectorInfo ROS messages or dataclass instances
            
        Returns:
            List of SectorInfo dataclass instances
        """
        result = []
        for sector in sectors:
            if isinstance(sector, SectorInfoMsg):
                # Convert ROS message to dataclass
                result.append(SectorInfo(
                    sector_id=sector.sector_id,
                    blocked=sector.blocked,
                    sector_type=sector.sector_type
                ))
            else:
                # Already a dataclass
                result.append(sector)
        return result
    
    def _waypoint_changed(
        self, 
        old_waypoint: Optional[PoseStamped], 
        new_waypoint: Optional[PoseStamped]
    ) -> bool:
        """Check if waypoint has changed (10cm tolerance).
        
        Note: None != not None is considered a change (True).
        
        Args:
            old_waypoint: Previous waypoint (can be None)
            new_waypoint: Current waypoint (can be None)
            
        Returns:
            True if waypoint changed, False otherwise
        """
        # If one is None and the other is not → change
        if (old_waypoint is None) != (new_waypoint is None):
            return True
        
        # If both are None → no change
        if old_waypoint is None and new_waypoint is None:
            return False
        
        # Both present → coordinate comparison
        dx = abs(old_waypoint.pose.position.x - new_waypoint.pose.position.x)
        dy = abs(old_waypoint.pose.position.y - new_waypoint.pose.position.y)
        return (dx > 0.1 or dy > 0.1)  # 10cm tolerance
    
    def apply(
        self,
        steering: float,
        speed: float,
        occupancy_grid: Optional[OccupancyGrid] = None,
        robot_pose: PoseStamped = None,
        obstacle_sectors: Optional[ObstacleSectors] = None,
        next_waypoint: Optional[PoseStamped] = None
    ) -> Tuple[float, float]:
        """Apply obstacle avoidance to steering and speed commands.
        
        This is the main interface method expected by navigation_helper.
        
        Args:
            steering: Original steering command from waypoint follower
            speed: Original speed command from waypoint follower
            occupancy_grid: Current occupancy grid with obstacle data (kept for interface compatibility, not used)
            robot_pose: Current robot pose
            obstacle_sectors: ObstacleSectors message with sector analysis
            next_waypoint: Optional next waypoint from waypoint follower (for deadlock recovery)
            
        Returns:
            Tuple of (modified_steering, modified_speed)
        """
        # Early return if disabled - pass through original commands
        if not self._enabled:
            return steering, speed
        
        # Get sectors from obstacle_sectors message if provided
        if obstacle_sectors is not None:
            sectors = self._convert_sectors_to_dataclass(obstacle_sectors.sectors)
        elif occupancy_grid is not None:
            # Fallback: if no sectors provided, we can't do avoidance
            # This should not happen in normal operation
            self._logger.warning(
                "ObstacleAvoidanceController.apply() called without obstacle_sectors. "
                "Sectors should be provided from /obstacles/sectors topic."
            )
            return steering, speed
        else:
            # No data available
            return steering, speed
        
        # Update deadlock detector
        current_position = None
        if robot_pose is not None:
            current_position = (robot_pose.pose.position.x, robot_pose.pose.position.y)
        current_state_name = self._state_machine.get_state().name
        self._deadlock_detector.update_state(current_state_name, current_position)
        
        # Check for deadlock
        deadlock_detected = self._deadlock_detector.check_deadlock(current_state_name, current_position)
        
        # Implement 5 simple rules (in order):
        
        # Rule 5 (FIRST): Reset when deadlock cleared
        if not deadlock_detected:
            self._deadlock_target_waypoint = None
            self._deadlock_warning_sent = False
        
        # Rule 1: Check if waypoint changed
        waypoint_changed_detected = self._waypoint_changed(
            self._deadlock_target_waypoint,
            next_waypoint
        )
        if self._deadlock_target_waypoint is not None and waypoint_changed_detected:
            # New path planned → reset
            self._deadlock_target_waypoint = None
            self._deadlock_warning_sent = False
            # Continue with normal processing (new chance with new path)
        
        # Rule 2: Deadlock detected → save waypoint
        if deadlock_detected and (waypoint_changed_detected or self._deadlock_target_waypoint is None):
            # Only save if next_waypoint is available
            if next_waypoint is not None:
                self._deadlock_target_waypoint = next_waypoint
                self._deadlock_warning_sent = False  # Reset warning when new waypoint is set
        
        # Create context (with deadlock information for EmergencyStopState)
        context = AvoidanceContext(
            sectors=sectors,
            occupancy_grid=occupancy_grid,
            robot_pose=robot_pose,
            original_steering=steering,
            original_speed=speed,
            next_waypoint=next_waypoint,
            deadlock_detected=deadlock_detected,
            deadlock_target_waypoint=self._deadlock_target_waypoint
        )
        
        # Update state machine and get commands (pass enabled flag)
        modified_steering, modified_speed = self._state_machine.update(context, enabled=self._enabled)
        
        # Rule 4: Send warning (after state machine update)
        if deadlock_detected and self._deadlock_target_waypoint is not None and not self._deadlock_warning_sent:
            # Send warning (only once, when deadlock is still active)
            warning_msg = (
                f"Deadlock: Robot stuck in EMERGENCY_STOP state for "
                f"{self._deadlock_detector.timeout}s with minimal movement. "
                f"Rotation attempted but still blocked. Manual intervention may be required."
            )
            self._logger.warning(warning_msg)
            
            # Call callback if provided (e.g., to send warning to web app)
            if self._deadlock_callback is not None:
                try:
                    self._deadlock_callback(warning_msg)
                except Exception as e:
                    self._logger.error(f"Error in deadlock callback: {e}")
            
            self._deadlock_warning_sent = True
        
        # If state returned None, None, use original commands
        if modified_steering is None and modified_speed is None:
            return steering, speed
        
        # If only one is None, use original for that one
        if modified_steering is None:
            modified_steering = steering
        if modified_speed is None:
            modified_speed = speed
        
        return modified_steering, modified_speed
    
    def get_current_state_name(self) -> str:
        """Get the name of the current avoidance state.
        
        Returns:
            String name of the current state (e.g., "FREE_DRIVE", "SLOW_APPROACH", "EMERGENCY_STOP")
        """
        return self._state_machine.get_state().name

