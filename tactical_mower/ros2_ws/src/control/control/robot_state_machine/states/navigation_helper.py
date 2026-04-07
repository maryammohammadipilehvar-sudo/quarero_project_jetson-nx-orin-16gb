"""Shared navigation helper functions for waypoint-following states."""

from typing import Optional, Tuple, Callable, TYPE_CHECKING, Any

if TYPE_CHECKING:
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from interfaces.msg import ObstacleSectors


def execute_waypoint_following(
    waypoint_follower: Any,  # WaypointFollower from navigation.waypoint_follower
    robot_pose_map: 'PoseStamped',
    on_waypoint_reached: Optional[Callable[[int], None]] = None,
    on_route_completed: Optional[Callable[[], None]] = None,
    obstacle_avoidance: Optional[Any] = None,  # Obstacle avoidance controller/state machine
    obstacle_sectors: Optional['ObstacleSectors'] = None  # Obstacle sectors with sector analysis
) -> Tuple[Optional[float], Optional[float]]:
    """Execute waypoint following and return steering/speed commands.
    
    This helper function encapsulates the common waypoint following logic
    used by both NAVIGATING and RETURNING_TO_HOME states.
    
    Optionally applies obstacle avoidance to modify the commands based on
    detected obstacles in the environment.
    
    Args:
        waypoint_follower: WaypointFollower instance
        robot_pose_map: Current robot pose (in map frame)
        on_waypoint_reached: Optional callback when waypoint is reached (idx)
        on_route_completed: Optional callback when route is completed
        obstacle_avoidance: Optional obstacle avoidance controller/state machine.
            Must implement an `apply(steering, speed, occupancy_grid, robot_pose, obstacle_sectors)` method
            that returns (modified_steering, modified_speed).
        obstacle_sectors: Optional ObstacleSectors message with sector analysis
        
    Returns:
        Tuple of (steering, speed) commands, or (None, None) if navigation should stop
    """
    if not waypoint_follower.is_active():
        return None, None
    
    # Update waypoint follower (pure algorithm, no ROS dependencies)
    steering, continue_nav, speed = waypoint_follower.update(
        robot_pose_map,
        on_waypoint_reached=on_waypoint_reached,
        on_route_completed=on_route_completed
    )
    
    if not continue_nav:
        # Navigation stopped - return zero commands
        return 0.0, 0.0
    
    if steering is not None:
        # Use returned speed (default 100) with computed steering
        spd = speed if speed is not None else 100.0
        
        # Apply obstacle avoidance if provided
        if obstacle_avoidance is not None and obstacle_sectors is not None:
            try:
                # Get next waypoint from waypoint follower (for deadlock recovery)
                next_waypoint = None
                if hasattr(waypoint_follower, 'get_next_waypoint'):
                    next_waypoint = waypoint_follower.get_next_waypoint(robot_pose_map)
                
                # Call obstacle avoidance with sectors and next waypoint
                modified_steering, modified_speed = obstacle_avoidance.apply(
                    steering, spd, 
                    occupancy_grid=None,  # Not needed when sectors are provided
                    robot_pose=robot_pose_map,
                    obstacle_sectors=obstacle_sectors,
                    next_waypoint=next_waypoint
                )
                return modified_steering, modified_speed
            except Exception:
                # If obstacle avoidance fails, fall back to original commands
                # Logging should be done by the caller if needed
                pass
        
        return steering, spd
    
    # Steering is None - don't publish commands (let manual control work)
    return None, None

