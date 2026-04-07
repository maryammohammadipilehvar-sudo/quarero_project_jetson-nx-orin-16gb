"""Context information for obstacle avoidance state machine."""

from dataclasses import dataclass
from typing import List, Optional

if False:  # TYPE_CHECKING
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseStamped


@dataclass
class SectorInfo:
    """Information about a sector in obstacle analysis."""
    sector_id: int
    blocked: bool  # True if sector is blocked by obstacles, False otherwise
    sector_type: str  # STOP or SLOW - indicates required action type


@dataclass
class AvoidanceContext:
    """Context information for avoidance state machine decisions.
    
    This dataclass contains all the information needed by avoidance states
    to make transition decisions and execute their behavior.
    """
    sectors: List[SectorInfo]  # Sector analysis results
    occupancy_grid: Optional['OccupancyGrid'] = None  # Current occupancy grid
    robot_pose: Optional['PoseStamped'] = None  # Current robot pose
    original_steering: Optional[float] = None  # Original steering command from waypoint follower
    original_speed: Optional[float] = None  # Original speed command from waypoint follower
    next_waypoint: Optional['PoseStamped'] = None  # Next waypoint from waypoint follower
    deadlock_detected: bool = False  # Whether deadlock is currently detected
    deadlock_target_waypoint: Optional['PoseStamped'] = None  # Target waypoint for deadlock recovery rotation


def has_blocked_stop_sectors(context: AvoidanceContext) -> bool:
    """Check if any STOP sector is blocked.
    
    Args:
        context: Current avoidance context
        
    Returns:
        True if at least one STOP sector is blocked, False otherwise
    """
    return any(sector.blocked and sector.sector_type == "STOP" for sector in context.sectors)


def has_blocked_slow_sectors(context: AvoidanceContext) -> bool:
    """Check if any SLOW sector is blocked.
    
    Args:
        context: Current avoidance context
        
    Returns:
        True if at least one SLOW sector is blocked, False otherwise
    """
    return any(sector.blocked and sector.sector_type == "SLOW" for sector in context.sectors)


def has_any_blocked_sectors(context: AvoidanceContext) -> bool:
    """Check if any sector is blocked.
    
    Args:
        context: Current avoidance context
        
    Returns:
        True if at least one sector is blocked, False otherwise
    """
    return any(sector.blocked for sector in context.sectors)

