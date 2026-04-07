"""Sector analyzer for obstacle detection based on robot geometry."""

import math
from typing import List, Dict, Any
from nav_msgs.msg import OccupancyGrid
from interfaces.msg import SectorInfo


class SectorDefinition:
    """Definition of a single sector."""
    
    def __init__(self, sector_id: int, name: str, x_min: float, x_max: float, 
                 y_min: float, y_max: float, priority: int, sector_type: str):
        """Initialize sector definition.
        
        Args:
            sector_id: Unique identifier for this sector
            name: Human-readable name
            x_min: Minimum x coordinate [m]
            x_max: Maximum x coordinate [m]
            y_min: Minimum y coordinate [m]
            y_max: Maximum y coordinate [m]
            priority: Priority for overlap resolution (higher = checked first)
            sector_type: STOP or SLOW
        """
        self.sector_id = sector_id
        self.name = name
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.priority = priority
        self.sector_type = sector_type
    
    def contains_point(self, x: float, y: float) -> bool:
        """Check if point (x, y) is within this sector."""
        return (self.x_min <= x <= self.x_max and 
                self.y_min <= y <= self.y_max)


class SectorAnalyzer:
    """Analyzes OccupancyGrid by dividing space into configurable rectangular sectors."""
    
    def __init__(
        self,
        sectors_config: List[Dict[str, Any]],
        max_range: float = 3.0
    ):
        """Initialize sector analyzer.
        
        Args:
            sectors_config: List of sector definitions from config, each containing:
                - name: string
                - x_min: float [m]
                - x_max: float [m]
                - y_min: float [m]
                - y_max: float [m]
                - priority: int (higher = checked first)
                - sector_type: string ("STOP" or "SLOW")
            max_range: Maximum range to analyze [m]
        """
        self.max_range = max_range
        
        # Parse sector definitions from config
        self.sectors: List[SectorDefinition] = []
        for idx, sector_cfg in enumerate(sectors_config):
            sector = SectorDefinition(
                sector_id=idx,
                name=str(sector_cfg.get('name', f'sector_{idx}')),
                x_min=float(sector_cfg.get('x_min', 0.0)),
                x_max=float(sector_cfg.get('x_max', 0.0)),
                y_min=float(sector_cfg.get('y_min', 0.0)),
                y_max=float(sector_cfg.get('y_max', 0.0)),
                priority=int(sector_cfg.get('priority', idx)),
                sector_type=str(sector_cfg.get('sector_type', 'SLOW'))
            )
            self.sectors.append(sector)
        
        # Create a sorted list for overlap resolution (by priority, highest first)
        # But keep original order for sector_id mapping
        self.sectors_sorted_by_priority = sorted(self.sectors, key=lambda s: s.priority, reverse=True)
    
    def _get_sector_for_point(self, x: float, y: float) -> int:
        """Determine which sector a point belongs to.
        
        Returns sector ID, or -1 if point doesn't belong to any sector.
        Uses priority to resolve overlaps (higher priority sectors checked first).
        """
        for sector in self.sectors_sorted_by_priority:
            if sector.contains_point(x, y):
                return sector.sector_id
        return -1
    
    def analyze(self, occupancy_grid: OccupancyGrid) -> List[SectorInfo]:
        """Analyze occupancy grid and return sector information.
        
        Args:
            occupancy_grid: OccupancyGrid message to analyze
            
        Returns:
            List of SectorInfo ROS messages, one per configured sector
        """
        resolution = occupancy_grid.info.resolution
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y
        
        num_sectors = len(self.sectors)
        
        # Initialize sectors
        sectors: List[SectorInfo] = []
        for sector_def in self.sectors:
            sector = SectorInfo()
            sector.sector_id = sector_def.sector_id
            sector.blocked = False
            sector.sector_type = sector_def.sector_type  # From config only
            sectors.append(sector)
        
        # Statistics per sector
        total_cells_per_sector = [0] * num_sectors
        occupied_cells_per_sector = [0] * num_sectors
        
        # Analyze each cell in the grid
        for y in range(height):
            for x in range(width):
                # Convert grid index to world coordinates (robot frame)
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                
                # Skip if outside distance range
                distance = math.hypot(world_x, world_y)
                if distance < 0.1 or distance > self.max_range:
                    continue
                
                # Determine which sector this point belongs to
                sector_idx = self._get_sector_for_point(world_x, world_y)
                if sector_idx < 0:
                    continue  # Point doesn't belong to any sector
                
                total_cells_per_sector[sector_idx] += 1
                
                # Check if cell is occupied
                idx = y * width + x
                if 0 <= idx < len(occupancy_grid.data) and occupancy_grid.data[idx] >= 50:
                    occupied_cells_per_sector[sector_idx] += 1
        
        # Set blocked status for each sector
        for i, sector_def in enumerate(self.sectors):
            sector_id = sector_def.sector_id
            sector_info = sectors[i]
            
            # Sector is blocked if any obstacle detected
            sector_info.blocked = occupied_cells_per_sector[sector_id] > 0
        
        return sectors

