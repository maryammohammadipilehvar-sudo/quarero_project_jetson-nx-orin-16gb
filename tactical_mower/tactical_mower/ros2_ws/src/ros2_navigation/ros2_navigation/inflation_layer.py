#!/usr/bin/env python3
"""
Inflation Layer for inflating obstacles in occupancy grids.

This module provides functionality to inflate obstacles in occupancy grids
to create a safety buffer around obstacles, preventing the robot from planning
paths too close to obstacles.
"""

import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from ros2_navigation.grid_utils import validate_grid

# Try to import scipy for faster dilation (optional optimization)
try:
    from scipy.ndimage import binary_dilation
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class InflationLayer:
    """
    Inflates obstacles in occupancy grids to create safety buffers.
    
    The inflation algorithm expands occupied cells (obstacles) by a specified
    radius, marking nearby cells as occupied to ensure paths maintain a safe
    distance from obstacles.
    """
    
    # Cache for pre-computed inflation kernels (key: (inflation_cells, resolution, inflation_radius))
    _kernel_cache = {}
    
    @staticmethod
    def _create_circular_kernel(inflation_cells: int, resolution: float, inflation_radius: float) -> np.ndarray:
        """
        Create a circular kernel for inflation.
        
        Args:
            inflation_cells: Inflation radius in cells
            resolution: Grid resolution in meters
            inflation_radius: Inflation radius in meters
            
        Returns:
            Binary kernel array (True = inflate, False = don't inflate)
        """
        kernel_size = 2 * inflation_cells + 1
        center = inflation_cells
        
        # Pre-compute squared inflation radius to avoid sqrt in loop
        inflation_radius_sq = inflation_radius * inflation_radius
        
        # Create circular mask using vectorized operations
        y_coords, x_coords = np.ogrid[:kernel_size, :kernel_size]
        dx = (x_coords - center) * resolution
        dy = (y_coords - center) * resolution
        distances_sq = dx * dx + dy * dy
        kernel = distances_sq <= inflation_radius_sq
        
        return kernel
    
    @staticmethod
    def _get_kernel(inflation_cells: int, resolution: float, inflation_radius: float) -> np.ndarray:
        """
        Get or create inflation kernel (with caching).
        
        Args:
            inflation_cells: Inflation radius in cells
            resolution: Grid resolution in meters
            inflation_radius: Inflation radius in meters
            
        Returns:
            Binary kernel array
        """
        cache_key = (inflation_cells, resolution, inflation_radius)
        if cache_key not in InflationLayer._kernel_cache:
            InflationLayer._kernel_cache[cache_key] = InflationLayer._create_circular_kernel(
                inflation_cells, resolution, inflation_radius
            )
        return InflationLayer._kernel_cache[cache_key]
    
    @staticmethod
    def inflate_grid(occupancy_grid: OccupancyGrid, inflation_radius: float) -> OccupancyGrid:
        """
        Inflate obstacles in an occupancy grid.
        
        Optimized version using pre-computed kernels and vectorized operations.
        
        Args:
            occupancy_grid: Original occupancy grid to inflate
            inflation_radius: Radius in meters to inflate obstacles
            
        Returns:
            New OccupancyGrid with inflated obstacles
            
        Raises:
            ValueError: If grid is invalid or inflation_radius is negative
        """
        if not validate_grid(occupancy_grid):
            raise ValueError("Invalid occupancy grid provided")
        
        if inflation_radius < 0.0:
            raise ValueError(f"Inflation radius must be non-negative, got {inflation_radius}")
        
        # If inflation radius is zero, return original grid
        if inflation_radius == 0.0:
            return occupancy_grid
        
        # Get grid properties
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        
        # Convert inflation radius to grid cells
        inflation_cells = int(math.ceil(inflation_radius / resolution))
        
        # Convert grid data to numpy array for efficient processing
        grid_array = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))
        
        # Create binary mask for occupied cells (value > 50)
        occupied_mask = grid_array > 50
        
        # If no occupied cells, return original grid
        if not np.any(occupied_mask):
            inflated_grid = OccupancyGrid()
            inflated_grid.header = occupancy_grid.header
            inflated_grid.info = occupancy_grid.info
            inflated_grid.data = grid_array.flatten().tolist()
            return inflated_grid
        
        # OPTIMIZATION 1: Use scipy's binary_dilation if available (much faster)
        if SCIPY_AVAILABLE:
            # Get or create kernel
            kernel = InflationLayer._get_kernel(inflation_cells, resolution, inflation_radius)
            
            # Dilate occupied cells
            inflated_mask = binary_dilation(occupied_mask, structure=kernel)
            
            # Create inflated array: preserve original values, but mark inflated free cells as occupied
            inflated_array = grid_array.copy()
            # Only inflate free space (value 0), preserve unknown (-1) and original occupied cells
            free_space_mask = (grid_array == 0) & inflated_mask
            inflated_array[free_space_mask] = 100
        else:
            # OPTIMIZATION 2: Fallback to optimized manual inflation
            # Get or create kernel
            kernel = InflationLayer._get_kernel(inflation_cells, resolution, inflation_radius)
            
            # Create inflated array
            inflated_array = grid_array.copy()
            
            # Find all occupied cells
            occupied_indices = np.where(occupied_mask)
            
            # Apply kernel to each occupied cell using vectorized operations
            kernel_center = inflation_cells
            
            for y, x in zip(occupied_indices[0], occupied_indices[1]):
                # Calculate bounds for kernel application
                min_y = max(0, y - kernel_center)
                max_y = min(height, y + kernel_center + 1)
                min_x = max(0, x - kernel_center)
                max_x = min(width, x + kernel_center + 1)
                
                # Calculate kernel bounds (may be cropped at grid edges)
                kernel_min_y = kernel_center - (y - min_y)
                kernel_max_y = kernel_center + (max_y - y)
                kernel_min_x = kernel_center - (x - min_x)
                kernel_max_x = kernel_center + (max_x - x)
                
                # Extract kernel region
                kernel_region = kernel[kernel_min_y:kernel_max_y, kernel_min_x:kernel_max_x]
                
                # Apply kernel: only inflate free space (value 0)
                grid_region = inflated_array[min_y:max_y, min_x:max_x]
                free_space_region = (grid_region == 0)
                inflate_region = kernel_region & free_space_region
                grid_region[inflate_region] = 100
        
        # Create new OccupancyGrid with inflated data
        inflated_grid = OccupancyGrid()
        inflated_grid.header = occupancy_grid.header
        inflated_grid.info = occupancy_grid.info
        inflated_grid.data = inflated_array.flatten().tolist()
        
        return inflated_grid

