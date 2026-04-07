#!/usr/bin/env python3
"""
Grid utilities for occupancy grid operations.

This module provides utility functions for working with occupancy grids,
including validation, goal checking, and grid extension.
"""

import math
from typing import Optional, Tuple
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from rclpy.node import Node
from rclpy.time import Time, Duration
from tf2_ros import Buffer, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


def validate_grid(occupancy_grid: OccupancyGrid) -> bool:
    """
    Validate occupancy grid data.
    
    Args:
        occupancy_grid: The occupancy grid to validate
        
    Returns:
        True if grid is valid, False otherwise
    """
    if occupancy_grid is None:
        return False
    
    if occupancy_grid.info.width <= 0 or occupancy_grid.info.height <= 0:
        return False
    
    if occupancy_grid.info.resolution <= 0.0:
        return False
    
    expected_size = occupancy_grid.info.width * occupancy_grid.info.height
    if len(occupancy_grid.data) != expected_size:
        return False
    
    return True


def quaternion_to_yaw(quat: Quaternion) -> float:
    """
    Convert quaternion to yaw angle (rotation around z-axis).
    
    Args:
        quat: Quaternion
        
    Returns:
        Yaw angle in radians
    """
    # Extract yaw from quaternion
    # For a quaternion (x, y, z, w), yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    yaw = math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    )
    return yaw


def world_to_grid_coords(
    world_x: float,
    world_y: float,
    grid: OccupancyGrid
) -> Tuple[float, float]:
    """
    Transform world coordinates to grid coordinates, accounting for grid rotation.
    
    The grid's origin.orientation defines the rotation of the grid coordinate system
    in the world frame. This function applies the inverse rotation to transform
    world coordinates into grid-local coordinates.
    
    Args:
        world_x: X coordinate in world frame
        world_y: Y coordinate in world frame
        grid: The occupancy grid
        
    Returns:
        Tuple (grid_x, grid_y) as float coordinates (can be converted to cell indices)
    """
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    resolution = grid.info.resolution
    
    # Get rotation angle from quaternion
    yaw = quaternion_to_yaw(grid.info.origin.orientation)
    
    # Translate to origin
    dx = world_x - origin_x
    dy = world_y - origin_y
    
    # Apply inverse rotation (rotate by -yaw)
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    grid_x = dx * cos_yaw - dy * sin_yaw
    grid_y = dx * sin_yaw + dy * cos_yaw
    
    # Convert to cell coordinates
    grid_x = grid_x / resolution
    grid_y = grid_y / resolution
    
    return (grid_x, grid_y)


def grid_to_world_coords(
    grid_x: float,
    grid_y: float,
    grid: OccupancyGrid
) -> Tuple[float, float]:
    """
    Transform grid coordinates to world coordinates, accounting for grid rotation.
    
    Args:
        grid_x: X coordinate in grid frame (cell index or float)
        grid_y: Y coordinate in grid frame (cell index or float)
        grid: The occupancy grid
        
    Returns:
        Tuple (world_x, world_y) in world coordinates
    """
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    resolution = grid.info.resolution
    
    # Convert from cell coordinates to grid-local meters
    local_x = grid_x * resolution
    local_y = grid_y * resolution
    
    # Get rotation angle from quaternion
    yaw = quaternion_to_yaw(grid.info.origin.orientation)
    
    # Apply rotation
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = local_x * cos_yaw - local_y * sin_yaw
    world_y = local_x * sin_yaw + local_y * cos_yaw
    
    # Translate to world origin
    world_x = world_x + origin_x
    world_y = world_y + origin_y
    
    return (world_x, world_y)


def is_goal_in_grid(occupancy_grid: OccupancyGrid, goal_pose: PoseStamped) -> bool:
    """
    Check if goal pose is within the occupancy grid boundaries.
    
    Args:
        occupancy_grid: The occupancy grid
        goal_pose: The goal pose to check
        
    Returns:
        True if goal is within grid, False otherwise
    """
    if not validate_grid(occupancy_grid):
        return False
    
    # Get grid origin and dimensions
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    resolution = occupancy_grid.info.resolution
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    
    # Get goal position
    goal_x = goal_pose.pose.position.x
    goal_y = goal_pose.pose.position.y
    
    # Transform to grid coordinates (accounting for rotation)
    grid_x, grid_y = world_to_grid_coords(goal_x, goal_y, occupancy_grid)
    
    if grid_x < 0 or grid_x >= width:
        return False
    if grid_y < 0 or grid_y >= height:
        return False
    
    return True


def extend_grid_for_goal(
    occupancy_grid: OccupancyGrid,
    goal_pose: PoseStamped,
    extension_distance: float,
    resolution: float,
    node: Optional[Node] = None
) -> OccupancyGrid:
    """
    Extend occupancy grid to include goal pose if it's outside the grid.
    
    The extended areas are marked as unknown (-1).
    
    Args:
        occupancy_grid: The original occupancy grid
        goal_pose: The goal pose (may be outside grid)
        extension_distance: Distance to extend grid in meters
        resolution: Resolution for extended areas (should match or be compatible)
        node: Optional ROS2 Node reference for debug logging
        
    Returns:
        Extended occupancy grid with goal included
    """
    if not validate_grid(occupancy_grid):
        raise ValueError("Invalid occupancy grid provided")
    
    if goal_pose.header.frame_id != occupancy_grid.header.frame_id:
        raise ValueError(
            f"Frame mismatch: goal_pose is in frame '{goal_pose.header.frame_id}', "
            f"but grid is in frame '{occupancy_grid.header.frame_id}'. "
            f"Both must be in the same frame for grid extension."
        )
    
    if is_goal_in_grid(occupancy_grid, goal_pose):
        return occupancy_grid
    
    # Get original grid properties
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    orig_resolution = occupancy_grid.info.resolution
    orig_width = occupancy_grid.info.width
    orig_height = occupancy_grid.info.height

    # Use provided resolution or original resolution
    if resolution <= 0.0:
        resolution = orig_resolution
    
    # Get goal position
    goal_x = goal_pose.pose.position.x
    goal_y = goal_pose.pose.position.y
    
    # Transform goal to grid coordinates (in cell indices)
    goal_grid_x, goal_grid_y = world_to_grid_coords(goal_x, goal_y, occupancy_grid)
    
    # Convert to grid-local coordinates in meters
    # The grid extends from (0, 0) to (width*resolution, height*resolution) in grid-local coords
    goal_local_x = goal_grid_x * orig_resolution
    goal_local_y = goal_grid_y * orig_resolution
    orig_max_local_x = orig_width * orig_resolution
    orig_max_local_y = orig_height * orig_resolution
    
    # Calculate required grid bounds to include goal (in grid-local coordinates in meters)
    min_local_x = min(0.0, goal_local_x - extension_distance)
    max_local_x = max(orig_max_local_x, goal_local_x + extension_distance)
    min_local_y = min(0.0, goal_local_y - extension_distance)
    max_local_y = max(orig_max_local_y, goal_local_y + extension_distance)
    
    # Calculate new grid dimensions (in grid-local coordinates, keeping origin aligned)
    new_width = int(math.ceil((max_local_x - min_local_x) / resolution))
    new_height = int(math.ceil((max_local_y - min_local_y) / resolution))
    
    # Adjust origin to account for extension in negative directions
    # We need to shift the origin in grid-local coordinates
    origin_shift_x = min_local_x
    origin_shift_y = min_local_y
    
    # Convert origin shift to world coordinates
    yaw = quaternion_to_yaw(occupancy_grid.info.origin.orientation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_shift_x = origin_shift_x * cos_yaw - origin_shift_y * sin_yaw
    world_shift_y = origin_shift_x * sin_yaw + origin_shift_y * cos_yaw
    
    new_origin_x = origin_x + world_shift_x
    new_origin_y = origin_y + world_shift_y
    
    # Create extended grid
    extended_grid = OccupancyGrid()
    # Copy header but ensure frame_id matches
    extended_grid.header.frame_id = occupancy_grid.header.frame_id
    extended_grid.header.stamp = occupancy_grid.header.stamp  # Will be updated by publisher
    extended_grid.info.resolution = resolution
    extended_grid.info.width = new_width
    extended_grid.info.height = new_height
    extended_grid.info.origin.position.x = new_origin_x
    extended_grid.info.origin.position.y = new_origin_y
    extended_grid.info.origin.position.z = occupancy_grid.info.origin.position.z
    extended_grid.info.origin.orientation = occupancy_grid.info.origin.orientation
    
    # Initialize with unknown values (-1)
    extended_grid.data = [-1] * (new_width * new_height)
    
    # Copy original grid data to extended grid
    # We need to map each cell from the original grid to the new grid
    for orig_y in range(orig_height):
        for orig_x in range(orig_width):
            # Calculate world position of original cell center
            # First, get grid-local position of cell center
            orig_local_x = (orig_x + 0.5) * orig_resolution
            orig_local_y = (orig_y + 0.5) * orig_resolution
            
            # Transform to world coordinates using original grid's origin and rotation
            orig_world_x, orig_world_y = grid_to_world_coords(
                orig_x + 0.5, orig_y + 0.5, occupancy_grid
            )
            
            # Transform to new grid coordinates
            new_grid_x, new_grid_y = world_to_grid_coords(
                orig_world_x, orig_world_y, extended_grid
            )
            
            # Convert to cell indices (world_to_grid_coords returns cell coordinates as float)
            new_x = int(new_grid_x)
            new_y = int(new_grid_y)
            
            # Check bounds
            if 0 <= new_x < new_width and 0 <= new_y < new_height:
                # Get original data value
                orig_index = orig_y * orig_width + orig_x
                orig_value = occupancy_grid.data[orig_index]
                
                # Set in extended grid
                new_index = new_y * new_width + new_x
                extended_grid.data[new_index] = orig_value
    
    return extended_grid


def transform_pose_to_grid_frame(pose: PoseStamped, grid: OccupancyGrid) -> tuple:
    """
    Transform pose to grid coordinate system (cell indices).
    
    This function accounts for the grid's rotation as defined by origin.orientation.
    
    Args:
        pose: The pose to transform
        grid: The occupancy grid
        
    Returns:
        Tuple (grid_x, grid_y) as cell indices, or None if invalid
    """
    if not validate_grid(grid):
        return None
    
    pose_x = pose.pose.position.x
    pose_y = pose.pose.position.y
    
    # Transform to grid coordinates (accounting for rotation)
    grid_x, grid_y = world_to_grid_coords(pose_x, pose_y, grid)
    
    # Convert to integer cell indices
    grid_x = int(grid_x)
    grid_y = int(grid_y)
    
    return (grid_x, grid_y)


def world_to_grid_coords_int(pose: PoseStamped, grid: OccupancyGrid) -> Optional[Tuple[int, int]]:
    """
    Convert world coordinates to grid indices, accounting for grid rotation.
    
    The grid's origin.orientation defines the rotation of the grid coordinate system
    in the world frame. This function applies the inverse rotation to transform
    world coordinates into grid-local coordinates.
    
    Args:
        pose: Pose in world coordinates (must be in grid's frame)
        grid: Occupancy grid
        
    Returns:
        (grid_x, grid_y) as integers, or None if out of bounds
    """
    if grid is None:
        return None
    
    # Get grid origin and resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    resolution = grid.info.resolution
    
    # Get world coordinates
    world_x = pose.pose.position.x
    world_y = pose.pose.position.y
    
    # Get rotation angle from quaternion
    yaw = quaternion_to_yaw(grid.info.origin.orientation)
    
    # Translate to origin
    dx = world_x - origin_x
    dy = world_y - origin_y
    
    # Apply inverse rotation (rotate by -yaw)
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    grid_local_x = dx * cos_yaw - dy * sin_yaw
    grid_local_y = dx * sin_yaw + dy * cos_yaw
    
    # Convert to cell coordinates
    grid_x = int(grid_local_x / resolution)
    grid_y = int(grid_local_y / resolution)
    
    # Check bounds
    if grid_x < 0 or grid_x >= grid.info.width:
        return None
    if grid_y < 0 or grid_y >= grid.info.height:
        return None
    
    return (grid_x, grid_y)


def is_cell_occupied(grid: OccupancyGrid, grid_x: int, grid_y: int) -> bool:
    """
    Check if a grid cell is occupied.
    
    Args:
        grid: Occupancy grid
        grid_x, grid_y: Grid cell indices
        
    Returns:
        True if cell is occupied (value > 50), False otherwise
    """
    if grid is None:
        return False
    
    if grid_x < 0 or grid_x >= grid.info.width:
        return False
    if grid_y < 0 or grid_y >= grid.info.height:
        return False
    
    index = grid_y * grid.info.width + grid_x
    if index >= len(grid.data):
        return False
    
    return grid.data[index] > 50


def _clip_line_to_grid_bounds(
    x0: float, y0: float, 
    x1: float, y1: float, 
    width: int, height: int
) -> Optional[Tuple[int, int, int, int]]:
    """
    Clip a line segment to grid bounds using Cohen-Sutherland algorithm.
    
    Args:
        x0, y0: Start point (can be float, will be converted to int after clipping)
        x1, y1: End point (can be float, will be converted to int after clipping)
        width, height: Grid dimensions
        
    Returns:
        Tuple (x0, y0, x1, y1) of clipped integer coordinates, or None if line is completely outside
    """
    # Region codes for Cohen-Sutherland
    INSIDE = 0
    LEFT = 1
    RIGHT = 2
    BOTTOM = 4
    TOP = 8
    
    def compute_code(x: float, y: float) -> int:
        code = INSIDE
        if x < 0:
            code |= LEFT
        elif x >= width:
            code |= RIGHT
        if y < 0:
            code |= BOTTOM
        elif y >= height:
            code |= TOP
        return code
    
    code0 = compute_code(x0, y0)
    code1 = compute_code(x1, y1)
    
    max_iterations = 20  # Prevent infinite loops
    iterations = 0
    
    while iterations < max_iterations:
        iterations += 1
        
        if code0 == 0 and code1 == 0:
            # Both points inside
            return (int(x0), int(y0), int(x1), int(y1))
        
        if code0 & code1 != 0:
            # Both points on same side, line completely outside
            return None
        
        # Pick point outside the rectangle
        code_out = code0 if code0 != 0 else code1
        
        # Find intersection point
        if code_out & TOP:
            x = x0 + (x1 - x0) * (height - 1 - y0) / (y1 - y0) if y1 != y0 else x0
            y = height - 1
        elif code_out & BOTTOM:
            x = x0 + (x1 - x0) * (0 - y0) / (y1 - y0) if y1 != y0 else x0
            y = 0
        elif code_out & RIGHT:
            y = y0 + (y1 - y0) * (width - 1 - x0) / (x1 - x0) if x1 != x0 else y0
            x = width - 1
        elif code_out & LEFT:
            y = y0 + (y1 - y0) * (0 - x0) / (x1 - x0) if x1 != x0 else y0
            x = 0
        else:
            return None
        
        # Replace outside point
        if code_out == code0:
            x0, y0 = x, y
            code0 = compute_code(x0, y0)
        else:
            x1, y1 = x, y
            code1 = compute_code(x1, y1)
    
    # Max iterations reached, return None to be safe
    return None


def check_line_collision(
    grid: OccupancyGrid, 
    start_pose: PoseStamped, 
    end_pose: PoseStamped, 
    debug: bool = False
) -> Tuple[bool, bool]:
    """
    Check if a line between two poses collides with obstacles, clipping to grid bounds.
    
    This function clips the line segment to the grid boundaries before checking,
    allowing collision detection even when one or both endpoints are outside the grid.
    Uses Bresenham's line algorithm to check all cells along the (clipped) line.
    
    Args:
        grid: Occupancy grid
        start_pose: Start pose (must be in grid's frame)
        end_pose: End pose (must be in grid's frame)
        debug: Enable debug logging
        
    Returns:
        Tuple (collision_detected, was_checked):
        - collision_detected: True if collision detected
        - was_checked: True if the segment (or part of it) was actually checked
    """
    if grid is None:
        return False, False
    
    # Convert to grid coordinates (float for clipping)
    start_grid = world_to_grid_coords(
        start_pose.pose.position.x,
        start_pose.pose.position.y,
        grid
    )
    end_grid = world_to_grid_coords(
        end_pose.pose.position.x,
        end_pose.pose.position.y,
        grid
    )
    
    # Clip line to grid bounds
    clipped = _clip_line_to_grid_bounds(
        start_grid[0], start_grid[1],
        end_grid[0], end_grid[1],
        grid.info.width, grid.info.height
    )
    
    if clipped is None:
        # Line completely outside grid
        if debug:
            print(f"[LINE_COLLISION] Line completely outside grid bounds")
        return False, False
    
    x0, y0, x1, y1 = clipped
    
    if debug:
        print(f"[LINE_COLLISION] Checking clipped segment: ({x0}, {y0}) -> ({x1}, {y1})")
    
    # If start and end are the same, just check that cell
    if x0 == x1 and y0 == y1:
        occupied = is_cell_occupied(grid, x0, y0)
        if debug:
            cell_value = grid.data[y0 * grid.info.width + x0] if (y0 * grid.info.width + x0) < len(grid.data) else -1
            print(f"[LINE_COLLISION] Same cell ({x0}, {y0}): value={cell_value}, occupied={occupied}")
        return occupied, True
    
    # Bresenham's line algorithm
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    
    x, y = x0, y0
    cells_checked = 0
    
    while True:
        occupied = is_cell_occupied(grid, x, y)
        cells_checked += 1
        
        if debug and cells_checked <= 10:
            cell_value = grid.data[y * grid.info.width + x] if (y * grid.info.width + x) < len(grid.data) else -1
            print(f"[LINE_COLLISION] Cell ({x}, {y}): value={cell_value}, occupied={occupied}")
        
        if occupied:
            if debug:
                print(f"[LINE_COLLISION] COLLISION at ({x}, {y}) after {cells_checked} cells")
            return True, True
        
        if x == x1 and y == y1:
            break
        
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    
    if debug:
        print(f"[LINE_COLLISION] No collision after {cells_checked} cells")
    
    return False, True


def transform_occupancy_grid(
    occupancy_grid: OccupancyGrid,
    target_frame: str,
    tf_buffer: Buffer,
    node: Optional[Node] = None,
    tf_timeout: float = 1.0
) -> OccupancyGrid:
    """
    Transform an occupancy grid from its current frame to a target frame.
    
    This function transforms the grid's origin position and orientation to the target frame.
    The grid data itself remains unchanged (cells are relative to the origin).
    
    Args:
        occupancy_grid: The occupancy grid to transform
        target_frame: Target frame ID (e.g., 'odom')
        tf_buffer: TF2 buffer for looking up transformations
        node: Optional ROS2 node for logging
        tf_timeout: Timeout for TF lookups in seconds
        
    Returns:
        Transformed occupancy grid with updated frame_id and origin
        
    Raises:
        TransformException: If transform lookup fails
        ValueError: If grid is invalid
    """
    if not validate_grid(occupancy_grid):
        raise ValueError("Invalid occupancy grid provided")
    
    # If already in target frame, return copy
    if occupancy_grid.header.frame_id == target_frame:
        # Return a copy to avoid modifying the original
        transformed_grid = OccupancyGrid()
        transformed_grid.header = occupancy_grid.header
        transformed_grid.info = occupancy_grid.info
        transformed_grid.data = list(occupancy_grid.data)
        return transformed_grid
    
    # Get the transform from grid frame to target frame
    timeout_duration = Duration(seconds=tf_timeout)
    
    # For static transforms (like base_footprint, map), always use Time(0) to get latest available
    # Static transforms are published once with an old timestamp, but are always valid
    # Using Time(0) ensures we get the latest available transform regardless of timestamp
    transform_time = Time()
    use_latest = True
    
    # Try to get transform with better error reporting
    # First check if frames are available in TF tree
    transform = None
    try:
        if node:
            node.get_logger().debug(
                f'[FRAME] Looking up transform: {occupancy_grid.header.frame_id} → {target_frame} '
                f'(timeout={tf_timeout}s)'
            )
            # Check if frames can be resolved (this helps debug frame existence issues)
            try:
                # Try to see if we can resolve the transform chain
                can_transform = tf_buffer.can_transform(
                    target_frame,
                    occupancy_grid.header.frame_id,
                    transform_time if not use_latest else Time(),
                    timeout=timeout_duration
                )
                if not can_transform:
                    if node:
                        node.get_logger().warn(
                            f'[FRAME] Transform not available: {occupancy_grid.header.frame_id} → {target_frame}. '
                            f'Frames may not be connected in TF tree or transform is too old.'
                        )
            except Exception as check_e:
                if node:
                    node.get_logger().debug(
                        f'[FRAME] Can-transform check failed (non-critical): {check_e}'
                    )
        
        transform = tf_buffer.lookup_transform(
            target_frame,
            occupancy_grid.header.frame_id,
            transform_time,
            timeout=timeout_duration
        )
        if node:
            node.get_logger().debug(
                f'[FRAME] Transform lookup successful (exact timestamp)'
            )
    except TransformException as e1:
        # If exact timestamp fails, try latest available transform
        if not use_latest:
            if node:
                node.get_logger().debug(
                    f'[FRAME] Exact timestamp lookup failed: {e1}. '
                    f'Trying latest available transform...'
                )
            try:
                transform = tf_buffer.lookup_transform(
                    target_frame,
                    occupancy_grid.header.frame_id,
                    Time(),
                    timeout=timeout_duration
                )
                if node:
                    node.get_logger().debug(
                        f'[FRAME] Transform lookup successful (latest available)'
                    )
            except TransformException as e2:
                # Both attempts failed
                error_msg = (
                    f'Failed to lookup transform from {occupancy_grid.header.frame_id} to {target_frame}. '
                    f'Exact timestamp error: {e1}. Latest transform error: {e2}. '
                    f'Check if TF tree is properly configured and frames exist.'
                )
                if node:
                    node.get_logger().error(f'[FRAME] {error_msg}')
                raise TransformException(error_msg) from e2
        else:
            # Already tried latest, so this is the final error
            error_msg = (
                f'Failed to lookup transform from {occupancy_grid.header.frame_id} to {target_frame}: {e1}. '
                f'Check if TF tree is properly configured and frames exist.'
            )
            if node:
                node.get_logger().error(f'[FRAME] {error_msg}')
            raise TransformException(error_msg) from e1
    
    # Create a pose for the grid origin
    origin_pose = Pose()
    origin_pose.position = occupancy_grid.info.origin.position
    origin_pose.orientation = occupancy_grid.info.origin.orientation
    
    # Create a PoseStamped for transformation (do_transform_pose expects PoseStamped, not Pose)
    from geometry_msgs.msg import PoseStamped
    origin_pose_stamped = PoseStamped()
    origin_pose_stamped.header.frame_id = occupancy_grid.header.frame_id
    origin_pose_stamped.header.stamp = occupancy_grid.header.stamp
    origin_pose_stamped.pose = origin_pose
    
    # Transform the origin pose using the full TransformStamped
    # do_transform_pose expects (PoseStamped, TransformStamped) and returns PoseStamped
    transformed_origin_pose_stamped = do_transform_pose_stamped(origin_pose_stamped, transform)
    
    # Create transformed grid
    transformed_grid = OccupancyGrid()
    transformed_grid.header.stamp = transform.header.stamp
    transformed_grid.header.frame_id = target_frame
    
    # Copy grid metadata
    transformed_grid.info.resolution = occupancy_grid.info.resolution
    transformed_grid.info.width = occupancy_grid.info.width
    transformed_grid.info.height = occupancy_grid.info.height
    
    # Set transformed origin (access pose from PoseStamped)
    transformed_grid.info.origin.position = transformed_origin_pose_stamped.pose.position
    transformed_grid.info.origin.orientation = transformed_origin_pose_stamped.pose.orientation
    
    # Copy grid data (unchanged, as cells are relative to origin)
    transformed_grid.data = list(occupancy_grid.data)
    
    if node:
        # Log transformation details
        origin_before = occupancy_grid.info.origin.position
        origin_after = transformed_grid.info.origin.position
        node.get_logger().debug(
            f'[FRAME] Grid transformation complete: '
            f'{occupancy_grid.header.frame_id} → {target_frame}. '
            f'Origin: ({origin_before.x:.3f}, {origin_before.y:.3f}) → '
            f'({origin_after.x:.3f}, {origin_after.y:.3f})'
        )
    
    return transformed_grid

