#!/usr/bin/env python3
"""
Replanning Service for checking path collisions and replanning if needed.

This module implements the service handler for ReplanPath,
which checks if a path has collisions and replans if necessary.
"""

import time
from typing import Tuple
from rclpy.node import Node
from ros2_navigation.srv import ReplanPath
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from ros2_navigation.tf_manager import TFManager
from ros2_navigation.path_planning_service import PathPlanningService
from ros2_navigation.occupancy_grid_subscriber import OccupancyGridSubscriber
from ros2_navigation.grid_utils import (
    validate_grid,
    world_to_grid_coords_int,
    is_cell_occupied,
    check_line_collision,
    extend_grid_for_goal,
    is_goal_in_grid
)


class ReplanningService:
    """
    Service handler for path replanning requests.
    """
    
    def __init__(
        self,
        node: Node,
        tf_manager: TFManager,
        path_planning_service: PathPlanningService,
        grid_subscriber: OccupancyGridSubscriber,
        service_name: str = 'replan_path',
        tf_timeout: float = 1.0,
        grid_extension_distance: float = 5.0,
        grid_extension_resolution: float = 0.1
    ):
        """
        Initialize Replanning Service.
        
        Args:
            node: ROS2 node instance
            tf_manager: TF Manager for pose transformations
            path_planning_service: Path Planning Service for replanning
            grid_subscriber: OccupancyGrid Subscriber
            service_name: Name of the replanning service
            tf_timeout: Timeout for TF lookups in seconds
            grid_extension_distance: Distance to extend grid if waypoints are outside (meters)
            grid_extension_resolution: Resolution for extended grid areas (meters)
        """
        self.node = node
        self.tf_manager = tf_manager
        self.path_planning_service = path_planning_service
        self.grid_subscriber = grid_subscriber
        self.tf_timeout = tf_timeout
        self.grid_extension_distance = grid_extension_distance
        self.grid_extension_resolution = grid_extension_resolution
        
        # Create service server
        self.service = self.node.create_service(
            ReplanPath,
            service_name,
            self.replan_path_callback
        )
        
        self.node.get_logger().info(
            f'ReplanningService initialized, service: {service_name}'
        )
    
    def replan_path_callback(self, request, response):
        """
        Service callback for replanning requests.
        
        Checks the path for collisions and replans if necessary.
        
        Args:
            request: ReplanPath request
            response: ReplanPath response
            
        Returns:
            Response with collision status and new path if replanned
        """
        total_start_time = time.time()
        
        self.node.get_logger().info(
            f'[REPLAN REQUEST] Path: {len(request.path.poses)} waypoints, '
            f'start_index={request.start_waypoint_index}, '
            f'goal=({request.goal_pose.pose.position.x:.3f}, {request.goal_pose.pose.position.y:.3f})'
        )
        
        # Log path details
        if len(request.path.poses) > 0:
            first_wp = request.path.poses[0]
            last_wp = request.path.poses[-1]
            self.node.get_logger().info(
                f'[REPLAN REQUEST] Path range: first=({first_wp.pose.position.x:.2f}, {first_wp.pose.position.y:.2f}), '
                f'last=({last_wp.pose.position.x:.2f}, {last_wp.pose.position.y:.2f}), '
                f'frame={request.path.header.frame_id}'
            )
        
        # Timing tracking
        grid_extension_time = 0.0
        collision_check_time = 0.0
        replanning_time = 0.0
        grid_extended = False
        
        try:
            # 1. Validate request
            if request.path is None or len(request.path.poses) == 0:
                response.has_collision = False
                response.path_found = False
                response.message = "Empty path provided"
                return response
            
            # 2. Get occupancy grid
            grid = self.grid_subscriber.get_current_grid()
            if not grid or not validate_grid(grid):
                response.has_collision = False
                response.path_found = False
                response.message = "Occupancy grid not available"
                return response
            
            original_grid_size = grid.info.width * grid.info.height
            
            # 2.5. Extend grid if next waypoints are outside (for long paths)
            grid_extension_start = time.time()
            grid, grid_extended = self._extend_grid_for_path_if_needed(
                grid,
                request.path,
                request.start_waypoint_index
            )
            grid_extension_time = time.time() - grid_extension_start
            
            if grid_extended:
                new_grid_size = grid.info.width * grid.info.height
                self.node.get_logger().info(
                    f'[REPLAN TIMING] Grid extension: {grid_extension_time*1000:.1f}ms '
                    f'(grid expanded from {original_grid_size} to {new_grid_size} cells)'
                )
            
            # 3. Check for collisions
            collision_check_start = time.time()
            has_collision = self._check_path_collision(
                request.path,
                grid,
                request.start_waypoint_index
            )
            collision_check_time = time.time() - collision_check_start
            
            response.has_collision = has_collision
            
            # 4. If collision detected, replan
            if has_collision:
                self.node.get_logger().info(
                    f'Collision detected in path, replanning to goal '
                    f'({request.goal_pose.pose.position.x:.2f}, {request.goal_pose.pose.position.y:.2f})'
                )
                
                # Create a ComputePathToPose request for replanning
                from ros2_navigation.srv import ComputePathToPose
                replan_request = ComputePathToPose.Request()
                replan_request.goal_pose = request.goal_pose
                # Empty start_pose means service will read from TF
                replan_request.start_pose = PoseStamped()
                replan_request.start_pose.header.frame_id = ''
                
                # Call path planning service
                replan_start = time.time()
                replan_response = self.path_planning_service.compute_path_callback(
                    replan_request,
                    ComputePathToPose.Response()
                )
                replanning_time = time.time() - replan_start
                
                if replan_response.path_found:
                    response.path = replan_response.path
                    response.path_found = True
                    response.message = f"Path replanned successfully: {replan_response.message}"
                    self.node.get_logger().info(
                        f'Replanning successful: new path with {len(replan_response.path.poses)} waypoints'
                    )
                else:
                    response.path = Path()
                    response.path_found = False
                    response.message = f"Replanning failed: {replan_response.message}"
                    self.node.get_logger().warn(
                        f'Replanning failed: {replan_response.message}'
                    )
            else:
                # No collision, return empty path
                response.path = Path()
                response.path_found = False
                response.message = "No collision detected in path"
            
            # Log total timing with breakdown
            total_time = time.time() - total_start_time
            other_time = total_time - grid_extension_time - collision_check_time - replanning_time
            
            self.node.get_logger().info(
                f'[REPLAN TIMING] Total: {total_time*1000:.1f}ms | '
                f'Grid extension: {grid_extension_time*1000:.1f}ms ({grid_extension_time/total_time*100:.1f}%) | '
                f'Collision check: {collision_check_time*1000:.1f}ms ({collision_check_time/total_time*100:.1f}%) | '
                f'Replanning: {replanning_time*1000:.1f}ms ({replanning_time/total_time*100:.1f}%) | '
                f'Other: {other_time*1000:.1f}ms ({other_time/total_time*100:.1f}%)'
            )
            
            if grid_extended:
                self.node.get_logger().info(
                    f'[REPLAN OVERHEAD] Grid extension overhead: {grid_extension_time*1000:.1f}ms '
                    f'({grid_extension_time/total_time*100:.1f}% of total time)'
                )
            
            return response
            
        except Exception as e:
            total_time = time.time() - total_start_time
            self.node.get_logger().error(
                f'[REPLAN TIMING] Total: {total_time*1000:.1f}ms (ERROR occurred) | '
                f'Unexpected error in replanning: [{type(e).__name__}] {e}'
            )
            response.has_collision = False
            response.path_found = False
            response.message = f"Unexpected error: {str(e)}"
            return response
    
    def _extend_grid_for_path_if_needed(
        self,
        grid: OccupancyGrid,
        path: Path,
        start_waypoint_index: int
    ) -> Tuple[OccupancyGrid, bool]:
        """
        Extend grid if next waypoints are outside grid bounds.
        
        This helps with long paths where waypoints may be outside the current grid.
        Only checks the next few waypoints (up to 5) for performance.
        
        Args:
            grid: Current occupancy grid
            path: Path to check
            start_waypoint_index: Start checking from this waypoint index
            
        Returns:
            Tuple of (Extended grid if extension was needed, original grid otherwise, bool indicating if grid was extended)
        """
        if path is None or len(path.poses) == 0:
            return grid, False
        
        if start_waypoint_index < 0:
            start_waypoint_index = 0
        if start_waypoint_index >= len(path.poses):
            return grid, False
        
        # Check next few waypoints to see if any are outside grid
        # Only check first 5 waypoints for performance (focus on immediate next waypoints)
        end_index = min(start_waypoint_index + 5, len(path.poses))
        waypoint_to_extend_for = None
        
        for i in range(start_waypoint_index, end_index):
            waypoint = path.poses[i]
            
            # Transform waypoint to grid frame if needed
            waypoint_in_grid_frame = waypoint
            if waypoint.header.frame_id != grid.header.frame_id:
                try:
                    waypoint_in_grid_frame = self.tf_manager.transform_pose(
                        waypoint,
                        grid.header.frame_id
                    )
                except Exception as e:
                    self.node.get_logger().debug(
                        f'Cannot transform waypoint {i} for grid extension: {e}'
                    )
                    continue
            
            # Check if waypoint is in grid
            if not is_goal_in_grid(grid, waypoint_in_grid_frame):
                waypoint_to_extend_for = waypoint_in_grid_frame
                self.node.get_logger().debug(
                    f'Waypoint {i} at ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f}) '
                    f'is outside grid, will extend grid...'
                )
                break
        
        # Extend grid if needed
        if waypoint_to_extend_for is not None:
            try:
                extended_grid = extend_grid_for_goal(
                    grid,
                    waypoint_to_extend_for,
                    self.grid_extension_distance,
                    self.grid_extension_resolution,
                    node=self.node
                )
                self.node.get_logger().info(
                    f'[REPLAN] Grid extended: '
                    f'{grid.info.width}x{grid.info.height} -> '
                    f'{extended_grid.info.width}x{extended_grid.info.height} cells'
                )
                return extended_grid, True
            except Exception as e:
                self.node.get_logger().warn(
                    f'Failed to extend grid for replanning: {e}, using original grid'
                )
                return grid, False
        
        return grid, False
    
    def _check_path_collision(
        self,
        path: Path,
        grid: OccupancyGrid,
        start_waypoint_index: int
    ) -> bool:
        """
        Check if the path collides with obstacles.
        Checks both waypoints and path segments.
        
        Args:
            path: Path to check
            grid: Occupancy grid
            start_waypoint_index: Start checking from this waypoint index
            
        Returns:
            True if collision detected, False otherwise
        """
        if path is None or len(path.poses) == 0:
            self.node.get_logger().debug('_check_path_collision: no path available')
            return False
        
        if start_waypoint_index < 0:
            start_waypoint_index = 0
        if start_waypoint_index >= len(path.poses):
            self.node.get_logger().debug('_check_path_collision: start_waypoint_index out of bounds')
            return False
        
        # Check if path frame matches grid frame (may need transformation)
        if path.header.frame_id != grid.header.frame_id:
            self.node.get_logger().debug(
                f'Frame mismatch: path frame={path.header.frame_id}, '
                f'grid frame={grid.header.frame_id} - attempting transformation'
            )
        
        # Log collision check start
        total_remaining_waypoints = len(path.poses) - start_waypoint_index
        self.node.get_logger().info(
            f'[COLLISION CHECK] Starting: path has {len(path.poses)} waypoints, '
            f'start_index={start_waypoint_index}, '
            f'remaining={total_remaining_waypoints}, '
            f'grid size: {grid.info.width}x{grid.info.height}, '
            f'grid frame: {grid.header.frame_id}, path frame: {path.header.frame_id}'
        )
        
        # Log first and last waypoint positions
        if len(path.poses) > start_waypoint_index:
            first_check_wp = path.poses[start_waypoint_index]
            self.node.get_logger().info(
                f'[COLLISION CHECK] First waypoint to check (index {start_waypoint_index}): '
                f'({first_check_wp.pose.position.x:.2f}, {first_check_wp.pose.position.y:.2f}), '
                f'frame={first_check_wp.header.frame_id}'
            )
        
        # Check all remaining waypoints (from start_waypoint_index onwards)
        waypoints_checked = 0
        waypoints_out_of_bounds = 0
        for i in range(start_waypoint_index, len(path.poses)):
            waypoint = path.poses[i]
            
            # Transform waypoint to grid frame if needed
            waypoint_in_grid_frame = waypoint
            if waypoint.header.frame_id != grid.header.frame_id:
                try:
                    waypoint_in_grid_frame = self.tf_manager.transform_pose(
                        waypoint,
                        grid.header.frame_id
                    )
                except Exception as e:
                    self.node.get_logger().debug(
                        f'Cannot transform waypoint {i} to grid frame: {e}, skipping collision check'
                    )
                    continue
            
            # Check if waypoint is in occupied cell
            grid_coords = world_to_grid_coords_int(waypoint_in_grid_frame, grid)
            if grid_coords is None:
                waypoints_out_of_bounds += 1
                # Log first few out-of-bounds waypoints
                if waypoints_out_of_bounds <= 3:
                    self.node.get_logger().debug(
                        f'Waypoint {i} at ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f}) '
                        f'is out of grid bounds'
                    )
                continue
            
            waypoints_checked += 1
            if is_cell_occupied(grid, grid_coords[0], grid_coords[1]):
                cell_value = grid.data[grid_coords[1] * grid.info.width + grid_coords[0]]
                self.node.get_logger().warn(
                    f'Waypoint {i} at ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f}) '
                    f'is in occupied cell (grid=({grid_coords[0]}, {grid_coords[1]}), value={cell_value})'
                )
                return True
        
        # Log waypoint check summary
        if waypoints_out_of_bounds > 0:
            self.node.get_logger().info(
                f'[COLLISION CHECK] Waypoints: checked {waypoints_checked}, '
                f'{waypoints_out_of_bounds} out of bounds (total: {total_remaining_waypoints})'
            )
            if waypoints_checked == 0 and waypoints_out_of_bounds == total_remaining_waypoints:
                self.node.get_logger().warn(
                    f'[COLLISION CHECK] All {total_remaining_waypoints} remaining waypoints are out of grid bounds!'
                )
        else:
            self.node.get_logger().debug(
                f'[COLLISION CHECK] Waypoints: checked {waypoints_checked} (all in bounds)'
            )
        
        # Check all path segments between remaining waypoints
        # If start_waypoint_index is at the last waypoint, we should still check
        # the segment from the previous waypoint to the current one
        # Adjust start index to ensure we check at least one segment
        segment_start_index = start_waypoint_index
        if segment_start_index >= len(path.poses) - 1 and len(path.poses) > 1:
            # If we're at or past the last waypoint, check the last segment
            segment_start_index = len(path.poses) - 2
            self.node.get_logger().info(
                f'[COLLISION CHECK] start_waypoint_index at last waypoint, '
                f'adjusting to check last segment from index {segment_start_index}'
            )
        
        total_segments = len(path.poses) - 1 - segment_start_index
        self.node.get_logger().info(
            f'[COLLISION CHECK] Checking {total_segments} path segments '
            f'(from waypoint {segment_start_index} to {len(path.poses) - 1})...'
        )
        
        if total_segments <= 0:
            self.node.get_logger().warn(
                f'[COLLISION CHECK] WARNING: No segments to check! '
                f'path.poses={len(path.poses)}, start_waypoint_index={start_waypoint_index}, '
                f'segment_start_index={segment_start_index}'
            )
            return False
        
        segments_checked = 0
        segments_skipped = 0
        for i in range(segment_start_index, len(path.poses) - 1):
            start_waypoint = path.poses[i]
            end_waypoint = path.poses[i + 1]
            
            # Transform to grid frame if needed
            start_in_grid_frame = start_waypoint
            end_in_grid_frame = end_waypoint
            
            if start_waypoint.header.frame_id != grid.header.frame_id:
                try:
                    start_in_grid_frame = self.tf_manager.transform_pose(
                        start_waypoint,
                        grid.header.frame_id
                    )
                except Exception as e:
                    segments_skipped += 1
                    if segments_skipped <= 3:
                        self.node.get_logger().debug(
                            f'Cannot transform start waypoint {i} to grid frame: {e}, skipping segment'
                        )
                    continue
            
            if end_waypoint.header.frame_id != grid.header.frame_id:
                try:
                    end_in_grid_frame = self.tf_manager.transform_pose(
                        end_waypoint,
                        grid.header.frame_id
                    )
                except Exception as e:
                    segments_skipped += 1
                    if segments_skipped <= 3:
                        self.node.get_logger().debug(
                            f'Cannot transform end waypoint {i+1} to grid frame: {e}, skipping segment'
                        )
                    continue
            
            # Check collision - this handles segments where one or both endpoints
            # are outside the grid by clipping and checking the portion within the grid
            debug_collision = (segments_checked <= 3)
            collision_detected, was_checked = check_line_collision(
                grid, start_in_grid_frame, end_in_grid_frame, debug=debug_collision
            )
            
            if not was_checked:
                # Segment completely outside grid
                segments_skipped += 1
                if segments_skipped <= 3:
                    start_coords = world_to_grid_coords_int(start_in_grid_frame, grid)
                    end_coords = world_to_grid_coords_int(end_in_grid_frame, grid)
                    self.node.get_logger().debug(
                        f'[COLLISION CHECK] Segment {i}->{i+1} completely outside grid: '
                        f'start={start_coords}, end={end_coords}'
                    )
                continue
            
            segments_checked += 1
            
            if collision_detected:
                # Get grid coordinates for logging
                start_grid = world_to_grid_coords_int(start_in_grid_frame, grid)
                end_grid = world_to_grid_coords_int(end_in_grid_frame, grid)
                self.node.get_logger().warn(
                    f'Path segment {i}->{i+1} collides with obstacle '
                    f'(from ({start_waypoint.pose.position.x:.2f}, {start_waypoint.pose.position.y:.2f}) '
                    f'grid=({start_grid[0] if start_grid else "N/A (clipped)"}, {start_grid[1] if start_grid else "N/A (clipped)"}) '
                    f'to ({end_waypoint.pose.position.x:.2f}, {end_waypoint.pose.position.y:.2f}) '
                    f'grid=({end_grid[0] if end_grid else "N/A (clipped)"}, {end_grid[1] if end_grid else "N/A (clipped)"}))'
                )
                return True
        
        # Log segment check summary
        self.node.get_logger().info(
            f'[COLLISION CHECK] Segments: checked {segments_checked}, '
            f'skipped {segments_skipped} (total: {total_segments}), no collision detected'
        )
        
        return False

