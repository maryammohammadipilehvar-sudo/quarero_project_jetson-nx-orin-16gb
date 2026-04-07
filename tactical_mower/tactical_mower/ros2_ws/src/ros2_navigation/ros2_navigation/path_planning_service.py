#!/usr/bin/env python3
"""
Path Planning Service for handling path planning requests.

This module implements the service handler for ComputePathToPose,
coordinating between all components for path planning.
"""

import time
import math
from rclpy.node import Node
from ros2_navigation.srv import ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ros2_navigation.tf_manager import TFManager
from ros2_navigation.planner_manager import PlannerManager
from ros2_navigation.grid_utils import (
    validate_grid,
    is_goal_in_grid,
    extend_grid_for_goal
)
from ros2_navigation.occupancy_grid_subscriber import OccupancyGridSubscriber
from ros2_navigation.path_publisher import PathPublisher
from ros2_navigation.inflated_grid_publisher import InflatedGridPublisher
from ros2_navigation.extended_grid_publisher import ExtendedGridPublisher
from ros2_navigation.inflation_layer import InflationLayer


class PathPlanningService:
    """
    Service handler for path planning requests.
    """
    
    def __init__(
        self,
        node: Node,
        tf_manager: TFManager,
        planner_manager: PlannerManager,
        grid_subscriber: OccupancyGridSubscriber,
        path_publisher: PathPublisher,
        inflated_grid_publisher: InflatedGridPublisher,
        extended_grid_publisher: ExtendedGridPublisher,
        grid_extension_distance: float = 5.0,
        grid_extension_resolution: float = 0.1,
        max_planning_time: float = 5.0
    ):
        """
        Initialize Path Planning Service.
        
        Args:
            node: ROS2 node instance
            tf_manager: TF Manager for pose transformations
            planner_manager: Planner Manager for path planning
            grid_subscriber: OccupancyGrid Subscriber
            path_publisher: Path Publisher for visualization
            inflated_grid_publisher: Inflated Grid Publisher for RViz visualization
            extended_grid_publisher: Extended Grid Publisher for RViz visualization
            grid_extension_distance: Distance to extend grid if goal is outside
            grid_extension_resolution: Resolution for extended grid areas
            max_planning_time: Maximum time for planning in seconds
        """
        self.node = node
        self.tf_manager = tf_manager
        self.planner_manager = planner_manager
        self.grid_subscriber = grid_subscriber
        self.path_publisher = path_publisher
        self.inflated_grid_publisher = inflated_grid_publisher
        self.extended_grid_publisher = extended_grid_publisher
        self.grid_extension_distance = grid_extension_distance
        self.grid_extension_resolution = grid_extension_resolution
        self.max_planning_time = max_planning_time
        
        # Create service server
        self.service = self.node.create_service(
            ComputePathToPose,
            'compute_path_to_pose',
            self.compute_path_callback
        )
        
        self.node.get_logger().info(
            'PathPlanningService initialized, service: compute_path_to_pose'
        )
    
    def compute_path_callback(self, request, response):
        """
        Service callback for path planning requests.
        
        Args:
            request: ComputePathToPose request
            response: ComputePathToPose response
            
        Returns:
            Response with planned path or error information
        """
        start_time = time.time()
        step_time = start_time
        
        self.node.get_logger().info(
            f'Received path planning request: goal=({request.goal_pose.pose.position.x:.3f}, '
            f'{request.goal_pose.pose.position.y:.3f}), frame={request.goal_pose.header.frame_id}, '
            f'start_frame={request.start_pose.header.frame_id}'
        )
        
        try:
            # 1. Validate request
            if not self._validate_request(request):
                response.path_found = False
                response.message = "Invalid request"
                response.planning_time = time.time() - start_time
                return response
            
            # 2. Get robot pose if start_pose not provided
            start_pose = request.start_pose
            if start_pose.header.frame_id == '' or not self._is_pose_valid(start_pose):
                try:
                    step_start = time.time()
                    start_pose = self.tf_manager.get_robot_pose()
                    step_time = time.time()
                    self.node.get_logger().info(f'[TIMING] Get robot pose: {(step_time - step_start)*1000:.1f}ms')
                except Exception as e:
                    self.node.get_logger().error(f'Failed to get robot pose: {e}')
                    response.path_found = False
                    response.message = f"Failed to get robot pose: {str(e)}"
                    response.planning_time = time.time() - start_time
                    return response
            
            # 3. Transform goal_pose to global frame if needed
            goal_pose = request.goal_pose
            original_goal_frame = goal_pose.header.frame_id
            self.node.get_logger().info(
                f'[FRAME] Goal pose received in frame: {original_goal_frame}, '
                f'target global frame: {self.tf_manager.global_frame}'
            )
            if goal_pose.header.frame_id != self.tf_manager.global_frame:
                try:
                    step_start = time.time()
                    goal_pose = self.tf_manager.transform_pose(
                        goal_pose,
                        self.tf_manager.global_frame
                    )
                    step_time = time.time()
                    self.node.get_logger().info(
                        f'[FRAME] ✓ Goal pose transformed: {original_goal_frame} → {self.tf_manager.global_frame} '
                        f'({(step_time - step_start)*1000:.1f}ms)'
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f'[FRAME] ✗ Failed to transform goal pose from {original_goal_frame} to '
                        f'{self.tf_manager.global_frame}: {e}'
                    )
                    response.path_found = False
                    response.message = f"Failed to transform goal pose: {str(e)}"
                    response.planning_time = time.time() - start_time
                    return response
            else:
                self.node.get_logger().debug(
                    f'[FRAME] Goal pose already in global frame {self.tf_manager.global_frame}, skipping transformation'
                )
            
            # 4. Get occupancy grid
            step_start = time.time()
            occupancy_grid = self.grid_subscriber.get_current_grid()
            step_time = time.time()
            self.node.get_logger().info(f'[TIMING] Get occupancy grid: {(step_time - step_start)*1000:.1f}ms')
            if not occupancy_grid or not validate_grid(occupancy_grid):
                self.node.get_logger().error('Occupancy grid not available')
                response.path_found = False
                response.message = "Occupancy grid not available"
                response.planning_time = time.time() - start_time
                return response
            
            grid_size = occupancy_grid.info.width * occupancy_grid.info.height
            self.node.get_logger().info(
                f'[TIMING] Grid size: {occupancy_grid.info.width}x{occupancy_grid.info.height} = {grid_size} cells'
            )
            self.node.get_logger().info(
                f'[FRAME] Grid frame: {occupancy_grid.header.frame_id}, '
                f'Global frame: {self.tf_manager.global_frame}, '
                f'Goal frame: {goal_pose.header.frame_id}, '
                f'Start frame: {start_pose.header.frame_id}'
            )
            
            # 4.5. Transform poses to grid frame for grid operations
            # Grid operations (is_goal_in_grid, extend_grid_for_goal, plan_path) require poses in grid frame
            # The planner's transform_pose_to_grid_frame assumes poses are already in grid frame
            goal_pose_in_grid_frame = goal_pose
            start_pose_in_grid_frame = start_pose
            
            # Transform goal pose to grid frame if needed
            if goal_pose.header.frame_id != occupancy_grid.header.frame_id:
                try:
                    step_start = time.time()
                    goal_pose_in_grid_frame = self.tf_manager.transform_pose(
                        goal_pose,
                        occupancy_grid.header.frame_id
                    )
                    step_time = time.time()
                    self.node.get_logger().info(
                        f'[FRAME] ✓ Goal pose transformed to grid frame: '
                        f'{goal_pose.header.frame_id} → {occupancy_grid.header.frame_id} '
                        f'({(step_time - step_start)*1000:.1f}ms)'
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f'[FRAME] ✗ Failed to transform goal pose to grid frame: '
                        f'{goal_pose.header.frame_id} → {occupancy_grid.header.frame_id}: {e}'
                    )
                    response.path_found = False
                    response.message = f"Failed to transform goal pose to grid frame: {str(e)}"
                    response.planning_time = time.time() - start_time
                    return response
            else:
                self.node.get_logger().debug(
                    f'[FRAME] Goal pose already in grid frame {occupancy_grid.header.frame_id}, skipping transformation'
                )
            
            # Transform start pose to grid frame if needed
            if start_pose.header.frame_id != occupancy_grid.header.frame_id:
                try:
                    step_start = time.time()
                    start_pose_in_grid_frame = self.tf_manager.transform_pose(
                        start_pose,
                        occupancy_grid.header.frame_id
                    )
                    step_time = time.time()
                    self.node.get_logger().info(
                        f'[FRAME] ✓ Start pose transformed to grid frame: '
                        f'{start_pose.header.frame_id} → {occupancy_grid.header.frame_id} '
                        f'({(step_time - step_start)*1000:.1f}ms)'
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f'[FRAME] ✗ Failed to transform start pose to grid frame: '
                        f'{start_pose.header.frame_id} → {occupancy_grid.header.frame_id}: {e}'
                    )
                    response.path_found = False
                    response.message = f"Failed to transform start pose to grid frame: {str(e)}"
                    response.planning_time = time.time() - start_time
                    return response
            else:
                self.node.get_logger().debug(
                    f'[FRAME] Start pose already in grid frame {occupancy_grid.header.frame_id}, skipping transformation'
                )
            
            # 5. Extend grid if goal is outside
            if not is_goal_in_grid(occupancy_grid, goal_pose_in_grid_frame):
                try:
                    step_start = time.time()
                    # extend_grid_for_goal requires goal_pose and grid to be in the same frame
                    # goal_pose_in_grid_frame is already transformed above
                    occupancy_grid = extend_grid_for_goal(
                        occupancy_grid,
                        goal_pose_in_grid_frame,
                        self.grid_extension_distance,
                        self.grid_extension_resolution,
                        node=self.node
                    )
                    step_time = time.time()
                    new_grid_size = occupancy_grid.info.width * occupancy_grid.info.height
                    self.node.get_logger().info(
                        f'[TIMING] Grid extension: {(step_time - step_start)*1000:.1f}ms '
                        f'(grid expanded from {grid_size} to {new_grid_size} cells)'
                    )
                    self.extended_grid_publisher.publish_extended_grid(occupancy_grid)
                except Exception as e:
                    self.node.get_logger().error(f'Failed to extend grid: {e}')
                    response.path_found = False
                    response.message = f"Failed to extend grid: {str(e)}"
                    response.planning_time = time.time() - start_time
                    return response
            
            # 6. Apply inflation and publish inflated grid for visualization
            if self.planner_manager.inflation_radius > 0.0:
                try:
                    step_start = time.time()
                    inflated_grid = InflationLayer.inflate_grid(
                        occupancy_grid,
                        self.planner_manager.inflation_radius
                    )
                    step_time = time.time()
                    self.node.get_logger().info(
                        f'[TIMING] Grid inflation (visualization): {(step_time - step_start)*1000:.1f}ms'
                    )
                    self.inflated_grid_publisher.publish_inflated_grid(inflated_grid)
                except Exception as e:
                    self.node.get_logger().warn(f'Failed to inflate grid for visualization: {e}')
            
            # 7. Plan path (inflation is applied inside planner_manager.plan_path)
            # Use poses in grid frame for planning
            planning_start = time.time()
            path = self.planner_manager.plan_path(
                start_pose_in_grid_frame,
                goal_pose_in_grid_frame,
                occupancy_grid
            )
            planning_time = time.time() - planning_start
            self.node.get_logger().info(f'[TIMING] Path planning: {planning_time*1000:.1f}ms')
            
            # Check timeout
            total_time = time.time() - start_time
            pre_planning_time = planning_start - start_time
            self.node.get_logger().info(
                f'[TIMING] Pre-planning overhead: {pre_planning_time*1000:.1f}ms, '
                f'Planning: {planning_time*1000:.1f}ms, '
                f'Total: {total_time*1000:.1f}ms'
            )
            if total_time > self.max_planning_time:
                self.node.get_logger().warn(f'Planning timeout: {total_time}s > {self.max_planning_time}s')
                response.path_found = False
                response.message = "Planning timeout"
                response.planning_time = total_time
                return response
            
            # 8. Check if path was found
            if path is None or len(path.poses) == 0:
                self.node.get_logger().warn('No path found')
                response.path_found = False
                response.message = "No path found"
                response.planning_time = total_time
                return response
            
            # 8.5. Transform path to global frame if needed
            post_planning_start = time.time()
            
            path_original_frame = path.header.frame_id
            self.node.get_logger().info(
                f'[FRAME] Path planning complete. Path frame: {path_original_frame}, '
                f'Grid frame: {occupancy_grid.header.frame_id}, '
                f'Global frame: {self.tf_manager.global_frame}, '
                f'Waypoints: {len(path.poses)}'
            )
            
            # Check if path is already in global frame (common case when grid frame = global frame)
            if path.header.frame_id == self.tf_manager.global_frame:
                # Path already in global frame, just update timestamp
                path.header.stamp = self.node.get_clock().now().to_msg()
                self.node.get_logger().info(
                    f'[FRAME] ✓ Path already in global frame {self.tf_manager.global_frame}, '
                    f'skipping transformation (performance optimized)'
                )
            elif path.header.frame_id == occupancy_grid.header.frame_id and \
                 occupancy_grid.header.frame_id == self.tf_manager.global_frame:
                # Path frame matches grid frame which is already global frame
                path.header.frame_id = self.tf_manager.global_frame
                path.header.stamp = self.node.get_clock().now().to_msg()
                self.node.get_logger().info(
                    f'[FRAME] ✓ Path frame ({path_original_frame}) matches global frame, '
                    f'skipping transformation (performance optimized)'
                )
            else:
                # Transformation needed - transform all waypoints from grid frame to global frame
                # This is the expensive operation we want to avoid by having grids in map frame
                self.node.get_logger().warn(
                    f'[FRAME] ⚠️ Path transformation required: {path_original_frame} → {self.tf_manager.global_frame}. '
                    f'This is slow ({len(path.poses)} waypoints). '
                    f'Consider enabling auto_transform_grid to improve performance.'
                )
                try:
                    step_start = time.time()
                    transformed_poses = []
                    failed_count = 0
                    
                    # Transform each waypoint sequentially
                    for i, pose in enumerate(path.poses):
                        try:
                            transformed_pose = self.tf_manager.transform_pose(
                                pose,
                                self.tf_manager.global_frame
                            )
                            transformed_poses.append(transformed_pose)
                        except Exception as e:
                            failed_count += 1
                            self.node.get_logger().warn(
                                f'[FRAME] ✗ Failed to transform waypoint {i+1}/{len(path.poses)}: {e}'
                            )
                            # If too many fail, abort early
                            if failed_count > len(path.poses) / 2:
                                self.node.get_logger().error(
                                    f'[FRAME] Too many waypoint transformations failed ({failed_count}/{len(path.poses)}), aborting'
                                )
                                break
                            continue
                    
                    step_time = time.time()
                    transform_duration = (step_time - step_start) * 1000
                    self.node.get_logger().info(
                        f'[FRAME] ✓ Transformed {len(transformed_poses)}/{len(path.poses)} waypoints: '
                        f'{path_original_frame} → {self.tf_manager.global_frame} '
                        f'({transform_duration:.1f}ms, {transform_duration/len(path.poses):.1f}ms per waypoint)'
                    )
                    
                    if len(transformed_poses) == 0:
                        self.node.get_logger().error('[FRAME] All waypoints failed to transform')
                        response.path_found = False
                        response.message = "Failed to transform path waypoints"
                        response.planning_time = total_time
                        return response
                    
                    # If some failed but we have enough, use what we have
                    if len(transformed_poses) < len(path.poses):
                        self.node.get_logger().warn(
                            f'[FRAME] Only {len(transformed_poses)}/{len(path.poses)} waypoints transformed successfully, using partial path'
                        )
                    
                    path.poses = transformed_poses
                    path.header.frame_id = self.tf_manager.global_frame
                    path.header.stamp = self.node.get_clock().now().to_msg()
                except Exception as e:
                    self.node.get_logger().error(f'[FRAME] ✗ Failed to transform path: {e}')
                    self.node.get_logger().warn(
                        f'[FRAME] Using path in original frame {path.header.frame_id} (may cause issues)'
                    )
            
            # 9. Set response
            response_set_start = time.time()
            response.path = path
            response.path_found = True
            response.planning_time = total_time
            response.message = f"Path found with {len(path.poses)} waypoints"
            response_set_time = time.time() - response_set_start
            self.node.get_logger().debug(f'[TIMING] Set response: {response_set_time*1000:.1f}ms')
            
            # 10. Publish path for visualization
            publish_start = time.time()
            self.path_publisher.publish_path(path)
            publish_time = time.time() - publish_start
            self.node.get_logger().info(f'[TIMING] Publish path: {publish_time*1000:.1f}ms')
            
            post_planning_time = time.time() - post_planning_start
            final_total_time = time.time() - start_time
            self.node.get_logger().info(
                f'[TIMING] Post-planning overhead: {post_planning_time*1000:.1f}ms, '
                f'Final total: {final_total_time*1000:.1f}ms'
            )
            
            self.node.get_logger().info(
                f'Path planned: {len(path.poses)} waypoints, time: {final_total_time:.3f}s'
            )
            
            return response
            
        except Exception as e:
            self.node.get_logger().error(f'Unexpected error in path planning: [{type(e).__name__}] {e}')
            response.path_found = False
            response.message = f"Unexpected error: {str(e)}"
            response.planning_time = time.time() - start_time
            return response
    
    def _validate_request(self, request) -> bool:
        """
        Validate service request.
        
        Args:
            request: ComputePathToPose request
            
        Returns:
            True if valid, False otherwise
        """
        if not self._is_pose_valid(request.goal_pose):
            self.node.get_logger().error('Invalid goal_pose in request')
            return False
        
        if request.start_pose.header.frame_id != '':
            if not self._is_pose_valid(request.start_pose):
                self.node.get_logger().error('Invalid start_pose in request')
                return False
        
        return True
    
    def _is_pose_valid(self, pose: PoseStamped) -> bool:
        """
        Check if pose is valid.
        
        Args:
            pose: Pose to validate
            
        Returns:
            True if valid, False otherwise
        """
        if pose is None:
            return False
        
        # Check for NaN or Inf
        pos = pose.pose.position
        if (math.isnan(pos.x) or math.isinf(pos.x) or
            math.isnan(pos.y) or math.isinf(pos.y) or
            math.isnan(pos.z) or math.isinf(pos.z)):
            return False
        
        return True
    
    def _set_response_fields(
        self,
        response,
        path: Path,
        success: bool,
        message: str,
        planning_time: float
    ):
        """
        Set response fields (handles Option A/B differences).
        
        Args:
            response: Service response
            path: Planned path
            success: Whether planning was successful
            message: Status message
            planning_time: Planning time in seconds
        """
        response.path = path
        response.path_found = success
        response.planning_time = planning_time
        response.message = message

