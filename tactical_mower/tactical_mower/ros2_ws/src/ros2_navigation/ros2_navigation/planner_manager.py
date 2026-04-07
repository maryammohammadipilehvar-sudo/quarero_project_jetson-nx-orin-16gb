#!/usr/bin/env python3
"""
Planner Manager for nav2 planner plugin integration.

This module manages nav2 planner plugins and performs path planning.
"""

import time
import math
import heapq
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from ros2_navigation.inflation_layer import InflationLayer


class PlannerManager:
    """
    Manages nav2 planner plugins and performs path planning.
    """
    
    def __init__(self, node: Node, plugin_name: str, params: dict = None, algorithm_type: str = 'astar', inflation_radius: float = 0.0):
        """
        Initialize Planner Manager.
        
        Args:
            node: ROS2 node instance
            plugin_name: Name of the planner plugin (e.g., 'nav2_smac_planner::SmacPlanner2D')
            params: Optional parameters for the planner
            algorithm_type: Type of path planning algorithm ('astar' or 'theta_star')
            inflation_radius: Radius in meters to inflate obstacles (0.0 to disable)
        """
        self.node = node
        self.plugin_name = plugin_name
        self.params = params or {}
        self.algorithm_type = algorithm_type.lower()
        self.planner = None
        self.inflation_radius = inflation_radius
        
        valid_algorithms = ['astar', 'theta_star']
        if self.algorithm_type not in valid_algorithms:
            self.node.get_logger().warn(
                f'Invalid algorithm_type "{self.algorithm_type}", using "astar" instead. '
                f'Valid options: {valid_algorithms}'
            )
            self.algorithm_type = 'astar'
        
        try:
            self.planner = self._load_planner_plugin(plugin_name)
            self.node.get_logger().info(
                f'PlannerManager initialized: plugin={plugin_name}, algorithm={self.algorithm_type}, '
                f'inflation_radius={self.inflation_radius}m'
            )
        except Exception as e:
            self.node.get_logger().error(
                f'Failed to load planner plugin {plugin_name}: {e}'
            )
            self.node.get_logger().warn('Using fallback planner')
    
    def _load_planner_plugin(self, plugin_name: str):
        """
        Load nav2 planner plugin.
        
        Args:
            plugin_name: Name of the planner plugin
            
        Returns:
            Planner instance
            
        Note: This is a placeholder. Actual nav2 plugin loading may require
        different approach depending on ROS2 distribution and nav2 version.
        """
        try:
            self.node.get_logger().warn(
                f'Direct plugin loading not implemented for {plugin_name}. '
                'Using fallback planner.'
            )
            return None
        except Exception as e:
            raise RuntimeError(f'Failed to load planner: {e}')
    
    def create_costmap_from_grid(self, occupancy_grid: OccupancyGrid) -> np.ndarray:
        """
        Create costmap array from OccupancyGrid for path planning.
        
        Args:
            occupancy_grid: Occupancy grid to convert
            
        Returns:
            Numpy array with cost values (0=free, 100=occupied, -1=unknown)
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        costmap = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))
        
        return costmap
    
    def plan_path(
        self,
        start_pose: PoseStamped,
        goal_pose: PoseStamped,
        occupancy_grid: OccupancyGrid
    ) -> Path:
        """
        Plan path from start to goal using occupancy grid.
        
        Args:
            start_pose: Start pose
            goal_pose: Goal pose
            occupancy_grid: Occupancy grid for planning
            
        Returns:
            Planned path, or empty path if planning fails
        """
        planning_grid = self._apply_inflation(occupancy_grid)
        
        if self.planner is None:
            if self.algorithm_type == 'theta_star':
                return self._plan_path_theta_star(start_pose, goal_pose, planning_grid)
            else:
                return self._plan_path_astar(start_pose, goal_pose, planning_grid)
        
        try:
            costmap = self.create_costmap_from_grid(planning_grid)
            path = self.planner.createPlan(start_pose, goal_pose)
            return path
        except Exception as e:
            self.node.get_logger().error(f'Path planning failed: {e}')
            if self.algorithm_type == 'theta_star':
                return self._plan_path_theta_star(start_pose, goal_pose, planning_grid)
            else:
                return self._plan_path_astar(start_pose, goal_pose, planning_grid)
    
    def _apply_inflation(self, occupancy_grid: OccupancyGrid) -> OccupancyGrid:
        """
        Apply inflation to occupancy grid if inflation is enabled.
        
        Args:
            occupancy_grid: Original occupancy grid
            
        Returns:
            Inflated occupancy grid if inflation is enabled, original grid otherwise
        """
        if self.inflation_radius <= 0.0:
            return occupancy_grid
        
        try:
            inflated_grid = InflationLayer.inflate_grid(occupancy_grid, self.inflation_radius)
            self.node.get_logger().debug(
                f'Applied inflation with radius {self.inflation_radius}m to grid '
                f'{occupancy_grid.info.width}x{occupancy_grid.info.height}'
            )
            return inflated_grid
        except Exception as e:
            self.node.get_logger().error(f'Failed to apply inflation: {e}')
            return occupancy_grid
    
    def _plan_path_astar(
        self,
        start_pose: PoseStamped,
        goal_pose: PoseStamped,
        occupancy_grid: OccupancyGrid
    ) -> Path:
        """
        Path planning using A* algorithm.
        
        Args:
            start_pose: Start pose
            goal_pose: Goal pose
            occupancy_grid: Occupancy grid
            
        Returns:
            Planned path
        """
        from ros2_navigation.grid_utils import transform_pose_to_grid_frame
        
        # Get grid coordinates
        start_grid = transform_pose_to_grid_frame(start_pose, occupancy_grid)
        goal_grid = transform_pose_to_grid_frame(goal_pose, occupancy_grid)
        
        if start_grid is None or goal_grid is None:
            self.node.get_logger().error('Failed to transform poses to grid frame')
            return Path()
        
        start_x, start_y = start_grid
        goal_x, goal_y = goal_grid
        
        path = self._astar_path(
            start_x, start_y,
            goal_x, goal_y,
            occupancy_grid
        )
        
        if not path:
            self.node.get_logger().warn('Path planning algorithm found no path')
            return Path()
        
        from ros2_navigation.grid_utils import grid_to_world_coords
        
        path_poses = []
        for grid_x, grid_y in path:
            world_x, world_y = grid_to_world_coords(grid_x, grid_y, occupancy_grid)
            
            pose = PoseStamped()
            pose.header.frame_id = occupancy_grid.header.frame_id
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_poses.append(pose)
        
        result_path = Path()
        result_path.header.frame_id = occupancy_grid.header.frame_id
        result_path.header.stamp = self.node.get_clock().now().to_msg()
        result_path.poses = path_poses
        
        return result_path
    
    def _astar_path(
        self,
        start_x: int,
        start_y: int,
        goal_x: int,
        goal_y: int,
        occupancy_grid: OccupancyGrid
    ) -> list:
        """
        Simple A* path planning algorithm.
        
        Args:
            start_x, start_y: Start grid coordinates
            goal_x, goal_y: Goal grid coordinates
            occupancy_grid: Occupancy grid
            
        Returns:
            List of (x, y) grid coordinates
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        # Check bounds
        if not (0 <= start_x < width and 0 <= start_y < height):
            self.node.get_logger().error(
                f'[_astar_path] Start out of bounds: grid=({start_x}, {start_y}), '
                f'bounds=[0, {width})x[0, {height})'
            )
            return []
        if not (0 <= goal_x < width and 0 <= goal_y < height):
            self.node.get_logger().error(
                f'[_astar_path] Goal out of bounds: grid=({goal_x}, {goal_y}), '
                f'bounds=[0, {width})x[0, {height})'
            )
            return []
        
        # Validate start and goal cells are free
        if not self._is_cell_free(start_x, start_y, occupancy_grid):
            self.node.get_logger().warn(
                f'Start cell ({start_x}, {start_y}) is occupied, searching for nearest free cell'
            )
            free_start = self._find_nearest_free_cell(start_x, start_y, occupancy_grid)
            if free_start is None:
                self.node.get_logger().error(
                    f'Could not find free cell near start position ({start_x}, {start_y})'
                )
                return []
            start_x, start_y = free_start
        
        if not self._is_cell_free(goal_x, goal_y, occupancy_grid):
            self.node.get_logger().warn(
                f'Goal cell ({goal_x}, {goal_y}) is occupied, searching for nearest free cell'
            )
            free_goal = self._find_nearest_free_cell(goal_x, goal_y, occupancy_grid)
            if free_goal is None:
                self.node.get_logger().error(
                    f'Could not find free cell near goal position ({goal_x}, {goal_y})'
                )
                return []
            goal_x, goal_y = free_goal
        
        open_set = []
        initial_h = self._heuristic(start_x, start_y, goal_x, goal_y)
        heapq.heappush(open_set, (initial_h, start_x, start_y))
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): initial_h}
        closed_set = set()
        iterations = 0
        max_iterations = width * height * 2
        
        while open_set:
            iterations += 1
            if iterations > max_iterations:
                self.node.get_logger().warn(
                    f'Path planning exceeded max iterations: {iterations} > {max_iterations}'
                )
                break
            
            current_f, current_x, current_y = heapq.heappop(open_set)
            
            if (current_x, current_y) in closed_set:
                continue
            
            closed_set.add((current_x, current_y))
            
            if current_x == goal_x and current_y == goal_y:
                path = []
                while (current_x, current_y) in came_from:
                    path.append((current_x, current_y))
                    current_x, current_y = came_from[(current_x, current_y)]
                path.append((start_x, start_y))
                path.reverse()
                self.node.get_logger().info(
                    f'Path found: {len(path)} waypoints, {iterations} iterations'
                )
                return path
            
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                
                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    continue
                
                index = neighbor_y * width + neighbor_x
                if occupancy_grid.data[index] > 50:
                    continue
                
                tentative_g = g_score[(current_x, current_y)] + math.sqrt(dx*dx + dy*dy)
                if (neighbor_x, neighbor_y) not in g_score or tentative_g < g_score[(neighbor_x, neighbor_y)]:
                    came_from[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    g_score[(neighbor_x, neighbor_y)] = tentative_g
                    f_score[(neighbor_x, neighbor_y)] = tentative_g + self._heuristic(neighbor_x, neighbor_y, goal_x, goal_y)
                    if (neighbor_x, neighbor_y) not in closed_set:
                        heapq.heappush(open_set, (f_score[(neighbor_x, neighbor_y)], neighbor_x, neighbor_y))
        
        self.node.get_logger().warn(
            f'No path found after {iterations} iterations. '
            f'Start=({start_x}, {start_y}), Goal=({goal_x}, {goal_y})'
        )
        return []
    
    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Heuristic function for path planning (Euclidean distance)."""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def _is_cell_free(self, x: int, y: int, occupancy_grid: OccupancyGrid) -> bool:
        """
        Check if a grid cell is free (not occupied).
        
        Args:
            x, y: Grid coordinates
            occupancy_grid: Occupancy grid
            
        Returns:
            True if cell is free, False if occupied or out of bounds
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        if not (0 <= x < width and 0 <= y < height):
            return False
        
        index = y * width + x
        return occupancy_grid.data[index] <= 50  # Free if <= 50, occupied if > 50
    
    def _find_nearest_free_cell(
        self,
        start_x: int,
        start_y: int,
        occupancy_grid: OccupancyGrid,
        max_search_radius: int = 10
    ) -> tuple:
        """
        Find the nearest free cell to the given coordinates.
        
        Uses a spiral search pattern starting from the given cell.
        
        Args:
            start_x, start_y: Starting grid coordinates
            occupancy_grid: Occupancy grid
            max_search_radius: Maximum search radius in cells
            
        Returns:
            Tuple (x, y) of nearest free cell, or None if none found
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        # Check if start cell is already free
        if self._is_cell_free(start_x, start_y, occupancy_grid):
            return (start_x, start_y)
        
        for radius in range(1, max_search_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:
                        x = start_x + dx
                        y = start_y + dy
                        
                        if self._is_cell_free(x, y, occupancy_grid):
                            return (x, y)
        
        self.node.get_logger().warn(
            f'No free cell found within {max_search_radius} cells of ({start_x}, {start_y})'
        )
        return None
    
    def _plan_path_theta_star(
        self,
        start_pose: PoseStamped,
        goal_pose: PoseStamped,
        occupancy_grid: OccupancyGrid
    ) -> Path:
        """
        Path planning using Theta* algorithm.
        
        Theta* is an any-angle path planning algorithm that extends A* by allowing
        paths to pass through corners of grid cells, resulting in shorter and smoother paths.
        
        Args:
            start_pose: Start pose
            goal_pose: Goal pose
            occupancy_grid: Occupancy grid
            
        Returns:
            Planned path
        """
        from ros2_navigation.grid_utils import transform_pose_to_grid_frame
        
        # Get grid coordinates
        start_grid = transform_pose_to_grid_frame(start_pose, occupancy_grid)
        goal_grid = transform_pose_to_grid_frame(goal_pose, occupancy_grid)
        
        if start_grid is None or goal_grid is None:
            self.node.get_logger().error('Failed to transform poses to grid frame')
            return Path()
        
        start_x, start_y = start_grid
        goal_x, goal_y = goal_grid
        
        path = self._theta_star_path(
            start_x, start_y,
            goal_x, goal_y,
            occupancy_grid
        )
        
        if not path:
            self.node.get_logger().warn('Theta* algorithm found no path')
            return Path()
        
        from ros2_navigation.grid_utils import grid_to_world_coords
        
        path_poses = []
        for grid_x, grid_y in path:
            world_x, world_y = grid_to_world_coords(grid_x, grid_y, occupancy_grid)
            
            pose = PoseStamped()
            pose.header.frame_id = occupancy_grid.header.frame_id
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_poses.append(pose)
        
        result_path = Path()
        result_path.header.frame_id = occupancy_grid.header.frame_id
        result_path.header.stamp = self.node.get_clock().now().to_msg()
        result_path.poses = path_poses
        
        return result_path
    
    def _theta_star_path(
        self,
        start_x: int,
        start_y: int,
        goal_x: int,
        goal_y: int,
        occupancy_grid: OccupancyGrid
    ) -> list:
        """
        Theta* path planning algorithm.
        
        Theta* extends A* by allowing line-of-sight checks between nodes,
        enabling paths to pass through corners of grid cells for shorter paths.
        
        Args:
            start_x, start_y: Start grid coordinates
            goal_x, goal_y: Goal grid coordinates
            occupancy_grid: Occupancy grid
            
        Returns:
            List of (x, y) grid coordinates
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        if not (0 <= start_x < width and 0 <= start_y < height):
            return []
        if not (0 <= goal_x < width and 0 <= goal_y < height):
            return []
        
        if not self._is_cell_free(start_x, start_y, occupancy_grid):
            self.node.get_logger().warn(
                f'Start cell ({start_x}, {start_y}) is occupied, searching for nearest free cell'
            )
            free_start = self._find_nearest_free_cell(start_x, start_y, occupancy_grid)
            if free_start is None:
                self.node.get_logger().error('Could not find free cell near start position')
                return []
            start_x, start_y = free_start
        
        if not self._is_cell_free(goal_x, goal_y, occupancy_grid):
            self.node.get_logger().warn(
                f'Goal cell ({goal_x}, {goal_y}) is occupied, searching for nearest free cell'
            )
            free_goal = self._find_nearest_free_cell(goal_x, goal_y, occupancy_grid)
            if free_goal is None:
                self.node.get_logger().error('Could not find free cell near goal position')
                return []
            goal_x, goal_y = free_goal
        
        open_set = []
        heapq.heappush(open_set, (self._heuristic(start_x, start_y, goal_x, goal_y), start_x, start_y))
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self._heuristic(start_x, start_y, goal_x, goal_y)}
        closed_set = set()
        iterations = 0
        max_iterations = width * height * 2
        
        while open_set:
            iterations += 1
            if iterations > max_iterations:
                self.node.get_logger().warn(f'Theta* exceeded max iterations: {max_iterations}')
                break
            
            # Get node with lowest f_score from priority queue
            current_f, current_x, current_y = heapq.heappop(open_set)
            
            if (current_x, current_y) in closed_set:
                continue
            
            closed_set.add((current_x, current_y))
            
            if current_x == goal_x and current_y == goal_y:
                path = []
                while (current_x, current_y) in came_from:
                    path.append((current_x, current_y))
                    current_x, current_y = came_from[(current_x, current_y)]
                path.append((start_x, start_y))
                path.reverse()
                optimized_path = self._optimize_path_with_los(path, occupancy_grid)
                return optimized_path
            
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                
                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    continue
                
                index = neighbor_y * width + neighbor_x
                if occupancy_grid.data[index] > 50:
                    continue
                
                parent = came_from.get((current_x, current_y))
                if parent is not None:
                    parent_x, parent_y = parent
                    if self._line_of_sight(parent_x, parent_y, neighbor_x, neighbor_y, occupancy_grid):
                        tentative_g = g_score[(parent_x, parent_y)] + self._heuristic(
                            parent_x, parent_y, neighbor_x, neighbor_y
                        )
                        if (neighbor_x, neighbor_y) not in g_score or tentative_g < g_score[(neighbor_x, neighbor_y)]:
                            came_from[(neighbor_x, neighbor_y)] = (parent_x, parent_y)
                            g_score[(neighbor_x, neighbor_y)] = tentative_g
                            f_score[(neighbor_x, neighbor_y)] = tentative_g + self._heuristic(
                                neighbor_x, neighbor_y, goal_x, goal_y
                            )
                            if (neighbor_x, neighbor_y) not in closed_set:
                                heapq.heappush(open_set, (f_score[(neighbor_x, neighbor_y)], neighbor_x, neighbor_y))
                        continue
                
                tentative_g = g_score[(current_x, current_y)] + math.sqrt(dx*dx + dy*dy)
                if (neighbor_x, neighbor_y) not in g_score or tentative_g < g_score[(neighbor_x, neighbor_y)]:
                    came_from[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    g_score[(neighbor_x, neighbor_y)] = tentative_g
                    f_score[(neighbor_x, neighbor_y)] = tentative_g + self._heuristic(
                        neighbor_x, neighbor_y, goal_x, goal_y
                    )
                    if (neighbor_x, neighbor_y) not in closed_set:
                        heapq.heappush(open_set, (f_score[(neighbor_x, neighbor_y)], neighbor_x, neighbor_y))
        
        return []
    
    def _line_of_sight(
        self,
        x0: int, y0: int,
        x1: int, y1: int,
        occupancy_grid: OccupancyGrid
    ) -> bool:
        """
        Check if there is a line-of-sight between two grid cells.
        
        Uses Bresenham's line algorithm to check if all cells along the line
        between (x0, y0) and (x1, y1) are free.
        
        Args:
            x0, y0: Start grid coordinates
            x1, y1: End grid coordinates
            occupancy_grid: Occupancy grid
            
        Returns:
            True if line-of-sight exists, False otherwise
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if not (0 <= x < width and 0 <= y < height):
                return False
            
            index = y * width + x
            if occupancy_grid.data[index] > 50:
                return False
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True
    
    def _optimize_path_with_los(self, path: list, occupancy_grid: OccupancyGrid) -> list:
        """
        Optimize path by removing intermediate waypoints that have line-of-sight.
        
        This post-processing step further optimizes the Theta* path by removing
        unnecessary waypoints.
        
        Args:
            path: Original path as list of (x, y) coordinates
            occupancy_grid: Occupancy grid
            
        Returns:
            Optimized path with fewer waypoints
        """
        if len(path) <= 2:
            return path
        
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._line_of_sight(
                    path[i][0], path[i][1],
                    path[j][0], path[j][1],
                    occupancy_grid
                ):
                    optimized.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                optimized.append(path[i + 1])
                i += 1
        
        return optimized
    
    def _create_direct_path(
        self,
        start_pose: PoseStamped,
        goal_pose: PoseStamped
    ) -> Path:
        """
        Create a simple direct path (fallback when planner not available).
        
        Args:
            start_pose: Start pose
            goal_pose: Goal pose
            
        Returns:
            Path with start and goal waypoints
        """
        path = Path()
        path.header.frame_id = goal_pose.header.frame_id
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.poses = [start_pose, goal_pose]
        
        return path

