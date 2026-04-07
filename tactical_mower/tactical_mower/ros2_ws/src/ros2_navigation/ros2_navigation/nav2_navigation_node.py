#!/usr/bin/env python3
"""
Main navigation node for ROS2 navigation package.

This node orchestrates all components for path planning.
"""

import time
import rclpy
from rclpy.node import Node
from ros2_navigation.tf_manager import TFManager
from ros2_navigation.occupancy_grid_subscriber import OccupancyGridSubscriber
from ros2_navigation.planner_manager import PlannerManager
from ros2_navigation.path_planning_service import PathPlanningService
from ros2_navigation.replanning_service import ReplanningService
from ros2_navigation.path_publisher import PathPublisher
from ros2_navigation.inflated_grid_publisher import InflatedGridPublisher
from ros2_navigation.extended_grid_publisher import ExtendedGridPublisher
from ros2_navigation.inflation_layer import InflationLayer
from nav_msgs.msg import OccupancyGrid


class Nav2NavigationNode(Node):
    """
    Main navigation node that orchestrates all components.
    """
    
    def __init__(self):
        """Initialize navigation node and all components."""
        super().__init__('nav2_navigation_node')
        
        self._declare_parameters()
        params = self._get_parameters()
        self._initialize_components(params)
        
        # Log frame configuration for clarity
        self.get_logger().info('Nav2NavigationNode initialized')
        self.get_logger().info(
            f'[FRAME] Frame configuration: '
            f'robot_base_frame={params["robot_base_frame"]}, '
            f'global_frame={params["global_frame"]}, '
            f'auto_transform_grid={params["auto_transform_grid"]}'
        )
        if params['auto_transform_grid']:
            self.get_logger().info(
                f'[FRAME] Grids will be automatically transformed to: {params["global_frame"]}'
            )
    
    def _declare_parameters(self):
        """Declare all node parameters."""
        # Topics
        self.declare_parameter('occupancy_grid_topic', 'lidar/occupancy_grid')
        self.declare_parameter('planned_path_topic', 'planned_path')
        
        # Frames
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('global_frame', 'odom')
        
        # Planner
        self.declare_parameter('planner_plugin', 'nav2_smac_planner::SmacPlanner2D')
        self.declare_parameter('planner_frequency', 1.0)
        
        # Path Planning Algorithm
        self.declare_parameter('algorithm_type', 'astar')
        
        # Grid Extension
        self.declare_parameter('grid_extension_distance', 5.0)
        self.declare_parameter('grid_extension_resolution', 0.1)
        
        # Error Handling
        self.declare_parameter('max_planning_time', 5.0)
        self.declare_parameter('tf_timeout', 1.0)
        
        # Obstacle Avoidance
        self.declare_parameter('inflation_radius', 0.0)  # meters - distance to keep from obstacles (0.0 to disable)
        self.declare_parameter('inflated_grid_topic', 'inflated_occupancy_grid')  # Topic for RViz visualization
        self.declare_parameter('extended_grid_topic', 'extended_occupancy_grid')  # Topic for RViz visualization of extended grids
        self.declare_parameter('inflated_grid_update_rate', 2.0)  # Hz - rate for continuous inflated grid updates
        
        # Replanning Service
        self.declare_parameter('replanning_service_name', 'replan_path')
        self.declare_parameter('replanning_grid_topic', 'inflated_occupancy_grid')  # Grid topic for replanning (use inflated grid for safety margin)
        
        # Grid Transformation
        self.declare_parameter('auto_transform_grid', True)  # Automatically transform grids to global_frame
    
    def _get_parameters(self) -> dict:
        """Get all parameters as dictionary."""
        return {
            'occupancy_grid_topic': self.get_parameter('occupancy_grid_topic').get_parameter_value().string_value,
            'planned_path_topic': self.get_parameter('planned_path_topic').get_parameter_value().string_value,
            'robot_base_frame': self.get_parameter('robot_base_frame').get_parameter_value().string_value,
            'global_frame': self.get_parameter('global_frame').get_parameter_value().string_value,
            'planner_plugin': self.get_parameter('planner_plugin').get_parameter_value().string_value,
            'planner_frequency': self.get_parameter('planner_frequency').get_parameter_value().double_value,
            'algorithm_type': self.get_parameter('algorithm_type').get_parameter_value().string_value,
            'grid_extension_distance': self.get_parameter('grid_extension_distance').get_parameter_value().double_value,
            'grid_extension_resolution': self.get_parameter('grid_extension_resolution').get_parameter_value().double_value,
            'max_planning_time': self.get_parameter('max_planning_time').get_parameter_value().double_value,
            'tf_timeout': self.get_parameter('tf_timeout').get_parameter_value().double_value,
            'inflation_radius': self.get_parameter('inflation_radius').get_parameter_value().double_value,
            'inflated_grid_topic': self.get_parameter('inflated_grid_topic').get_parameter_value().string_value,
            'extended_grid_topic': self.get_parameter('extended_grid_topic').get_parameter_value().string_value,
            'replanning_service_name': self.get_parameter('replanning_service_name').get_parameter_value().string_value,
            'replanning_grid_topic': self.get_parameter('replanning_grid_topic').get_parameter_value().string_value,
            'auto_transform_grid': self.get_parameter('auto_transform_grid').get_parameter_value().bool_value,
            'inflated_grid_update_rate': self.get_parameter('inflated_grid_update_rate').get_parameter_value().double_value,
        }
    
    def _initialize_components(self, params: dict):
        """Initialize all components."""
        self.inflation_radius = params['inflation_radius']
        
        self.tf_manager = TFManager(
            self,
            params['robot_base_frame'],
            params['global_frame'],
            params['tf_timeout']
        )
        
        # Determine if grid transformation should be enabled
        auto_transform_frame = None
        if params['auto_transform_grid']:
            auto_transform_frame = params['global_frame']
        
        self.grid_subscriber = OccupancyGridSubscriber(
            self,
            params['occupancy_grid_topic'],
            grid_update_callback=self._on_grid_update if params['inflation_radius'] > 0.0 else None,
            auto_transform_to_frame=auto_transform_frame,
            tf_buffer=self.tf_manager.tf_buffer if params['auto_transform_grid'] else None,
            tf_timeout=params['tf_timeout']
        )
        
        # Separate grid subscriber for replanning (uses inflated grid for safety margin)
        self.replanning_grid_subscriber = OccupancyGridSubscriber(
            self,
            params['replanning_grid_topic'],
            grid_update_callback=None,  # No callback needed for replanning grid
            auto_transform_to_frame=auto_transform_frame,
            tf_buffer=self.tf_manager.tf_buffer if params['auto_transform_grid'] else None,
            tf_timeout=params['tf_timeout']
        )
        
        self.path_publisher = PathPublisher(
            self,
            params['planned_path_topic']
        )
        
        self.inflated_grid_publisher = InflatedGridPublisher(
            self,
            params['inflated_grid_topic']
        )
        
        self.extended_grid_publisher = ExtendedGridPublisher(
            self,
            params['extended_grid_topic']
        )
        
        self.planner_manager = PlannerManager(
            self,
            params['planner_plugin'],
            {},
            params['algorithm_type'],
            params['inflation_radius']
        )
        
        self.path_planning_service = PathPlanningService(
            self,
            self.tf_manager,
            self.planner_manager,
            self.grid_subscriber,
            self.path_publisher,
            self.inflated_grid_publisher,
            self.extended_grid_publisher,
            params['grid_extension_distance'],
            params['grid_extension_resolution'],
            params['max_planning_time']
        )
        
        self.replanning_service = ReplanningService(
            self,
            self.tf_manager,
            self.path_planning_service,
            self.replanning_grid_subscriber,  # Use separate grid subscriber for replanning (inflated grid)
            params['replanning_service_name'],
            params['tf_timeout'],
            params['grid_extension_distance'],
            params['grid_extension_resolution']
        )
        
        # Create timer for continuous inflated grid updates (if inflation is enabled)
        if params['inflation_radius'] > 0.0 and params['inflated_grid_update_rate'] > 0.0:
            update_period = 1.0 / params['inflated_grid_update_rate']
            self.inflated_grid_timer = self.create_timer(
                update_period,
                self._update_inflated_grid
            )
            self.get_logger().info(
                f'Inflated grid continuous update enabled: {params["inflated_grid_update_rate"]} Hz'
            )
        else:
            self.inflated_grid_timer = None
    
    def _on_grid_update(self, grid: OccupancyGrid):
        """
        Callback called when a new occupancy grid is received.
        Inflates and publishes the grid for RViz visualization.
        
        Note: This is called when a new grid arrives, but continuous updates
        are handled by _update_inflated_grid() timer.
        
        Args:
            grid: New occupancy grid
        """
        # Grid update callback is kept for immediate updates, but continuous
        # updates are handled by timer to ensure RViz stays synchronized
        if self.inflation_radius <= 0.0:
            return
        
        try:
            inflated_grid = InflationLayer.inflate_grid(grid, self.inflation_radius)
            self.inflated_grid_publisher.publish_inflated_grid(inflated_grid)
        except Exception as e:
            self.get_logger().warn(f'Failed to inflate and publish grid: {e}')
    
    def _update_inflated_grid(self):
        """
        Timer callback for continuous inflated grid updates.
        Ensures RViz always has the latest inflated grid, even if grid updates are infrequent.
        """
        if self.inflation_radius <= 0.0:
            return
        
        # Get current grid
        current_grid = self.grid_subscriber.get_current_grid()
        if current_grid is None:
            return  # No grid available yet
        
        try:
            inflated_grid = InflationLayer.inflate_grid(current_grid, self.inflation_radius)
            self.inflated_grid_publisher.publish_inflated_grid(inflated_grid)
        except Exception as e:
            self.get_logger().debug(f'Failed to update inflated grid: {e}')


def main(args=None):
    """Main function to start the navigation node."""
    rclpy.init(args=args)
    
    node = Nav2NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

