#!/usr/bin/env python3
"""
Inflated Grid Publisher for publishing inflated occupancy grids for visualization.

This module publishes inflated occupancy grids to a topic for visualization
in RViz or other tools.
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from ros2_navigation.grid_utils import validate_grid


class InflatedGridPublisher:
    """
    Publishes inflated occupancy grids for visualization.
    """
    
    def __init__(self, node: Node, topic_name: str):
        """
        Initialize Inflated Grid Publisher.
        
        Args:
            node: ROS2 node instance
            topic_name: Name of the topic to publish inflated grids to
        """
        self.node = node
        self.topic_name = topic_name
        
        # QoS Profile optimized for visualization (RViz)
        # RELIABLE: Required for RViz compatibility (RViz uses RELIABLE by default)
        # TRANSIENT_LOCAL: RViz gets last message even if started late
        # depth=1: Only keep latest message (sufficient for visualization)
        visualization_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Create publisher
        self.publisher = self.node.create_publisher(
            OccupancyGrid,
            topic_name,
            visualization_qos
        )
        
        self.node.get_logger().info(
            f'InflatedGridPublisher initialized, publishing to: {topic_name}'
        )
    
    def publish_inflated_grid(self, inflated_grid: OccupancyGrid):
        """
        Publish an inflated occupancy grid.
        
        Args:
            inflated_grid: Inflated OccupancyGrid message to publish
        """
        if inflated_grid is None:
            self.node.get_logger().warn('Cannot publish: inflated_grid is None')
            return
        
        if not validate_grid(inflated_grid):
            self.node.get_logger().warn('Cannot publish: invalid inflated_grid')
            return
        
        # Update header timestamp
        inflated_grid.header.stamp = self.node.get_clock().now().to_msg()
        
        self.publisher.publish(inflated_grid)
        
        self.node.get_logger().debug(
            f'Published inflated grid: {inflated_grid.info.width}x{inflated_grid.info.height}, '
            f'frame: {inflated_grid.header.frame_id}, to {self.topic_name}'
        )

