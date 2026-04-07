#!/usr/bin/env python3
"""
Extended Grid Publisher for publishing extended occupancy grids for visualization.

This module publishes extended occupancy grids to a topic for visualization
in RViz or other tools.
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from ros2_navigation.grid_utils import validate_grid


class ExtendedGridPublisher:
    """
    Publishes extended occupancy grids for visualization.
    """
    
    def __init__(self, node: Node, topic_name: str):
        """
        Initialize Extended Grid Publisher.
        
        Args:
            node: ROS2 node instance
            topic_name: Name of the topic to publish extended grids to
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
            f'ExtendedGridPublisher initialized, publishing to: {topic_name}'
        )
    
    def publish_extended_grid(self, extended_grid: OccupancyGrid):
        """
        Publish an extended occupancy grid.
        
        Args:
            extended_grid: Extended OccupancyGrid message to publish
        """
        if extended_grid is None:
            self.node.get_logger().warn('Cannot publish: extended_grid is None')
            return
        
        if not validate_grid(extended_grid):
            self.node.get_logger().warn('Cannot publish: invalid extended_grid')
            return
        
        # Update header timestamp
        extended_grid.header.stamp = self.node.get_clock().now().to_msg()
        
        self.publisher.publish(extended_grid)
        
        self.node.get_logger().debug(
            f'Published extended grid: {extended_grid.info.width}x{extended_grid.info.height}, '
            f'frame: {extended_grid.header.frame_id}, to {self.topic_name}'
        )


