#!/usr/bin/env python3
"""
Path Publisher for publishing planned paths for visualization.

This module publishes planned paths to a topic for visualization
in RViz or other tools.
"""

from rclpy.node import Node
from nav_msgs.msg import Path


class PathPublisher:
    """
    Publishes planned paths for visualization.
    """
    
    def __init__(self, node: Node, topic_name: str):
        """
        Initialize Path Publisher.
        
        Args:
            node: ROS2 node instance
            topic_name: Name of the topic to publish paths to
        """
        self.node = node
        self.topic_name = topic_name
        
        # Create publisher
        self.publisher = self.node.create_publisher(
            Path,
            topic_name,
            10
        )
        
        self.node.get_logger().info(
            f'PathPublisher initialized, publishing to: {topic_name}'
        )
    
    def publish_path(self, path: Path):
        """
        Publish a planned path.
        
        Args:
            path: Path message to publish
        """
        if path is None:
            self.node.get_logger().warn('Cannot publish: path is None')
            return
        
        if len(path.poses) == 0:
            self.node.get_logger().warn('Cannot publish: path has 0 waypoints')
            return
        
        # Update header timestamp
        path.header.stamp = self.node.get_clock().now().to_msg()
        
        self.publisher.publish(path)
        
        self.node.get_logger().debug(
            f'Published path: {len(path.poses)} waypoints, frame: {path.header.frame_id}, '
            f'to {self.topic_name}'
        )

