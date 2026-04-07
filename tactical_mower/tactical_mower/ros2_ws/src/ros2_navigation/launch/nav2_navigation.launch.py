#!/usr/bin/env python3
"""
Launch file for nav2_navigation node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description."""
    # Get package share directory
    package_share = get_package_share_directory('ros2_navigation')
    
    # Path to config file
    config_file = os.path.join(
        package_share,
        'config',
        'nav2_navigation_params.yaml'
    )
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file'
    )
    
    # Create node
    nav2_navigation_node = Node(
        package='ros2_navigation',
        executable='nav2_navigation_node',
        name='nav2_navigation_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        config_file_arg,
        nav2_navigation_node,
    ])

