from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get path to config file
    config_file = os.path.join(
        get_package_share_directory('control'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),

        # Joy Controller Node
        Node(
            package='control',
            executable='joy_controller',
            name='joy_controller',
            output='screen',
            parameters=[],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Tactical Waypoint Follower Node
        Node(
            package='control',
            executable='tactical_wp_follower',
            name='tactical_wp_follower',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Nav2 Navigation Node (always started - follower selection is dynamic via obstacle_avoidance setting)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros2_navigation'),
                    'launch',
                    'nav2_navigation.launch.py'
                )
            )
        ),

        # Tactical Scheduler Node
        Node(
            package='control',
            executable='tactical_scheduler',
            name='tactical_scheduler',
            output='screen',
            parameters=[],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Sector Publisher Node
        Node(
            package='control',
            executable='sector_publisher',
            name='sector_publisher',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Sector Visualization Node (RViz markers)
        Node(
            package='control',
            executable='sector_viz',
            name='sector_viz',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Obstacle Visualization Node
        Node(
            package='control',
            executable='obstacle_viz',
            name='obstacle_viz',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])