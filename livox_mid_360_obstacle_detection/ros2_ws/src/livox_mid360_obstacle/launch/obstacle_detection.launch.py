from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    livox_start_delay = LaunchConfiguration('livox_start_delay')

    # Get path to config file
    config_file = os.path.join(
        get_package_share_directory('livox_mid360_obstacle'),
        'config',
        'obstacle_config.yaml'
    )

    declare_livox_start_delay = DeclareLaunchArgument(
        'livox_start_delay',
        default_value='3.0',
        description='Delay before starting livox driver (seconds) - reduced since wait_for_lidar.sh handles initial wait'
    )

    # Include the Livox driver launch file
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'launch',
                'msg_MID360_launch.py'
            )
        ])
    )

    return LaunchDescription([
        declare_livox_start_delay,
        TimerAction(period=livox_start_delay, actions=[livox_driver_launch]),
        Node(
            package='livox_mid360_obstacle',
            executable='livox_obstacle_node',
            name='livox_obstacle_node',
            output='screen',
            parameters=[config_file],
            # arguments=['--ros-args', '--log-level', 'debug'],
        )
    ])


