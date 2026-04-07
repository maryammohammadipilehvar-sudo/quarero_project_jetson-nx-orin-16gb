from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(package_name, file_path, node_key):
    config_file_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(config_file_path, 'r') as f:
        full_cfg = yaml.safe_load(f)
    return full_cfg[node_key]['ros__parameters']

def generate_launch_description():
    package_name = 'drive'
    config_rel_path = 'config/params.yaml'

    robot_params = load_yaml(package_name, config_rel_path, 'robot_controller')
    roboclaw_params = load_yaml(package_name, config_rel_path, 'roboclaw_wrapper')

    return LaunchDescription([
        Node(
            package='drive',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            parameters=[robot_params],
        ),
        Node(
            package='drive',
            executable='roboclaw_wrapper',
            name='roboclaw_drive',
            output='screen',
            parameters=[roboclaw_params],
        ),
    ])
