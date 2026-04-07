from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    obstacle_params = os.path.join(
        get_package_share_directory('realsense_obstacle'),
        'config',
        'params.yaml'
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',                          # ← enable depth
            'rgb_camera.color_profile': '1280x720x30',       # ← color stream
            'depth_module.depth_profile': '640x480x30',      # ← depth stream
            'align_depth.enable': 'true',                    # ← align to color frame
            'spatial_filter.enable': 'true',                 # ← smooth depth
            'temporal_filter.enable': 'true',                # ← reduce noise
            'hole_filling_filter.enable': 'true',            # ← fill gaps
        }.items()
    )

    obstacle_node = Node(
        package='realsense_obstacle',
        executable='simple_obstacle_detector',
        name='sector_obstacle_detector',
        output='screen',
        parameters=[obstacle_params]
    )

    return LaunchDescription([
        realsense,
        obstacle_node,    # ← uncommented
    ])