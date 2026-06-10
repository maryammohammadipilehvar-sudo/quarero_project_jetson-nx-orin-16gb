from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    oak_launch_file = os.path.join(
        get_package_share_directory('depthai_ros_driver'),
        'launch',
        'camera.launch.py'
    )

    obstacle_params = os.path.join(
        get_package_share_directory('realsense_obstacle'),
        'config',
        'params.yaml'
    )

    # OAK-D Lite stereo depth, aligned to RGB. depthai_ros_driver
    # publishes the depth image on /oak/stereo/image_raw (32FC1, meters).
    oak = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oak_launch_file),
        launch_arguments={
            'camera_model': 'OAK-D-LITE',
            'name': 'oak',
            'params_file': obstacle_params,
        }.items()
    )

    obstacle_node = Node(
        package='realsense_obstacle',
        executable='simple_obstacle_detector',
        name='sector_obstacle_detector',
        output='screen',
        parameters=[obstacle_params]
    )

    # robot_web_interface subscribes to the legacy RealSense RGB topic
    # /camera/camera/color/image_raw. Relay the OAK's rectified RGB
    # output so the web UI's RGB pane keeps working without touching
    # the web app.
    rgb_relay = Node(
        package='topic_tools',
        executable='relay',
        name='oak_rgb_relay',
        arguments=['/oak/rgb/image_rect', '/camera/camera/color/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        oak,
        obstacle_node,
        rgb_relay,
    ])