from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('eneo_ip_therm_camera'),
        'config',
        'thermal_camera.yml'
    )

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    def axis_url(ip: str, port: int, channel: int) -> str:
        return (
            f"rtsp://Quarero:quarero00@{ip}:{port}/axis-media/media.amp"
            f"?camera={channel}&audio=0"
        )

    ip = cfg["ip"]
    port = cfg.get("port", 554)

    rgb_stream_url = axis_url(
        ip=ip,
        port=port,
        channel=int(cfg["channel"]["rgb"])
    )

    thermal_stream_url = axis_url(
        ip=ip,
        port=port,
        channel=int(cfg["channel"]["thermal"])
    )

    node_rgb_stream = Node(
        package='eneo_ip_therm_camera',
        executable='rtsp_image_publisher',
        name='rgb_camera_stream',
        parameters=[
            {'rtsp_url': rgb_stream_url},
            {'frame_rate': 10.0},
            {'topic_name': 'rtsp_camera/image_raw'}
        ],
        remappings=[('rtsp_camera/image_raw', 'ip_camera/rgb_raw')],
        output='screen'
    )

    node_thermal_stream = Node(
        package='eneo_ip_therm_camera',
        executable='rtsp_image_publisher',
        name='thermal_camera_stream',
        parameters=[
            {'rtsp_url': thermal_stream_url},
            {'frame_rate': 10.0},
            {'topic_name': 'rtsp_camera/image_raw'}
        ],
        remappings=[('rtsp_camera/image_raw', 'ip_camera/thermal_raw')],
        output='screen'
    )

    return LaunchDescription([
        node_rgb_stream,
        #node_thermal_stream,
    ])