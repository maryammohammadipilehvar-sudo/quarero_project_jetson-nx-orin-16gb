from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():
    # Load config
    config_path = os.path.join(
        get_package_share_directory('eneo_ip_therm_camera'),
        'config',
        'thermal_camera.yml'
    )

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # Keep SAME placeholders: ip, port, channel, subtype
    # For AXIS:
    #   channel -> camera=<channel>
    #   subtype -> we emulate stream variants using query parameters (optional)
    template = (
        "rtsp://root:axis@{ip}:{port}/axis-media/media.amp"
        "?camera={channel}&subtype={subtype}"
    )

    def axis_url(ip: str, port: int, channel: int, subtype: int) -> str:
        """
        subtype mapping (emulated):
          0 = main/high quality
          1 = substream/low quality
          2 = mobile/very low quality

        AXIS doesn't have 'subtype', but adding it as a query param is harmless.
        We also optionally add resolution/fps hints based on subtype.
        If your camera ignores resolution/fps params, stream will still work.
        """
        base = template.format(ip=ip, port=port, channel=channel, subtype=subtype)

        # Optional quality control by subtype (feel free to edit/remove)
        if subtype == 0:
            # main
            return base + "&fps=25"
        elif subtype == 1:
            # substream
            return base + "&resolution=640x360&fps=10"
        elif subtype == 2:
            # mobile
            return base + "&resolution=320x180&fps=10"
        else:
            return base

    ip = cfg["ip"]
    port = cfg.get("port", 554)

    # URLs for Web-App streaming (lower quality)
    rgb_stream_url = axis_url(
        ip=ip,
        port=port,
        channel=int(cfg["channel"]["rgb"]),   # AXIS camera input ID
        subtype=2
    )

    thermal_stream_url = axis_url(
        ip=ip,
        port=port,
        channel=int(cfg["channel"]["thermal"]),  # AXIS camera input ID
        subtype=1
    )

    # URLs for Event recording (higher quality)
    rgb_record_url = axis_url(
        ip=ip,
        port=port,
        channel=int(cfg["channel"]["rgb"]),
        subtype=0
    )

    # Nodes for Web-App streaming
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

    # Optional: recording node (uncomment if you want it)
    # node_rgb_record = Node(
    #     package='eneo_ip_therm_camera',
    #     executable='rtsp_image_publisher',
    #     name='rgb_camera_record',
    #     parameters=[
    #         {'rtsp_url': rgb_record_url},
    #         {'frame_rate': 25.0},
    #         {'topic_name': 'rtsp_camera/image_raw'}
    #     ],
    #     remappings=[('rtsp_camera/image_raw', 'ip_camera/rgb_raw_record')],
    #     output='screen'
    # )

    return LaunchDescription([
        node_rgb_stream,
        node_thermal_stream,
        # node_rgb_record,
    ])