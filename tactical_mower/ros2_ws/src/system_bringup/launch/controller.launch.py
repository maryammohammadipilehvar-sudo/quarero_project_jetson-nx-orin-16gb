from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for system bringup.
    
    Launches all required ROS2 nodes for robot operation:
    - Drive system (robot_controller, roboclaw_wrapper)
    - Control system (joy_controller, tactical_wp_follower, tactical_scheduler)
    - GPS/localization (fixposition_driver)
    - Video ringbuffer for security events
    - Eneo event publisher for security alerts
    """
    # Launch arguments (forwarded into included launches where supported)
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)',
    )

    # drive.launch.py - starts robot_controller and roboclaw_wrapper
    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drive'),
                'launch',
                'drive.launch.py'
            )
        )
    )

    # control.launch.py - starts joy_controller, tactical_wp_follower, tactical_scheduler, and Nav2
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('control'),
                'launch',
                'control.launch.py'
            )
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    # fixposition_driver_ros2 node.launch (XML launch file)
    fixpos_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fixposition_driver_ros2'),
                'launch',
                'node.launch'
            )
        )
    )

    # video_ringbuffer node — captures per-camera pre/post-event MP4 clips on /capture_event_clips
    # storage_root MUST be /routen/security_events so clips land on the host bind-mount, not
    # the container's ephemeral filesystem. AUDIT HIGH-2 was that this node was disabled
    # entirely; re-enabled 2026-05-23 as Phase 1 of the arrival-pipeline.
    # See DESIGN_ARRIVAL_PIPELINE.md §5.4.
    video_ringbuffer_node = Node(
        package='video_ringbuffer',
        executable='ringbuffer_node',
        name='video_ringbuffer',
        output='screen',
        parameters=[{
            'storage_root': '/routen/security_events',
            'max_total_size_mb': 20480.0,
            'pre_event_seconds': 10.0,
            'post_event_seconds': 10.0,
            'max_buffer_memory_mb': 1024.0,
            'max_frames_per_camera': 300,
        }]
    )

    # eneo_event_publisher node - receives UDP events from Eneo cameras and publishes SecurityAlert
    eneo_event_publisher_node = Node(
        package='eneo_event_publisher',
        executable='eneo_event_publisher',
        name='eneo_event_publisher',
        output='screen',
        parameters=[{
            # 'udp_port': 5002,
            # 'camera_ip': '192.168.10.128',
            # 'buffer_size': 4096,
            # 'filter_start_only': True,  # Only process "start" events
        }]
    )

    return LaunchDescription([
        log_level_arg,
        drive_launch,
        control_launch,
        fixpos_launch,
        video_ringbuffer_node,
        eneo_event_publisher_node,
    ])