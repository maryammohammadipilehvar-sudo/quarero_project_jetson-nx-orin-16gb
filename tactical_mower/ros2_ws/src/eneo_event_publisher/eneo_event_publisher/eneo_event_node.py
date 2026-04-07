#!/usr/bin/env python3
"""
Eneo Event Publisher Node
=========================
Receives UDP events from Eneo cameras (port 5002) and publishes them as SecurityAlert messages.
Only "person" and "fire" events are published, with rate limiting (max. every 200ms).
"""

import time
import rclpy
from rclpy.node import Node
from interfaces.msg import SecurityAlert

from .eneo_event import EneoEvent
from .eneo_mapping import ENEO_TO_SECURITY_TYPE
from .eneo_parser import eneo_time_to_ros_time, map_device_name
from .udp_listener import UDPListener

try:
    # Import shared camera configuration from video_ringbuffer package
    from video_ringbuffer.camera_config import EVENT_RECORDING_CAMERA_IDS
except ImportError:
    # Fallback if video_ringbuffer is not available (should not happen in normal operation)
    EVENT_RECORDING_CAMERA_IDS = ['eneo_rgb', 'eneo_thermal']
    

# Allowed event types to be published
ALLOWED_EVENT_TYPES = {"FireDetect", "PersonDetect"}


class EneoEventPublisherNode(Node):
    """ROS2 Node that receives Eneo UDP events and publishes them as SecurityAlert messages."""
    
    def __init__(self):
        super().__init__('eneo_event_publisher')
        
        # Parameter
        self.declare_parameter('udp_port', 5002)
        self.declare_parameter('camera_ip', '192.168.10.128')
        self.declare_parameter('buffer_size', 4096)
        self.declare_parameter('filter_start_only', True)  # Only process "start" events
        
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.camera_ip = self.get_parameter('camera_ip').get_parameter_value().string_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.filter_start_only = self.get_parameter('filter_start_only').get_parameter_value().bool_value
        
        # Publisher for SecurityAlert
        self.alert_pub = self.create_publisher(
            SecurityAlert,
            '/security_alert',
            10
        )
        
        # Rate Limiting: Last publication time (in seconds)
        self.last_publish_time = 0.0
        self.min_publish_interval = 0.2  # 200ms in seconds
        
        # UDP Listener
        self.udp_listener = UDPListener(
            port=self.udp_port,
            buffer_size=self.buffer_size,
            callback=self._handle_eneo_event,
            logger=self.get_logger(),
            camera_ip_filter=self.camera_ip
        )
        
        self.get_logger().info(
            f'Eneo Event Publisher started - UDP Port: {self.udp_port}, '
            f'Camera IP: {self.camera_ip}, Rate Limit: {self.min_publish_interval*1000:.0f}ms'
        )
        
        # Start UDP Listener
        self.udp_listener.start()
    
    def _handle_eneo_event(self, event: EneoEvent):
        """
        Process an Eneo event and publish it as SecurityAlert.
        
        Args:
            event: EneoEvent object
        """
        # Only process allowed event types (FireDetect, PersonDetect)
        if event.event_type not in ALLOWED_EVENT_TYPES:
            self.get_logger().debug(
                f'Event type "{event.event_type}" ignored (only FireDetect/PersonDetect allowed)'
            )
            return
        
        # Only process "start" events if filter_start_only is enabled
        if self.filter_start_only and not event.is_start:
            return
        
        # Rate Limiting: Check if enough time has passed since last publication
        current_time = time.time()
        time_since_last = current_time - self.last_publish_time
        if time_since_last < self.min_publish_interval:
            self.get_logger().debug(
                f'Event ignored (Rate Limit: {time_since_last*1000:.1f}ms < {self.min_publish_interval*1000:.0f}ms)'
            )
            return
        
        # Create SecurityAlert message
        alert = SecurityAlert()
        
        # Map Event Type (should now always be "fire" or "person")
        alert.event_type = ENEO_TO_SECURITY_TYPE.get(event.event_type, 'unknown')
        
        # Safety check: Should never be "unknown" since we already filtered
        if alert.event_type == 'unknown':
            self.get_logger().warn(
                f'Unexpected event type "{event.event_type}" after filtering - ignoring'
            )
            return
        
        # Convert time
        alert.event_time = eneo_time_to_ros_time(event.event_time)
        
        # Device Name
        alert.device_name = map_device_name(event.device_name, event.ip_address)
        
        # Description
        channel_str = ', '.join(event.channels) if event.channels else 'none'
        alert.description = (
            f'Eneo Event: {event.event_type} ({event.event_action}) - '
            f'Camera: {event.device_name}, Channels: {channel_str}, IP: {event.ip_address}'
        )
        
        # Camera IDs - Standard cameras that should be recorded
        # Uses shared configuration from video_ringbuffer package
        alert.camera_ids = list(EVENT_RECORDING_CAMERA_IDS)
        
        # Publish Alert
        self.alert_pub.publish(alert)
        
        # Update Rate Limiting Timestamp
        self.last_publish_time = current_time
        
        self.get_logger().info(
            f'Security Alert published: type={alert.event_type}, '
            f'device={alert.device_name}, action={event.event_action}'
        )
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.udp_listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EneoEventPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
