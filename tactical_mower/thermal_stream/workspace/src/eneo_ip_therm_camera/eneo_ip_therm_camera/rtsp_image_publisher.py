import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import time


class RTSPImagePublisher(Node):
    """ROS2 Node zum Publizieren eines RTSP-Streams als sensor_msgs/Image."""

    def __init__(self):
        super().__init__('rtsp_image_publisher')

        # RTSP-URL als Parameter
        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('frame_rate', 10.0)  # 30 FPS for live streaming (increased from 10.0)
        self.declare_parameter('topic_name', 'rtsp_camera/image_raw')
        
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        frame_rate = float(self.get_parameter('frame_rate').value)
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # Zum Debuggen: URL einmal loggen
        self.get_logger().info(f"Using RTSP URL: {rtsp_url}, Frame rate: {frame_rate} FPS, Topic: {topic_name}")

        # Publisher mit optimierten QoS-Einstellungen für Streaming
        # KEEP_LAST mit depth=1: Kein Buffering - nur das neueste Frame wird behalten
        # CRITICAL: depth=1 verhindert Buffering und stellt sicher, dass nur das aktuellste Frame gesendet wird
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Nur 1 Frame - kein Buffering, immer das neueste Frame
        )
        self.publisher_ = self.create_publisher(Image, topic_name, camera_qos)
        self.bridge = CvBridge()

        # RTSP-Stream öffnen
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        
        # CRITICAL: Buffer-Size auf 1 setzen um alte Frames zu verwerfen
        # Das verhindert, dass alte Frames im OpenCV-Buffer bleiben
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Zusätzliche RTSP-Optimierungen für niedrige Latenz
        self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 1000)
        
        # Ensure we always get the latest frame by using grab() + retrieve() pattern
        # This is more reliable than read() for getting the absolute newest frame

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open RTSP stream: {rtsp_url}")
            raise RuntimeError("Failed to open RTSP stream")

        # Timer frequency: Poll at frame_rate for optimal latency
        # Direct frame_rate polling ensures we catch every frame without delay
        timer_interval = 1.0 / frame_rate if frame_rate > 0 else 0.033
        self.timer = self.create_timer(timer_interval, self.frame_callback)
        
        # Track last published frame timestamp to avoid publishing duplicates
        self.last_published_timestamp = None
        
        # Track last frame receive time for simple duplicate detection (fast, no MD5)
        self.last_frame_time = 0.0
        
        self.get_logger().info(f"RTSPImagePublisher initialized - QoS depth=1 (no buffering, latest frame only), buffer size: 1, polling: {1.0/timer_interval:.2f}Hz (frame_rate: {frame_rate} FPS)")

    def frame_callback(self):
        """Reads the newest frame from RTSP stream and publishes it as ROS Image.
        
        CRITICAL: Flush OpenCV buffer to get the absolute newest frame.
        Even with CAP_PROP_BUFFERSIZE=1, FFmpeg may buffer frames internally.
        We use grab() multiple times to discard old frames, then retrieve() 
        to decode only the newest one - this is much faster than read() multiple times.
        
        NOTE: We always publish to ensure subscribers can connect and receive data.
        """
        
        # CRITICAL: Flush buffer by grabbing (but not decoding) frames
        # grab() is much faster than read() because it doesn't decode
        # This discards old buffered frames to get the newest one
        for _ in range(3):  # Flush up to 3 old frames
            if not self.cap.grab():
                break
        
        # Now retrieve (decode) only the newest frame
        ret, frame = self.cap.retrieve()
        if not ret or frame is None:
            # Try direct read as fallback
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return
        
        # Get current timestamp and publish immediately
        # No duplicate detection needed - always publish the newest frame
        # The timer interval controls the frame rate, and QoS depth=1 on the
        # subscriber side ensures only the newest frame is kept
        current_timestamp = self.get_clock().now()
        
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = current_timestamp.to_msg()
            msg.header.frame_id = "rtsp_camera"
            self.publisher_.publish(msg)
            self.last_frame_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")


def main(args=None):
    """Initialisiert rclpy, startet den Node und räumt beim Beenden auf."""
    rclpy.init(args=args)

    try:
        node = RTSPImagePublisher()
    except Exception as e:
        print(f"Failed to start RTSPImagePublisher: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, "cap") and node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

