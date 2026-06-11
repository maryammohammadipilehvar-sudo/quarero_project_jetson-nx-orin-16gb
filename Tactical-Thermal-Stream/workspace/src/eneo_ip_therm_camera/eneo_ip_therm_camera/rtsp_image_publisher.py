import os
import time
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class RTSPImagePublisher(Node):
    """RTSP -> ROS2 Image publisher with background reader thread."""

    def __init__(self):
        super().__init__('rtsp_image_publisher')

        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('frame_rate', 10.0)
        self.declare_parameter('topic_name', 'rtsp_camera/image_raw')
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('open_timeout_msec', 5000)
        self.declare_parameter('resize_width', 1280)
        self.declare_parameter('resize_height', 720)

        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.reconnect_delay_sec = float(self.get_parameter('reconnect_delay_sec').value)
        self.open_timeout_msec = int(self.get_parameter('open_timeout_msec').value)
        self.resize_width = int(self.get_parameter('resize_width').value)
        self.resize_height = int(self.get_parameter('resize_height').value)

        if not self.rtsp_url:
            raise RuntimeError('rtsp_url parameter is empty')

        self.get_logger().info(
            f"Using RTSP URL: {self.rtsp_url}, "
            f"Frame rate: {self.frame_rate} FPS, "
            f"Topic: {self.topic_name}"
        )

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher_ = self.create_publisher(Image, self.topic_name, camera_qos)
        self.bridge = CvBridge()

        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;tcp|stimeout;{self.open_timeout_msec * 1000}"
        )

        self.cap = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_frame_ts = 0.0
        self.running = True
        self.consecutive_failures = 0
        self.max_failures_before_reconnect = 20

        self._connect_stream(blocking=True)

        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        timer_interval = 1.0 / self.frame_rate if self.frame_rate > 0 else 0.1
        self.timer = self.create_timer(timer_interval, self.frame_callback)

        self.get_logger().info(
            f"RTSPImagePublisher initialized - QoS depth=1, publish rate: {1.0 / timer_interval:.2f}Hz"
        )

    def _release_stream(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception as e:
                self.get_logger().warning(f"Error releasing stream: {e}")
            finally:
                self.cap = None

    def _connect_stream(self, blocking: bool):
        attempt = 0

        while self.running and rclpy.ok():
            attempt += 1
            self._release_stream()

            self.get_logger().info(
                f"Connecting to RTSP stream (attempt {attempt}): {self.rtsp_url}"
            )

            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            try:
                cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self.open_timeout_msec)
            except Exception:
                pass

            if cap.isOpened():
                self.cap = cap
                self.consecutive_failures = 0
                self.get_logger().info("RTSP stream connected successfully")
                return True

            try:
                cap.release()
            except Exception:
                pass

            if not blocking:
                self.get_logger().warning("RTSP connection failed")
                return False

            self.get_logger().warning(
                f"RTSP connection failed, retrying in {self.reconnect_delay_sec:.1f}s"
            )
            time.sleep(self.reconnect_delay_sec)

        return False

    def _reader_loop(self):
        while self.running and rclpy.ok():
            # Lazy mode: pause the whole decode pipeline when no ROS subscriber
            # is listening. Releases cv2.VideoCapture (stops H.264 decoding) and
            # sleeps until a subscriber connects.
            try:
                sub_count = self.publisher_.get_subscription_count()
            except Exception:
                sub_count = 1  # fall back to always-on if API breaks
            if sub_count == 0:
                if self.cap is not None and self.cap.isOpened():
                    self.get_logger().info(
                        'No subscribers — releasing RTSP capture to save CPU'
                    )
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                time.sleep(0.5)
                continue

            if self.cap is None or not self.cap.isOpened():
                self._connect_stream(blocking=False)
                time.sleep(self.reconnect_delay_sec)
                continue

            ret, frame = self.cap.read()

            if not ret or frame is None:
                self.consecutive_failures += 1

                if self.consecutive_failures >= self.max_failures_before_reconnect:
                    self.get_logger().warning(
                        f"Too many frame read failures ({self.consecutive_failures}), reconnecting stream"
                    )
                    self._connect_stream(blocking=False)
                    time.sleep(self.reconnect_delay_sec)
                else:
                    time.sleep(0.02)

                continue

            self.consecutive_failures = 0

            # Downscale to reduce CPU and ROS transport load
            if self.resize_width > 0 and self.resize_height > 0:
                frame = cv2.resize(frame, (self.resize_width, self.resize_height))

            with self.frame_lock:
                self.latest_frame = frame
                self.latest_frame_ts = time.time()

    def frame_callback(self):
        frame = None

        with self.frame_lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()

        if frame is None:
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "rtsp_camera"
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")

    def destroy_node(self):
        self.running = False

        try:
            if hasattr(self, "reader_thread") and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass

        self._release_stream()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = RTSPImagePublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f"Failed to start RTSPImagePublisher: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()