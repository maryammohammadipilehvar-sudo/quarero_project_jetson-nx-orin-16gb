"""
Person Detection Bridge Node
=============================
Polls Jetson 2 HTTP /status endpoint every 0.5s
and publishes /person_detected (Bool) ROS2 topic.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import urllib.request
import json


class PersonDetectionBridge(Node):

    def __init__(self):
        super().__init__("person_detection_bridge")
        self.declare_parameter("jetson2_url", "http://192.168.10.140:8080/status")
        self.declare_parameter("poll_rate", 2.0)
        self.declare_parameter("timeout", 2.0)

        self._url = self.get_parameter("jetson2_url").value
        poll_rate = float(self.get_parameter("poll_rate").value)
        self._timeout = float(self.get_parameter("timeout").value)
        self._last_status = False
        self._last_data = {}

        self._pub = self.create_publisher(Bool, "/person_detected", 10)
        self.create_timer(1.0 / poll_rate, self._poll)

        self.get_logger().info(
            f"PersonDetectionBridge started - polling {self._url} at {poll_rate}Hz"
        )

    def _poll(self):
        try:
            with urllib.request.urlopen(self._url, timeout=self._timeout) as resp:
                self._last_data = json.loads(resp.read().decode())
            detected = bool(self._last_data.get("person_detected", False))
        except Exception as e:
            self.get_logger().warning(f"Poll failed: {e}")
            detected = False

        msg = Bool()
        msg.data = detected
        self._pub.publish(msg)

        if detected != self._last_status:
            if detected:
                count = self._last_data.get("person_count", 0)
                self.get_logger().warning(f"PERSON DETECTED: {count} person(s)!")
            else:
                self.get_logger().info("Person detection cleared")
            self._last_status = detected


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
