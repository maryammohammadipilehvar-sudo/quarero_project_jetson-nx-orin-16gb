"""LiDAR-Camera Fusion Node.

Suppresses LiDAR obstacles within RealSense camera FOV not confirmed by camera.
Outside FOV or beyond max_range: LiDAR always trusted.
If camera data stale/missing: passthrough mode (pure LiDAR).
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import UInt8MultiArray


class FusionNode(Node):

    def __init__(self):
        super().__init__('lidar_camera_fusion')

        self.declare_parameter('camera.hfov_deg', 86.0)
        self.declare_parameter('camera.num_sectors', 5)
        self.declare_parameter('camera.max_range_m', 2.0)
        self.declare_parameter('camera.stale_timeout_s', 2.0)
        self.declare_parameter('topics.lidar_grid', '/obstacles/lidar')
        self.declare_parameter('topics.camera_sectors', '/obstacle_sectors')
        self.declare_parameter('topics.fused_output', '/obstacles/fused')

        hfov_deg = float(self.get_parameter('camera.hfov_deg').value)
        self._num_sectors = int(self.get_parameter('camera.num_sectors').value)
        self._max_range = float(self.get_parameter('camera.max_range_m').value)
        self._stale_timeout = float(self.get_parameter('camera.stale_timeout_s').value)
        lidar_topic = str(self.get_parameter('topics.lidar_grid').value)
        camera_topic = str(self.get_parameter('topics.camera_sectors').value)
        fused_topic = str(self.get_parameter('topics.fused_output').value)

        self._hfov_half_rad = math.radians(hfov_deg / 2.0)
        self._camera_sectors = None
        self._camera_stamp = 0.0

        self.create_subscription(OccupancyGrid, lidar_topic, self._lidar_cb, 10)
        self.create_subscription(UInt8MultiArray, camera_topic, self._camera_cb, 10)
        self._pub = self.create_publisher(OccupancyGrid, fused_topic, 10)

        self.get_logger().info(
            f'Fusion node started | FOV=±{hfov_deg/2:.0f}° | range={self._max_range}m | '
            f'{lidar_topic} + {camera_topic} → {fused_topic}'
        )

    def _camera_cb(self, msg: UInt8MultiArray):
        try:
            self._camera_sectors = list(msg.data)
            self._camera_stamp = time.time()
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')

    def _lidar_cb(self, msg: OccupancyGrid):
        try:
            self._lidar_cb_impl(msg)
        except Exception as e:
            self.get_logger().error(f'Fusion lidar callback error: {e}')

    def _lidar_cb_impl(self, msg: OccupancyGrid):
        camera_fresh = (
            self._camera_sectors is not None and
            (time.time() - self._camera_stamp) < self._stale_timeout
        )

        if not camera_fresh:
            self._pub.publish(msg)
            return

        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        fused_data = list(msg.data)
        near_sectors = self._camera_sectors[:self._num_sectors]

        for idx in range(width * height):
            if fused_data[idx] < 50:
                continue

            cx = idx % width
            cy = idx // width
            world_x = origin_x + (cx + 0.5) * resolution
            world_y = origin_y + (cy + 0.5) * resolution
            distance = math.hypot(world_x, world_y)

            if distance < 0.1 or distance > self._max_range:
                continue

            angle = math.atan2(world_y, world_x)
            if abs(angle) > self._hfov_half_rad:
                continue

            sector_idx = int((angle + self._hfov_half_rad) / (2.0 * self._hfov_half_rad) * self._num_sectors)
            sector_idx = max(0, min(self._num_sectors - 1, sector_idx))

            if near_sectors[sector_idx] == 0:
                fused_data[idx] = 0

        fused_msg = OccupancyGrid()
        fused_msg.header = msg.header
        fused_msg.info = msg.info
        fused_msg.data = fused_data
        self._pub.publish(fused_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
