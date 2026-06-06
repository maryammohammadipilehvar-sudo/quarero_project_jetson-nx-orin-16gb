#!/usr/bin/env python3
"""Live heading-vs-motion checker.

Subscribes to /fixposition/odometry_llh and /fixposition/ypr, anchors to the
first fix, and prints a verdict line every second once the robot has moved
>30 cm: does the bearing of the displacement match the heading the antennas
report?

Exits on Ctrl-C or after the timeout (default 60 s). Run inside ros2_jetson.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped


class Monitor(Node):
    def __init__(self):
        super().__init__("live_heading_check")
        self.lat = self.lon = None
        self.yaw = None
        self.anchor_lat = self.anchor_lon = None
        self.anchor_yaw = None
        self.t_start = None
        self.last_print = 0.0
        self.create_subscription(NavSatFix, "/fixposition/odometry_llh", self._llh, 10)
        self.create_subscription(Vector3Stamped, "/fixposition/ypr", self._ypr, 10)

    def _llh(self, m):
        self.lat = m.latitude
        self.lon = m.longitude
        if self.anchor_lat is None:
            self.anchor_lat = m.latitude
            self.anchor_lon = m.longitude
            self.t_start = time.time()
            self.get_logger().info(
                f"Anchored: lat={m.latitude:.7f} lon={m.longitude:.7f}"
            )

    def _ypr(self, m):
        self.yaw = m.vector.x
        if self.anchor_yaw is None:
            self.anchor_yaw = m.vector.x

    def tick(self):
        if self.lat is None or self.yaw is None or self.anchor_lat is None:
            return
        # local ENU offset, flat-earth approximation (fine over a few meters)
        de = (self.lon - self.anchor_lon) * math.cos(math.radians(self.anchor_lat)) * 111320.0
        dn = (self.lat - self.anchor_lat) * 111320.0
        d = math.hypot(de, dn)
        now = time.time()
        if now - self.last_print < 0.8:
            return
        self.last_print = now
        elapsed = now - self.t_start
        if d < 0.30:
            print(f"t+{elapsed:5.1f}s   waiting for motion … displacement {d*100:5.1f} cm, "
                  f"yaw_ypr={math.degrees(self.yaw):+6.1f}°", flush=True)
            return
        bearing = math.degrees(math.atan2(dn, de))
        yaw_deg = math.degrees(self.yaw)
        diff = (bearing - yaw_deg + 540) % 360 - 180  # signed shortest delta in (-180, 180]
        if abs(diff) < 30:
            verdict = "OK   ✓ heading matches motion"
        elif abs(abs(diff) - 180) < 30:
            verdict = "FLIPPED ✗ STILL 180° INVERTED — release L1!"
        else:
            verdict = "DRIFT   ? lateral/tangential motion, ambiguous"
        print(f"t+{elapsed:5.1f}s   d={d:5.2f} m   bearing={bearing:+6.1f}°   "
              f"yaw_ypr={yaw_deg:+6.1f}°   Δ={diff:+6.1f}°   →  {verdict}",
              flush=True)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    n = Monitor()
    deadline = time.time() + timeout
    print(f"[live-check] watching for {timeout:.0f}s. Ctrl-C to stop.", flush=True)
    try:
        while time.time() < deadline:
            rclpy.spin_once(n, timeout_sec=0.2)
            n.tick()
    except KeyboardInterrupt:
        pass
    print("[live-check] done.", flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
