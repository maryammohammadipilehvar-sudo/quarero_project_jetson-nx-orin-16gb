#!/usr/bin/env python3
"""Live heading-vs-motion checker — PATCH-AWARE version.

Same as live_heading_check.py but compares motion direction to (raw_yaw + π)
because the wp_follower now adds 180° to the FP yaw internally (temporary
hardware workaround in _ypr_callback). Use this while the +π patch is active.
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
        super().__init__("live_heading_check_patched")
        self.lat = self.lon = None
        self.yaw = None
        self.anchor_lat = self.anchor_lon = None
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

    def tick(self):
        if self.lat is None or self.yaw is None or self.anchor_lat is None:
            return
        de = (self.lon - self.anchor_lon) * math.cos(math.radians(self.anchor_lat)) * 111320.0
        dn = (self.lat - self.anchor_lat) * 111320.0
        d = math.hypot(de, dn)
        now = time.time()
        if now - self.last_print < 0.8:
            return
        self.last_print = now
        elapsed = now - self.t_start
        # patched heading = raw + π, normalized to (-π, π]
        patched_yaw = self.yaw + math.pi
        if patched_yaw > math.pi:
            patched_yaw -= 2.0 * math.pi
        if d < 0.30:
            print(f"t+{elapsed:5.1f}s   waiting for motion … d={d*100:5.1f} cm   "
                  f"raw_yaw={math.degrees(self.yaw):+6.1f}°   "
                  f"patched_yaw={math.degrees(patched_yaw):+6.1f}°",
                  flush=True)
            return
        bearing = math.degrees(math.atan2(dn, de))
        py_deg = math.degrees(patched_yaw)
        diff = (bearing - py_deg + 540) % 360 - 180
        if abs(diff) < 30:
            verdict = "OK   ✓ patched-heading matches motion — PATCH IS WORKING"
        elif abs(abs(diff) - 180) < 30:
            verdict = "FLIPPED ✗ patched-heading still wrong (now opposite of motion)"
        else:
            verdict = "DRIFT   ? lateral motion, ambiguous"
        print(f"t+{elapsed:5.1f}s   d={d:5.2f} m   motion_bearing={bearing:+6.1f}°   "
              f"patched_yaw={py_deg:+6.1f}°   Δ={diff:+6.1f}°   →  {verdict}",
              flush=True)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    n = Monitor()
    deadline = time.time() + timeout
    print(f"[patch-check] watching for {timeout:.0f}s. Ctrl-C to stop.", flush=True)
    print(f"[patch-check] Verdict labels are wrt the +π-patched yaw, "
          f"so 'OK' means the patch is working.", flush=True)
    try:
        while time.time() < deadline:
            rclpy.spin_once(n, timeout_sec=0.2)
            n.tick()
    except KeyboardInterrupt:
        pass
    print("[patch-check] done.", flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
