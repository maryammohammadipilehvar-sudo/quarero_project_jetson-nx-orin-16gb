#!/usr/bin/env python3
"""
Live dock-alignment check — use when re-deploying the robot to a new site.

After mounting the two markers and driving the robot to the *perfect* docked
position, run this. It reads the fused two-marker pose and prints, live:

  cross   = sideways offset from the dock centre-line (cm)   -> want ~0
  heading = robot heading vs the marker normal (deg)         -> want ~0
  perp    = camera->marker plane distance (m)                -> this IS stop_range_m

When BOTH cross and heading are within tolerance and held steady for a couple
seconds, it prints ">>> SAVE NOW". Only then save the charge point in the web app
(while the robot is stationary). That guarantees the auto-computed home point puts
the GPS approach line straight down the marker normal — which is the whole game.

WHY this tool exists: the GPS dock line direction = the robot's heading at the
instant you save. A diff-drive turns to move sideways, so "nudging" rotates it and
corrupts the line (this caused an off-axis approach + wall-crash on 2026-06-06).
With live feedback you can re-square after every nudge and only save when truly
aligned.

Run inside ros2_jetson:

  docker exec -it ros2_jetson bash -lc \
    "source /opt/ros/jazzy/setup.bash && \
     source /app/ros2_ws/install_ros2/setup.bash && \
     python3 /routen/../docking_aruco/dock_align_check.py"

(adjust the path to wherever this repo is mounted in the container)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

CROSS_TOL_M = 0.03      # 3 cm
HEAD_TOL_DEG = 3.0      # 3 deg
STALE_S = 1.0           # pose older than this -> markers not both visible
STABLE_S = 2.0          # must hold in-tolerance this long before "SAVE NOW"


def dock_errors(pose):
    """Robot pose in the dock frame from the marker pose (camera optical frame).
    Identical math to tactical_wp_follower._marker_dock_errors."""
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    r00 = 1 - 2 * (y * y + z * z); r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w);     r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w);     r22 = 1 - 2 * (x * x + y * y)
    px, py, pz = pose.position.x, pose.position.y, pose.position.z
    cross = -(r00 * px + r10 * py + r20 * pz)
    perp = abs(-(r02 * px + r12 * py + r22 * pz))
    h = math.atan2(r20, r22) - math.pi
    while h > math.pi:
        h -= 2 * math.pi
    while h < -math.pi:
        h += 2 * math.pi
    return cross, perp, math.degrees(h)


class DockAlign(Node):
    def __init__(self):
        super().__init__('dock_align_check')
        self._last = None
        self._in_tol_since = None
        self.create_subscription(PoseStamped, '/docking/aruco_pose', self._cb, 10)
        self.create_timer(0.2, self._tick)
        print("dock_align_check: place the robot at the perfect docked position, "
              "hold still until you see '>>> SAVE NOW', then save the charge point.\n")

    def _cb(self, msg):
        self._last = (msg.pose, self.get_clock().now().nanoseconds / 1e9)

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last is None or (now - self._last[1]) > STALE_S:
            self._in_tol_since = None
            print("\rwaiting for BOTH markers (fused pose) — only one or none in view"
                  "            ", end="", flush=True)
            return
        c, perp, h = dock_errors(self._last[0])
        ok = abs(c) < CROSS_TOL_M and abs(h) < HEAD_TOL_DEG
        if ok:
            if self._in_tol_since is None:
                self._in_tol_since = now
            stable = now - self._in_tol_since
        else:
            self._in_tol_since = None
            stable = 0.0
        if ok and stable >= STABLE_S:
            tag = ">>> SAVE NOW (stop_range_m = %.2f)" % perp
        elif ok:
            tag = "hold steady..."
        else:
            tag = "ADJUST:"
            if abs(c) >= CROSS_TOL_M:
                tag += " move %s %.0fcm" % ("LEFT" if c > 0 else "RIGHT", abs(c) * 100)
            if abs(h) >= HEAD_TOL_DEG:
                tag += " rotate %s %.0fdeg" % ("RIGHT" if h > 0 else "LEFT", abs(h))
        print("\rcross=%+5.1fcm  heading=%+5.1fdeg  perp=%.2fm   %s        "
              % (c * 100, h, perp, tag), end="", flush=True)


def main():
    rclpy.init()
    node = DockAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
