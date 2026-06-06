#!/usr/bin/env python3
"""
Teach-and-repeat setpoint capture for ArUco precision docking.

Run this ONCE, with the robot manually parked in the *perfect* docked position
(charging contacts mated), while the aruco_dock_detector is publishing
/docking/aruco_pose. It averages the marker pose over a few seconds and writes
the result as the docking setpoint into /routen/settings/aruco_dock.yaml.

The docking controller (tactical_wp_follower) then servos the final approach to
reproduce exactly this marker pose. No camera-mount measurements needed.

Run inside the ros2_jetson container (it has rclpy + the /routen mount):

  docker exec -it ros2_jetson bash -lc \
    "source /opt/ros/jazzy/setup.bash && \
     source /app/ros2_ws/install_ros2/setup.bash && \
     python3 /routen/../docking_aruco/teach_dock_setpoint.py --samples 40"

(Adjust the script path to wherever this repo is mounted in the container.)

After teaching, the file has setpoint_valid: true but enabled: false. Bench-verify
the steering direction first (see DESIGN_ARUCO_PRECISION_DOCKING.md §6), then
set enabled: true to arm it. Re-run with --enable to set both at once.
"""

import argparse
import math
import os

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


DEFAULT_OUT = '/routen/settings/aruco_dock.yaml'

# Full schema with safe defaults; teach only overwrites the setpoint_* fields
# (and enabled if --enable). Keys must match ArucoDockConfig attribute names.
DEFAULTS = {
    'enabled': False,
    'engage_distance': 1.5,
    'stale_timeout': 1.0,
    # Mode: teach writes a setpoint, but leave use_taught_setpoint False so behaviour
    # only changes when the operator explicitly opts into teach-and-repeat.
    'use_taught_setpoint': False,
    'center_marker': True,
    'setpoint_lateral': 0.0,
    'setpoint_range': 0.0,
    'setpoint_yaw': 0.0,
    'setpoint_valid': False,
    # Markerless target standoff + hard anti-wall floor (floor MUST be below standoff).
    'stop_range_m': 0.85,
    'min_range_m': 0.70,
    'range_floor_margin': 0.05,
    'gain_lateral': 120.0,
    'gain_yaw': 40.0,
    'invert_steering': False,
    'creep_speed_ratio': 0.35,
    'max_steering': 60.0,
    'dock_lateral_tolerance': 0.04,
    'dock_range_tolerance': 0.03,
    'dock_confirm_readings': 3,
    'max_range_jump': 0.3,
    'reacquire_grace': 2.0,
    'reengage_cooldown': 6.0,
    'servo_timeout': 25.0,
}


def _bearing(q) -> float:
    normal_x = 2.0 * (q.x * q.z + q.w * q.y)
    normal_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.atan2(normal_x, normal_z)


class SetpointTeacher(Node):
    def __init__(self, samples):
        super().__init__('aruco_teach_setpoint')
        self._target = samples
        self._lat = []
        self._rng = []
        self._yaw = []
        self.done = False
        self.create_subscription(PoseStamped, '/docking/aruco_pose', self._cb, 10)
        self.get_logger().info(
            f"Collecting {samples} marker samples — hold the robot in the docked "
            f"position. Waiting for /docking/aruco_pose...")

    def _cb(self, msg: PoseStamped):
        if self.done:
            return
        p = msg.pose
        self._lat.append(p.position.x)
        self._rng.append(p.position.z)
        self._yaw.append(_bearing(p.orientation))
        n = len(self._lat)
        if n % 10 == 0:
            self.get_logger().info(f"  {n}/{self._target} samples")
        if n >= self._target:
            self.done = True

    def result(self):
        n = len(self._lat)
        return {
            'setpoint_lateral': sum(self._lat) / n,
            'setpoint_range': sum(self._rng) / n,
            'setpoint_yaw': sum(self._yaw) / n,
            'setpoint_valid': True,
        }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default=DEFAULT_OUT, help='output config path')
    ap.add_argument('--samples', type=int, default=40, help='marker samples to average')
    ap.add_argument('--enable', action='store_true',
                    help='also set enabled: true (only after bench-verifying steering)')
    args = ap.parse_args()

    rclpy.init()
    node = SetpointTeacher(args.samples)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        teacher_done = node.done
        result = node.result() if teacher_done else None
        node.destroy_node()
        rclpy.shutdown()

    if not teacher_done or result is None:
        print("No marker samples received — is aruco_dock_detector running and "
              "the marker in view? Nothing written.")
        return 1

    # Merge: existing file (if any) → defaults for missing keys → taught setpoint.
    cfg = dict(DEFAULTS)
    if os.path.exists(args.out):
        try:
            with open(args.out, 'r') as f:
                existing = yaml.safe_load(f) or {}
            if isinstance(existing, dict):
                cfg.update(existing)
        except Exception as e:
            print(f"Warning: could not read existing {args.out}: {e}")
    cfg.update(result)
    if args.enable:
        cfg['enabled'] = True

    with open(args.out, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=True)

    print(f"\nTaught setpoint written to {args.out}:")
    print(f"  lateral = {cfg['setpoint_lateral']*100:+.1f} cm")
    print(f"  range   = {cfg['setpoint_range']*100:.1f} cm")
    print(f"  yaw     = {math.degrees(cfg['setpoint_yaw']):+.1f} deg")
    print(f"  enabled = {cfg['enabled']}")
    if not cfg['enabled']:
        print("\nNext: bench-verify steering direction, then set enabled: true "
              "(or re-run with --enable).")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
