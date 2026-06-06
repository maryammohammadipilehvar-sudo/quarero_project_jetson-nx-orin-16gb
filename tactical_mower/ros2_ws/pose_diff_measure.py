"""Measure current pose vs the *currently-saved* charge point — tolerance reference tool.

Reads the charge point live from settings.yaml (NOT hardcoded — it changes),
samples /fixposition/odometry_llh (lat/lon/alt) and /fixposition/ypr
(vector.x=yaw, .y=pitch, .z=roll in RAD) for a few seconds, averages, and
reports the full 6-DOF difference: position decomposed into longitudinal /
lateral / vertical (relative to the docked heading) plus yaw / pitch / roll.

Run it whenever the robot is physically parked on the dock correctly; the
residuals (offset + per-axis spread) are what you use as docking tolerances.
"""

import math
import os
import statistics as st

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

SETTINGS_PATH = os.environ.get("SETTINGS_PATH", "/routen/settings/settings.yaml")
COLLECT_SECONDS = 8.0
EARTH_M_PER_DEG = 111320.0


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def load_charge_point(path):
    with open(path) as f:
        s = yaml.safe_load(f)
    cp = s["charge_point"]
    return cp["latitude"], cp["longitude"], cp.get("yaw")


class Sampler(Node):
    def __init__(self):
        super().__init__("pose_diff_measure")
        self.lat, self.lon, self.alt = [], [], []
        self.yaw, self.pitch, self.roll = [], [], []
        self.create_subscription(NavSatFix, "/fixposition/odometry_llh", self._llh, 50)
        self.create_subscription(Vector3Stamped, "/fixposition/ypr", self._ypr, 50)

    def _llh(self, m):
        self.lat.append(m.latitude)
        self.lon.append(m.longitude)
        self.alt.append(m.altitude)

    def _ypr(self, m):
        # Fixposition packs (yaw, pitch, roll) into (x, y, z), radians
        self.yaw.append(math.degrees(m.vector.x))
        self.pitch.append(math.degrees(m.vector.y))
        self.roll.append(math.degrees(m.vector.z))


def stat(label, vals, unit):
    if not vals:
        print(f"  {label:14s}: NO DATA")
        return None, None, None
    m = st.mean(vals)
    sd = st.pstdev(vals) if len(vals) > 1 else 0.0
    p2p = max(vals) - min(vals)
    print(f"  {label:14s}: mean={m:.6f}{unit}  std={sd:.4f}  p2p={p2p:.4f}  n={len(vals)}")
    return m, sd, p2p


def main():
    charge_lat, charge_lon, charge_yaw = load_charge_point(SETTINGS_PATH)
    print(f"Saved charge point (live from {SETTINGS_PATH}):")
    print(f"  lat={charge_lat}  lon={charge_lon}  yaw={charge_yaw}")

    rclpy.init()
    node = Sampler()
    end = node.get_clock().now().nanoseconds / 1e9 + COLLECT_SECONDS
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("\n=== RAW SAMPLES (mean / std / peak-to-peak) ===")
    lat, _, _ = stat("latitude", node.lat, "deg")
    lon, _, _ = stat("longitude", node.lon, "deg")
    alt, _, _ = stat("altitude", node.alt, "m")
    yaw, yaw_sd, yaw_p2p = stat("yaw", node.yaw, "deg")
    pitch, pitch_sd, pitch_p2p = stat("pitch", node.pitch, "deg")
    roll, roll_sd, roll_p2p = stat("roll", node.roll, "deg")

    if None in (lat, lon, yaw):
        print("\n!! Missing data — is the robot powered and RTK up?")
        node.destroy_node(); rclpy.shutdown(); return

    east = (lon - charge_lon) * math.cos(math.radians(charge_lat)) * EARTH_M_PER_DEG
    north = (lat - charge_lat) * EARTH_M_PER_DEG
    horiz = math.hypot(east, north)

    if charge_yaw is None:
        print("\n(no saved yaw on charge point — skipping longitudinal/lateral split & yaw err)")
        node.destroy_node(); rclpy.shutdown(); return

    # ENU/REP-103: x=East, y=North, yaw=0 -> facing East, CCW positive.
    cy = math.radians(charge_yaw)
    fwd = (math.cos(cy), math.sin(cy))      # longitudinal (robot's facing dir at dock)
    left = (-math.sin(cy), math.cos(cy))    # lateral (+ = robot left of dock line)
    longitudinal = east * fwd[0] + north * fwd[1]
    lateral = east * left[0] + north * left[1]
    yaw_err = wrap180(yaw - charge_yaw)

    print("\n=== POSE DIFFERENCE: current  -  saved charge point ===")
    print(f"  Horizontal distance : {horiz*100:8.2f} cm   (E {east*100:+.2f}, N {north*100:+.2f})")
    print(f"  Longitudinal (fwd)  : {longitudinal*100:+8.2f} cm   (+ = past charge pt along heading)")
    print(f"  Lateral (sideways)  : {lateral*100:+8.2f} cm   (+ = left of dock line)")
    print(f"  Vertical (alt)      :   current alt {alt:.3f} m   (no saved alt reference)")
    print(f"  Yaw  (heading)      : {yaw_err:+8.2f} deg  (current {yaw:.2f} vs saved {charge_yaw:.2f})")
    print(f"  Pitch               : {pitch:+8.2f} deg  (no saved ref; fwd/back ground tilt)")
    print(f"  Roll                : {roll:+8.2f} deg  (no saved ref; left/right ground tilt)")

    print("\n=== SUGGESTED TOLERANCE REFERENCE (|offset| + ~2x noise spread) ===")
    print(f"  longitudinal : >= {abs(longitudinal)*100 + 2*0:.1f} cm  (offset {abs(longitudinal)*100:.1f} cm + GPS noise)")
    print(f"  lateral      : >= {abs(lateral)*100:.1f} cm")
    print(f"  yaw          : >= {abs(yaw_err) + 2*(yaw_sd or 0):.2f} deg  (offset {abs(yaw_err):.2f} + 2*std {yaw_sd or 0:.2f})")
    print(f"  pitch        :  ~ {abs(pitch) + 2*(pitch_sd or 0):.2f} deg")
    print(f"  roll         :  ~ {abs(roll) + 2*(roll_sd or 0):.2f} deg")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
