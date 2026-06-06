#!/usr/bin/env python3
"""Quick trajectory dump: robot start, mid, end positions and WP positions in
local ENU (using first GPS fix as origin). Also reports heading vs expected."""

import math
import sys
import yaml
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WGS84_A = 6378137.0
WGS84_E2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563)


def lla_to_ecef(lat, lon, alt):
    la, lo = math.radians(lat), math.radians(lon)
    s = math.sin(la)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    x = (n + alt) * math.cos(la) * math.cos(lo)
    y = (n + alt) * math.cos(la) * math.sin(lo)
    z = (n * (1.0 - WGS84_E2) + alt) * s
    return x, y, z


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    x, y, z = lla_to_ecef(lat, lon, alt)
    x0, y0, z0 = lla_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    sla, cla = math.sin(math.radians(lat0)), math.cos(math.radians(lat0))
    slo, clo = math.sin(math.radians(lon0)), math.cos(math.radians(lon0))
    e = -slo * dx + clo * dy
    n = -sla * clo * dx - sla * slo * dy + cla * dz
    return e, n


def main():
    run = Path(sys.argv[1])
    bag = str(run / "bag")
    so = rosbag2_py.StorageOptions(uri=bag, storage_id="mcap")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader()
    r.open(so, co)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}

    llh, ypr, state = [], [], []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic == "/fixposition/odometry_llh":
            m = deserialize_message(data, get_message(types[topic]))
            llh.append((t_ns * 1e-9, m.latitude, m.longitude, m.altitude))
        elif topic == "/fixposition/ypr":
            m = deserialize_message(data, get_message(types[topic]))
            ypr.append((t_ns * 1e-9, m.vector.x))  # yaw rad in ENU
        elif topic == "/tactical/robot/state":
            m = deserialize_message(data, get_message(types[topic]))
            state.append((t_ns * 1e-9, m.data))

    if not llh:
        print("no GPS in bag"); return

    # ENU origin = first GPS fix
    lat0, lon0, alt0 = llh[0][1], llh[0][2], llh[0][3]
    # WPs from route YAML. Pick whichever .yaml is alongside the bag.
    routes_dir = run / "routen_routes"
    candidates = [p for p in routes_dir.glob("*.yaml") if not p.name.endswith(".bak")]
    if not candidates:
        # fallback to current operator-bound routes
        candidates = [p for p in Path("/routen/routes").glob("*.yaml")
                      if not p.name.endswith(".bak")]
    wp_yaml = candidates[0]
    print(f"Using route file: {wp_yaml}")
    wps_lla = yaml.safe_load(wp_yaml.read_text())["waypoints"]
    wps_enu = [lla_to_enu(w["latitude"], w["longitude"], w.get("altitude", 0.0),
                          lat0, lon0, alt0) for w in wps_lla]

    print(f"ENU origin (= first GPS fix at t=0): lat={lat0:.7f} lon={lon0:.7f}")
    print()
    print("Waypoints in this ENU frame:")
    for i, (e, n) in enumerate(wps_enu):
        bearing = math.degrees(math.atan2(n, e))  # bearing from origin
        dist = math.hypot(e, n)
        print(f"  WP{i}:  east={e:+7.2f} m   north={n:+7.2f} m   "
              f"(bearing from origin: {bearing:+7.1f}°, dist {dist:.2f} m)")
    print()

    # Robot trajectory milestones (in same ENU frame)
    t0 = llh[0][0]
    samples = [
        ("start",    llh[0]),
        ("25 % in", llh[len(llh) // 4]),
        ("50 % in", llh[len(llh) // 2]),
        ("75 % in", llh[3 * len(llh) // 4]),
        ("end",     llh[-1]),
    ]
    print("Robot trajectory (subset of GPS fixes, ENU same frame as WPs):")
    print(f"  {'label':>10}  t(s)   east     north   yaw(ypr.x °)  bearing→WP0  bearing→WP1")
    for label, (t, lat, lon, alt) in samples:
        e, n = lla_to_enu(lat, lon, alt, lat0, lon0, alt0)
        # find yaw closest in time
        yaw = None
        if ypr:
            yaw = min(ypr, key=lambda yy: abs(yy[0] - t))[1]
        yaw_deg = math.degrees(yaw) if yaw is not None else float("nan")
        # bearing to WP0 / WP1 from robot
        def bear(wp):
            return math.degrees(math.atan2(wp[1] - n, wp[0] - e))
        b0 = bear(wps_enu[0])
        b1 = bear(wps_enu[1]) if len(wps_enu) > 1 else float("nan")
        print(f"  {label:>10}  {t - t0:+6.1f}  {e:+6.2f}   {n:+6.2f}     "
              f"{yaw_deg:+7.1f}      {b0:+7.1f}      {b1:+7.1f}")
    print()

    # End-to-start displacement direction (where did the robot actually go?)
    e_start, n_start = lla_to_enu(llh[0][1], llh[0][2], llh[0][3], lat0, lon0, alt0)
    e_end, n_end = lla_to_enu(llh[-1][1], llh[-1][2], llh[-1][3], lat0, lon0, alt0)
    de, dn = e_end - e_start, n_end - n_start
    actual_bearing = math.degrees(math.atan2(dn, de))
    total_dist = math.hypot(de, dn)
    print(f"Net displacement during bag: {total_dist:.2f} m at bearing "
          f"{actual_bearing:+7.1f}° (ENU)")
    print(f"  (compare to WP0 bearing from origin: "
          f"{math.degrees(math.atan2(wps_enu[0][1], wps_enu[0][0])):+7.1f}°)")


if __name__ == "__main__":
    main()
