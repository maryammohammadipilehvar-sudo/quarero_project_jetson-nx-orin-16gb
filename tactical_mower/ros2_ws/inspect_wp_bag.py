#!/usr/bin/env python3
"""Dump state-machine transitions + waypoint-index timeline + cmd_drive activity from a bag."""

import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(bag_dir):
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    targets = {
        "/tactical/robot/state",
        "/robot/state",
        "/cmd_drive",
        "/control/autonomous_operation",
        "/tactical/control/charging/undock",
        "/tactical/logging/warn",
        "/tactical/logging/info",
        "/tactical/robot/route_completed",
        "/fixposition/odometry_llh",
    }
    out = {t: [] for t in targets if t in type_map}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in out:
            continue
        try:
            msg = deserialize_message(data, get_message(type_map[topic]))
        except Exception:
            continue
        out[topic].append((t_ns * 1e-9, msg))
    return out


def main():
    bag = Path(sys.argv[1]) / "bag"
    s = read_bag(str(bag))
    t0 = s["/fixposition/odometry_llh"][0][0]

    def rel(t):
        return t - t0

    print("=" * 70)
    print("STATE-MACHINE TRANSITIONS  /tactical/robot/state")
    print("=" * 70)
    last_state = None
    for t, m in s.get("/tactical/robot/state", []):
        try:
            payload = json.loads(m.data) if m.data.startswith("{") else {"state": m.data}
            state = payload.get("state", "?")
        except Exception:
            state = m.data
        if state != last_state:
            print(f"  t+{rel(t):6.1f}s   {state}")
            last_state = state

    print()
    print("=" * 70)
    print("/robot/state — waypoint index + route name + autonomous flags")
    print("=" * 70)
    last_key = None
    for t, m in s.get("/robot/state", []):
        try:
            d = json.loads(m.data)
        except Exception:
            continue
        ar = d.get("active_route") or {}
        rn = ar.get("name", "")
        wi = ar.get("current_waypoint_index", None)
        wn = len(ar.get("waypoints", []))
        ae = d.get("autonomous_enabled")
        am = d.get("autonomous_mode")
        cs = d.get("charging_state")
        key = (rn, wi, wn, ae, am, cs)
        if key != last_key:
            print(f"  t+{rel(t):6.1f}s   route='{rn}'  wp_idx={wi}/{wn}  "
                  f"auton_en={ae}  auton_mode={am}  charging={cs}")
            last_key = key

    print()
    print("=" * 70)
    print("/control/autonomous_operation toggles")
    print("=" * 70)
    for t, m in s.get("/control/autonomous_operation", []):
        print(f"  t+{rel(t):6.1f}s   data={m.data}")

    print()
    print("=" * 70)
    print("/tactical/control/charging/undock events")
    print("=" * 70)
    for t, m in s.get("/tactical/control/charging/undock", []):
        print(f"  t+{rel(t):6.1f}s   data={m.data}")

    print()
    print("=" * 70)
    print("/tactical/robot/route_completed events")
    print("=" * 70)
    for t, m in s.get("/tactical/robot/route_completed", []):
        print(f"  t+{rel(t):6.1f}s   data={m.data}")

    print()
    print("=" * 70)
    print("/tactical/logging/info + warn")
    print("=" * 70)
    for t, m in s.get("/tactical/logging/info", []):
        print(f"  t+{rel(t):6.1f}s   INFO  {m.data}")
    for t, m in s.get("/tactical/logging/warn", []):
        print(f"  t+{rel(t):6.1f}s   WARN  {m.data}")

    print()
    print("=" * 70)
    print("/cmd_drive activity histogram (10-s buckets, by avg |v|, |omega|)")
    print("=" * 70)
    R = 0.535 / 2
    B = 0.637
    cmd = s.get("/cmd_drive", [])
    if cmd:
        t_start = cmd[0][0]
        t_end = cmd[-1][0]
        bucket = 10.0
        for bi in range(int((t_end - t_start) / bucket) + 1):
            lo = t_start + bi * bucket
            hi = lo + bucket
            sub = [m for tt, m in cmd if lo <= tt < hi]
            if not sub:
                continue
            vs = [R * (m.left_vel + m.right_vel) / 2 for m in sub]
            ws = [R * (m.right_vel - m.left_vel) / B for m in sub]
            v_abs = sum(abs(v) for v in vs) / len(vs)
            w_abs = sum(abs(w) for w in ws) / len(ws)
            print(f"  t+{rel(lo):6.1f}…{rel(hi):6.1f}s  n={len(sub):4d}  "
                  f"<|v|>={v_abs:.2f} m/s   <|ω|>={w_abs:.2f} rad/s")

    print()
    print("=" * 70)
    print("/cmd_drive timing gaps (only gaps > 0.25 s)")
    print("=" * 70)
    gaps = []
    for i in range(1, len(cmd)):
        dt = cmd[i][0] - cmd[i - 1][0]
        if dt > 0.25:
            gaps.append((cmd[i - 1][0], dt))
    print(f"  total /cmd_drive messages: {len(cmd)} over "
          f"{cmd[-1][0] - cmd[0][0]:.1f}s "
          f"(avg rate {len(cmd) / (cmd[-1][0] - cmd[0][0]):.2f} Hz)")
    print(f"  gaps > 0.25 s: {len(gaps)}")
    for t, dt in gaps[:15]:
        print(f"    t+{rel(t):6.1f}s   gap={dt*1000:.0f} ms")


if __name__ == "__main__":
    main()
