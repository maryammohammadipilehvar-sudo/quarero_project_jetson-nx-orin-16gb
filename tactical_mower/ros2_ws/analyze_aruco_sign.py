#!/usr/bin/env python3
"""One-off: from the dock bag, determine ArUco steering sign + detection reliability.

Run inside ros2_jetson (has ROS jazzy + rosbag2 + the bind-mounted bag).
"""
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1] if len(sys.argv) > 1 else \
    '/app/ros2_ws/ros_bags/aruco_dock_20260605_151456/bag'

reader = SequentialReader()
reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

aruco = []   # (t, x, z)
cmd = []     # (t, msg)
cmd_fields = None
while reader.has_next():
    name, data, t = reader.read_next()
    ts = t * 1e-9
    if name == '/docking/aruco_pose':
        m = deserialize_message(data, get_message(types[name]))
        aruco.append((ts, m.pose.position.x, m.pose.position.z))
    elif name == '/cmd_drive':
        m = deserialize_message(data, get_message(types[name]))
        if cmd_fields is None:
            cmd_fields = [f for f in m.get_fields_and_field_types().keys()]
        cmd.append((ts, m))

print(f"/cmd_drive fields: {cmd_fields}")
print(f"aruco_pose samples: {len(aruco)}  cmd_drive samples: {len(cmd)}")
if not aruco:
    print("NO aruco samples in bag — marker never published during recording.")
    sys.exit(0)

t0 = aruco[0][0]
span = aruco[-1][0] - t0
print(f"aruco time span: {span:.1f}s")

# Detection reliability: gaps between consecutive aruco samples
gaps = [aruco[i+1][0] - aruco[i][0] for i in range(len(aruco)-1)]
if gaps:
    big = [g for g in gaps if g > 0.5]
    print(f"inter-sample gaps: max={max(gaps):.2f}s  mean={sum(gaps)/len(gaps):.3f}s  "
          f"gaps>0.5s: {len(big)} (these are the 'marker lost/stale' dropouts)")

def val(m, names):
    for n in names:
        if hasattr(m, n):
            return getattr(m, n)
    return None

# Steering-sign evidence: when marker is off-centre (|x| big), is the robot's turn
# command toward the marker (centring → |x| should shrink) or away (|x| grows → wrong sign)?
print("\n t-t0   marker_x(cm)  range(m)   nearest cmd_drive(turn)   |x| trend")
ci = 0
prev_x = None
for (ts, x, z) in aruco:
    while ci+1 < len(cmd) and cmd[ci+1][0] <= ts:
        ci += 1
    cm = cmd[ci][1] if cmd else None
    left = val(cm, ['left_vel', 'left', 'left_velocity']) if cm else None
    right = val(cm, ['right_vel', 'right', 'right_velocity']) if cm else None
    # only show rows where the robot is actually commanding motion (the approach),
    # skip the long parked tail
    cmd_age = (ts - cmd[ci][0]) if cmd else 99
    moving = (left is not None and (abs(left) > 0.01 or abs(right) > 0.01) and cmd_age < 0.5)
    if not moving:
        prev_x = x
        continue
    turn = ""
    if left is not None and right is not None:
        d = left - right
        turn = f"L={left:+.2f} R={right:+.2f} dLR={d:+.2f}"
    trend = ""
    if prev_x is not None:
        trend = "x→edge(grow)" if abs(x) > abs(prev_x)+0.005 else ("x→centre(shrink)" if abs(x) < abs(prev_x)-0.005 else "~flat")
    prev_x = x
    print(f"{ts-t0:6.2f}  {x*100:+8.1f}    {z:5.2f}    {turn:24s}  {trend}")
