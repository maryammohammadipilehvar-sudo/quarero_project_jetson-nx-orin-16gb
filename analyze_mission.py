#!/usr/bin/env python3
"""Analyze autonomous mission bag for waypoint following accuracy and efficiency."""

import math
import json
from pathlib import Path
from mcap_ros2.reader import read_ros2_messages

BAG_PATH = Path.home() / "gits/tactical_mower/ros2_ws/ros_bags/mission_20260524_114831/bag"

# Target waypoints from test1.yaml
WAYPOINTS = [
    (48.65420285970295, 9.22481180253302),
    (48.654215368147604, 9.224893173941327),
    (48.654257019874, 9.224890491004214),
    (48.6542260026342, 9.22470268540518),
]

CHARGE_POS = (48.65420816564051, 9.224837786322238)

def haversine_m(lat1, lon1, lat2, lon2):
    """Distance between two GPS points in meters."""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

def main():
    print("=" * 70)
    print("AUTONOMOUS MISSION ANALYSIS — Waypoint Following Accuracy")
    print("=" * 70)

    # Collect data from bag
    positions = []  # (timestamp_ns, lat, lon)
    cmd_drives = []  # (timestamp_ns, left_vel, right_vel)
    states = []  # (timestamp_ns, state_json)
    speeds = []  # (timestamp_ns, speed_kmh)

    print("\nReading bag file...")
    mcap_file = None
    for f in BAG_PATH.glob("*.mcap"):
        mcap_file = f
        break

    if not mcap_file:
        print(f"ERROR: No .mcap file found in {BAG_PATH}")
        return

    print(f"  File: {mcap_file} ({mcap_file.stat().st_size / 1e6:.1f} MB)")

    ANALYSIS_TOPICS = [
        "/fixposition/odometry_llh",
        "/cmd_drive",
        "/tactical/robot/state",
        "/tactical/robot/speed_kmh",
        "/tactical/robot/route_completed",
        "/control/autonomous_operation",
    ]

    for msg in read_ros2_messages(str(mcap_file), topics=ANALYSIS_TOPICS):
        topic = msg.channel.topic
        ts = msg.log_time.timestamp()  # convert datetime to float seconds

        if topic == "/fixposition/odometry_llh":
            lat = msg.ros_msg.latitude
            lon = msg.ros_msg.longitude
            if lat != 0 and lon != 0:
                positions.append((ts, lat, lon))

        elif topic == "/cmd_drive":
            left = msg.ros_msg.left_vel
            right = msg.ros_msg.right_vel
            cmd_drives.append((ts, left, right))

        elif topic == "/tactical/robot/state":
            try:
                data = json.loads(msg.ros_msg.data)
                states.append((ts, data))
            except:
                pass

        elif topic == "/tactical/robot/speed_kmh":
            speeds.append((ts, msg.ros_msg.data))

    print(f"  Positions: {len(positions)}")
    print(f"  Drive cmds: {len(cmd_drives)}")
    print(f"  State msgs: {len(states)}")
    print(f"  Speed msgs: {len(speeds)}")

    if not positions:
        print("ERROR: No position data in bag")
        return

    # Time range
    t_start = positions[0][0]
    t_end = positions[-1][0]
    duration_s = (t_end - t_start) / 1
    print(f"\n  Duration: {duration_s:.1f}s ({duration_s/60.0:.1f} min)")

    # ===== NAVIGATING SEGMENTS =====
    print("\n" + "=" * 70)
    print("NAVIGATION SEGMENTS")
    print("=" * 70)

    nav_segments = []  # (start_ts, end_ts, start_wp, end_wp)
    current_nav_start = None
    current_wp = None

    for ts, state_data in states:
        state_name = state_data.get("state", "")
        ar = state_data.get("active_route") or {}
        wp_idx = ar.get("current_waypoint_index")

        if state_name == "NAVIGATING" and current_nav_start is None:
            current_nav_start = ts
            current_wp = wp_idx
        elif state_name != "NAVIGATING" and current_nav_start is not None:
            nav_segments.append((current_nav_start, ts, current_wp, wp_idx))
            current_nav_start = None

    if current_nav_start:
        nav_segments.append((current_nav_start, t_end, current_wp, current_wp))

    print(f"\n  Total NAVIGATING segments: {len(nav_segments)}")
    total_nav_time = sum((e - s) / 1 for s, e, _, _ in nav_segments)
    print(f"  Total NAVIGATING time: {total_nav_time:.1f}s ({total_nav_time/60:.1f} min)")
    print(f"  Total session time: {duration_s:.1f}s")
    print(f"  Navigation efficiency: {total_nav_time/duration_s*100:.1f}% of session spent navigating")

    # ===== WAYPOINT CLOSEST APPROACH =====
    print("\n" + "=" * 70)
    print("WAYPOINT ACCURACY — Closest approach to each waypoint")
    print("=" * 70)

    for i, (wp_lat, wp_lon) in enumerate(WAYPOINTS):
        min_dist = float('inf')
        min_ts = None
        for ts, lat, lon in positions:
            dist = haversine_m(lat, lon, wp_lat, wp_lon)
            if dist < min_dist:
                min_dist = dist
                min_ts = ts

        t_offset = (min_ts - t_start) / 1 if min_ts else 0
        print(f"\n  WP{i} ({wp_lat:.7f}, {wp_lon:.7f}):")
        print(f"    Closest approach: {min_dist:.3f} m (at t+{t_offset:.1f}s)")
        if min_dist > 0.5:
            print(f"    ⚠ EXCEEDS waypoint_tolerance (0.5m)")
        else:
            print(f"    ✓ Within tolerance (0.5m)")

    # ===== DRIVE COMMAND ANALYSIS =====
    print("\n" + "=" * 70)
    print("DRIVE COMMAND ANALYSIS")
    print("=" * 70)

    if cmd_drives:
        # Categorize commands
        rotating = 0  # opposite signs, similar magnitude
        driving = 0   # same sign
        stopped = 0   # near zero

        for ts, left, right in cmd_drives:
            if abs(left) < 0.1 and abs(right) < 0.1:
                stopped += 1
            elif left * right < 0:  # opposite signs = rotation
                rotating += 1
            else:
                driving += 1

        total = len(cmd_drives)
        print(f"\n  Total commands: {total}")
        print(f"  Driving forward/backward: {driving} ({driving/total*100:.1f}%)")
        print(f"  Rotating in place: {rotating} ({rotating/total*100:.1f}%)")
        print(f"  Stopped (near zero): {stopped} ({stopped/total*100:.1f}%)")

        # Max velocities
        max_left = max(abs(l) for _, l, _ in cmd_drives)
        max_right = max(abs(r) for _, _, r in cmd_drives)
        avg_speed = sum(abs(l) + abs(r) for _, l, r in cmd_drives) / (2 * total)
        print(f"\n  Max wheel speed: {max(max_left, max_right):.2f} rad/s")
        print(f"  Avg wheel speed: {avg_speed:.2f} rad/s")

    # ===== SPEED ANALYSIS =====
    print("\n" + "=" * 70)
    print("SPEED PROFILE")
    print("=" * 70)

    if speeds:
        nav_speeds = []
        for ts, spd in speeds:
            # Check if this timestamp is during a NAVIGATING segment
            for seg_start, seg_end, _, _ in nav_segments:
                if seg_start <= ts <= seg_end:
                    nav_speeds.append(spd)
                    break

        if nav_speeds:
            avg_nav_speed = sum(nav_speeds) / len(nav_speeds)
            max_nav_speed = max(nav_speeds)
            print(f"\n  During NAVIGATING:")
            print(f"    Avg speed: {avg_nav_speed:.3f} km/h ({avg_nav_speed/3.6*1000:.1f} mm/s)")
            print(f"    Max speed: {max_nav_speed:.3f} km/h")
            print(f"    Speed factor setting: 0.874 m/s = {0.874*3.6:.2f} km/h max")
            print(f"    Utilization: {avg_nav_speed/(0.874*3.6)*100:.1f}% of max speed")

    # ===== PATH EFFICIENCY =====
    print("\n" + "=" * 70)
    print("PATH EFFICIENCY")
    print("=" * 70)

    # Total distance traveled
    total_distance = 0
    for i in range(1, len(positions)):
        d = haversine_m(positions[i-1][1], positions[i-1][2],
                       positions[i][1], positions[i][2])
        if d < 5:  # filter GPS jumps > 5m
            total_distance += d

    # Ideal distance (sum of waypoint-to-waypoint distances)
    ideal_distance = 0
    for i in range(len(WAYPOINTS) - 1):
        ideal_distance += haversine_m(WAYPOINTS[i][0], WAYPOINTS[i][1],
                                      WAYPOINTS[i+1][0], WAYPOINTS[i+1][1])

    # Distance from start to first WP
    start_to_wp0 = haversine_m(positions[0][1], positions[0][2],
                                WAYPOINTS[0][0], WAYPOINTS[0][1])

    print(f"\n  Total distance traveled: {total_distance:.2f} m")
    print(f"  Ideal one-way route distance: {ideal_distance:.2f} m")
    print(f"  Start position to WP0: {start_to_wp0:.2f} m")
    print(f"  Route + approach: {ideal_distance + start_to_wp0:.2f} m")

    if ideal_distance > 0:
        # For ping-pong: ideal would be 2 × route distance
        ping_pong_ideal = 2 * ideal_distance + start_to_wp0
        print(f"  Ideal ping-pong distance: {ping_pong_ideal:.2f} m")
        print(f"  Path efficiency (vs ping-pong ideal): {ping_pong_ideal/total_distance*100:.1f}%")

    # ===== STATE TRANSITIONS =====
    print("\n" + "=" * 70)
    print("STATE TRANSITION LOG")
    print("=" * 70)

    prev_state = None
    transitions = []
    for ts, state_data in states:
        state_name = state_data.get("state", "")
        if state_name != prev_state:
            t_offset = (ts - t_start) / 1
            ar = state_data.get("active_route") or {}
            wp = ar.get("current_waypoint_index", "-")
            transitions.append((t_offset, prev_state, state_name, wp))
            prev_state = state_name

    print(f"\n  {'Time':>8} | {'From':<20} → {'To':<20} | WP")
    print(f"  {'-'*8}-+-{'-'*20}---{'-'*20}-+----")
    for t, frm, to, wp in transitions[:30]:
        frm_str = frm or "(start)"
        print(f"  {t:>7.1f}s | {frm_str:<20} → {to:<20} | {wp}")
    if len(transitions) > 30:
        print(f"  ... ({len(transitions) - 30} more transitions)")

    print(f"\n  Total transitions: {len(transitions)}")

    # ===== SUMMARY =====
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    wp_min_dists = []
    for i, (wp_lat, wp_lon) in enumerate(WAYPOINTS):
        min_dist = min(haversine_m(lat, lon, wp_lat, wp_lon) for _, lat, lon in positions)
        wp_min_dists.append(min_dist)

    avg_accuracy = sum(wp_min_dists) / len(wp_min_dists)
    max_miss = max(wp_min_dists)

    print(f"\n  Waypoint accuracy (avg closest approach): {avg_accuracy:.3f} m")
    print(f"  Worst waypoint miss: {max_miss:.3f} m")
    print(f"  All within tolerance (0.5m): {'YES' if max_miss <= 0.5 else 'NO'}")
    print(f"  Navigation time efficiency: {total_nav_time/duration_s*100:.1f}%")
    print(f"  Rotation vs driving ratio: {rotating}/{driving} = {rotating/max(driving,1):.1f}x")
    print(f"  Total distance: {total_distance:.1f} m in {duration_s:.0f}s")

    # Key issues found
    print(f"\n  ISSUES DETECTED:")
    issues = []
    if max_miss > 0.5:
        issues.append(f"  - Waypoint miss > tolerance: {max_miss:.3f}m (max)")
    if total and rotating / max(total, 1) > 0.5:
        issues.append(f"  - Excessive rotation: {rotating/total*100:.0f}% of commands are rotations")
    if len(transitions) > 10:
        issues.append(f"  - Excessive state transitions: {len(transitions)} (schedule interference?)")
    if not issues:
        issues.append("  - None critical")
    print("\n".join(issues))

if __name__ == "__main__":
    main()
