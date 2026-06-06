#!/usr/bin/env python3
"""Analyze the manual_heading_tes2t bag to find a 180° heading offset.

Compares:
  - /fixposition/ypr.vector.x          (radians, ENU: 0=East, +pi/2=North)
  - /fixposition/odometry_enu yaw       (radians, from pose.orientation quaternion)
  - /cmd_drive left_vel / right_vel    (rad/s wheel velocity commanded)
  - /joy_drive_raw left_stick_forward  (raw stick value)
  - /fixposition/odometry_llh lat/lon  (ground-truth position from GNSS)

For every "drive forward" phase (joy.left_stick_forward > 100 sustained),
the script computes the bearing-of-motion from the lat/lon trace, and
compares it to the YPR yaw and the odometry_enu yaw at that moment.
A 180° gap between bearing-of-motion and YPR yaw is the smoking gun.

Run inside ros2_jetson container:
  docker exec ros2_jetson bash -lc \
    "source /opt/ros/jazzy/setup.bash && \
     source /app/ros2_ws/install_ros2/setup.bash && \
     python3 /app/ros2_ws/analyze_heading_test.py /tmp/manual_heading_tes2t"
"""
import math
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def latlon_to_local(lat, lon, lat0, lon0):
    """Approximate ENU offset from (lat0, lon0) reference, meters."""
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    n = R * dlat
    e = R * math.cos(math.radians(lat0)) * dlon
    return e, n  # east, north (meters)


def main(bag_path):
    bag = Path(bag_path)
    ypr_t, ypr_yaw = [], []
    odom_t, odom_yaw = [], []
    odom_vx, odom_vy = [], []  # twist.linear.x, twist.linear.y (ENU velocity, m/s)
    llh_t, llh_lat, llh_lon = [], [], []
    cmd_t, cmd_l, cmd_r = [], [], []
    joy_t, joy_lsf, joy_lsx, joy_rsr = [], [], [], []
    state_t, state_data = [], []

    with AnyReader([bag]) as reader:
        for conn, ts, raw in reader.messages():
            t = ts * 1e-9
            if conn.topic == '/fixposition/ypr':
                msg = reader.deserialize(raw, conn.msgtype)
                ypr_t.append(t); ypr_yaw.append(float(msg.vector.x))
            elif conn.topic == '/fixposition/odometry_enu':
                msg = reader.deserialize(raw, conn.msgtype)
                q = msg.pose.pose.orientation
                odom_t.append(t)
                odom_yaw.append(quat_to_yaw(q.x, q.y, q.z, q.w))
                odom_vx.append(float(msg.twist.twist.linear.x))
                odom_vy.append(float(msg.twist.twist.linear.y))
            elif conn.topic == '/fixposition/odometry_llh':
                msg = reader.deserialize(raw, conn.msgtype)
                llh_t.append(t); llh_lat.append(msg.latitude); llh_lon.append(msg.longitude)
            elif conn.topic == '/cmd_drive':
                msg = reader.deserialize(raw, conn.msgtype)
                cmd_t.append(t); cmd_l.append(float(msg.left_vel)); cmd_r.append(float(msg.right_vel))
            elif conn.topic == '/joy_drive_raw':
                msg = reader.deserialize(raw, conn.msgtype)
                joy_t.append(t)
                joy_lsf.append(float(getattr(msg, 'left_stick_forward', 0)))
                joy_lsx.append(float(getattr(msg, 'left_stick_right', 0)) if hasattr(msg, 'left_stick_right') else 0.0)
                joy_rsr.append(float(getattr(msg, 'right_stick_right', 0)))
            elif conn.topic == '/robot/state':
                msg = reader.deserialize(raw, conn.msgtype)
                state_t.append(t); state_data.append(msg.data[:200])

    if not ypr_t:
        print("No /fixposition/ypr messages — aborting.")
        return 1

    t0 = min(ypr_t[0], odom_t[0] if odom_t else ypr_t[0], cmd_t[0] if cmd_t else ypr_t[0])
    print(f"\n=== bag: {bag.name} ===")
    print(f"duration   : {max(ypr_t) - t0:.2f} s")
    print(f"ypr msgs   : {len(ypr_t)}  | odom_enu msgs: {len(odom_t)}  | llh msgs: {len(llh_t)}")
    print(f"cmd_drive  : {len(cmd_t)}  | joy_raw     : {len(joy_t)}")

    # 1. Compare YPR yaw vs odometry_enu yaw at every odom sample.
    def nearest(times, vals, t):
        if not times:
            return None
        i = min(range(len(times)), key=lambda k: abs(times[k] - t))
        return vals[i]

    print("\n--- Heading source comparison (samples every 5 s) ---")
    print(f"{'t[s]':>6} {'ypr [rad]':>11} {'ypr[deg]':>10} {'odom_enu[deg]':>14} {'diff[deg]':>11}")
    step = max(1, len(ypr_t) // 12)
    for i in range(0, len(ypr_t), step):
        t = ypr_t[i]
        y_ypr = ypr_yaw[i]
        y_odom = nearest(odom_t, odom_yaw, t)
        if y_odom is None:
            continue
        diff = math.degrees(math.atan2(math.sin(y_ypr - y_odom), math.cos(y_ypr - y_odom)))
        print(f"{t - t0:6.2f} {y_ypr:11.4f} {math.degrees(y_ypr):10.2f} {math.degrees(y_odom):14.2f} {diff:11.2f}")

    # 2. Detect phases from joystick.
    print("\n--- Joystick phases (forward stick > 100 or rotation stick > 100) ---")
    phases = []
    cur = None
    for t, lsf, rsr in zip(joy_t, joy_lsf, joy_rsr):
        if abs(lsf) > 100 or abs(rsr) > 100:
            kind = 'FWD' if lsf > 100 else ('REV' if lsf < -100 else ('ROT_R' if rsr > 100 else 'ROT_L'))
            if cur is None or cur['kind'] != kind:
                if cur is not None:
                    cur['t_end'] = t
                    phases.append(cur)
                cur = {'kind': kind, 't_start': t, 't_end': t, 'lsf_peak': lsf, 'rsr_peak': rsr}
            else:
                cur['t_end'] = t
                if abs(lsf) > abs(cur['lsf_peak']):
                    cur['lsf_peak'] = lsf
                if abs(rsr) > abs(cur['rsr_peak']):
                    cur['rsr_peak'] = rsr
        else:
            if cur is not None and (t - cur['t_end']) > 0.5:
                phases.append(cur)
                cur = None
    if cur is not None:
        phases.append(cur)

    # Filter to phases at least 1.5 s long.
    phases = [p for p in phases if p['t_end'] - p['t_start'] >= 1.5]
    print(f"{'kind':>6} {'t_start':>8} {'t_end':>8} {'dur':>6} {'lsf_peak':>9} {'rsr_peak':>9}")
    for p in phases:
        print(f"{p['kind']:>6} {p['t_start'] - t0:8.2f} {p['t_end'] - t0:8.2f} "
              f"{p['t_end'] - p['t_start']:6.2f} {p['lsf_peak']:9.0f} {p['rsr_peak']:9.0f}")

    # 3. For each phase, find: avg cmd_drive sign, avg YPR yaw, bearing-of-motion from llh.
    print("\n--- Per-phase: cmd_drive sign vs YPR yaw vs bearing-of-motion ---")
    print(f"{'kind':>6} {'cmd_l_avg':>10} {'cmd_r_avg':>10} {'ypr[deg]':>10} {'bearing[deg]':>13} {'ypr-brg':>9}")
    for p in phases:
        in_phase_cmd_l = [v for t, v in zip(cmd_t, cmd_l) if p['t_start'] <= t <= p['t_end']]
        in_phase_cmd_r = [v for t, v in zip(cmd_t, cmd_r) if p['t_start'] <= t <= p['t_end']]
        in_phase_ypr = [v for t, v in zip(ypr_t, ypr_yaw) if p['t_start'] <= t <= p['t_end']]
        in_phase_lat = [(t, la, lo) for t, la, lo in zip(llh_t, llh_lat, llh_lon)
                        if p['t_start'] <= t <= p['t_end']]

        cmd_l_avg = sum(in_phase_cmd_l) / len(in_phase_cmd_l) if in_phase_cmd_l else float('nan')
        cmd_r_avg = sum(in_phase_cmd_r) / len(in_phase_cmd_r) if in_phase_cmd_r else float('nan')
        ypr_deg = math.degrees(sum(in_phase_ypr) / len(in_phase_ypr)) if in_phase_ypr else float('nan')

        bearing_deg = float('nan')
        if len(in_phase_lat) >= 2:
            _, lat_s, lon_s = in_phase_lat[0]
            _, lat_e, lon_e = in_phase_lat[-1]
            e, n = latlon_to_local(lat_e, lon_e, lat_s, lon_s)
            if math.hypot(e, n) > 0.10:  # require >10 cm displacement
                # ENU yaw convention: 0=East, +90=North → atan2(n, e)
                bearing_rad = math.atan2(n, e)
                bearing_deg = math.degrees(bearing_rad)
        diff_str = ''
        if not math.isnan(bearing_deg):
            d = math.degrees(math.atan2(
                math.sin(math.radians(ypr_deg - bearing_deg)),
                math.cos(math.radians(ypr_deg - bearing_deg))
            ))
            diff_str = f"{d:+.1f}"
        print(f"{p['kind']:>6} {cmd_l_avg:10.2f} {cmd_r_avg:10.2f} {ypr_deg:10.2f} "
              f"{bearing_deg if not math.isnan(bearing_deg) else 'n/a':>13} {diff_str:>9}")

    # 4. Static yaw snapshots at the very start and very end (robot likely stationary).
    print("\n--- Yaw snapshot at bag start and end (robot stationary) ---")
    print(f"start  ypr={math.degrees(ypr_yaw[0]):7.2f} deg  "
          f"odom_enu={math.degrees(odom_yaw[0]) if odom_yaw else 'n/a':>7} deg")
    print(f"end    ypr={math.degrees(ypr_yaw[-1]):7.2f} deg  "
          f"odom_enu={math.degrees(odom_yaw[-1]) if odom_yaw else 'n/a':>7} deg")

    # 5. Robust analysis: instantaneous velocity vector vs instantaneous yaw,
    #    in sliding 1.0 s windows. Survives constant-turning recordings.
    print("\n--- Sliding-window instantaneous bearing vs YPR yaw ---")
    print("(window=1.0s, only printed when ground speed > 0.20 m/s AND cmd_l and cmd_r same sign)")

    # Build lat/lon arrays indexed by time, plus a function for "displacement in window".
    win = 1.0
    fwd_diffs, rev_diffs = [], []
    fwd_speeds, rev_speeds = [], []
    sample_count = 0
    print(f"{'t[s]':>6} {'mode':>5} {'speed[m/s]':>11} {'ypr[deg]':>9} {'bearing[deg]':>13} {'diff[deg]':>10}")
    for i, t in enumerate(llh_t):
        t_start = t
        t_end = t + win
        # Find first sample at or after t_end
        j = i
        while j < len(llh_t) and llh_t[j] < t_end:
            j += 1
        if j >= len(llh_t):
            break
        dt = llh_t[j] - llh_t[i]
        if dt < 0.5:
            continue
        e, n = latlon_to_local(llh_lat[j], llh_lon[j], llh_lat[i], llh_lon[i])
        dist = math.hypot(e, n)
        speed = dist / dt
        if speed < 0.20:
            continue

        # average cmd_drive in the window
        cmd_l_win = [v for ct, v in zip(cmd_t, cmd_l) if t_start <= ct <= t_end]
        cmd_r_win = [v for ct, v in zip(cmd_t, cmd_r) if t_start <= ct <= t_end]
        if not cmd_l_win or not cmd_r_win:
            continue
        cl = sum(cmd_l_win) / len(cmd_l_win)
        cr = sum(cmd_r_win) / len(cmd_r_win)
        # require both wheels in the same direction (not pure spin)
        if cl * cr <= 0:
            continue
        mode = 'FWD' if (cl + cr) > 0 else 'REV'

        # instantaneous YPR yaw at midpoint
        t_mid = 0.5 * (t_start + t_end)
        y = nearest(ypr_t, ypr_yaw, t_mid)
        bearing = math.atan2(n, e)
        diff = math.degrees(math.atan2(math.sin(y - bearing), math.cos(y - bearing)))

        if mode == 'FWD':
            fwd_diffs.append(diff); fwd_speeds.append(speed)
        else:
            rev_diffs.append(diff); rev_speeds.append(speed)

        sample_count += 1
        if sample_count <= 30:  # cap printing
            print(f"{t_start - t0:6.2f} {mode:>5} {speed:11.3f} {math.degrees(y):9.2f} "
                  f"{math.degrees(bearing):13.2f} {diff:+10.2f}")

    def circ_mean_deg(xs):
        s = sum(math.sin(math.radians(d)) for d in xs)
        c = sum(math.cos(math.radians(d)) for d in xs)
        return math.degrees(math.atan2(s, c))

    print(f"\n  total windows: {sample_count}  (printed first 30)")
    if fwd_diffs:
        # Bucket the FWD windows by |diff|
        nose_first = [d for d in fwd_diffs if abs(d) < 45]
        rear_first = [d for d in fwd_diffs if abs(d) > 135]
        sideways = [d for d in fwd_diffs if 45 <= abs(d) <= 135]
        print(f"\n  Cmd-drive BOTH POSITIVE windows (n={len(fwd_diffs)}) bucketed by |ypr - bearing|:")
        print(f"    nose-first  (|diff| <  45 deg)     : {len(nose_first):4d}  ({100*len(nose_first)/len(fwd_diffs):5.1f} %)  -> motion matches yaw -> driving forward as expected")
        print(f"    sideways    (45 deg <= |diff| <= 135 deg)  : {len(sideways):4d}  ({100*len(sideways)/len(fwd_diffs):5.1f} %)  -> rotating or sliding")
        print(f"    REAR-FIRST  (|diff| > 135 deg)     : {len(rear_first):4d}  ({100*len(rear_first)/len(fwd_diffs):5.1f} %)  -> commanded forward but moving backward -> SIGN ERROR")
        if rear_first:
            print(f"    rear-first circular mean diff   : {circ_mean_deg(rear_first):+.2f} deg  (expected ~+-180)")
        if nose_first:
            print(f"    nose-first circular mean diff   : {circ_mean_deg(nose_first):+.2f} deg  (expected ~0)")
        print(f"\n  FWD windows overall circular mean of (ypr - bearing) = {circ_mean_deg(fwd_diffs):+.2f} deg")
    if rev_diffs:
        print(f"  REV windows (both cmd negative) (n={len(rev_diffs)}): circular mean = {circ_mean_deg(rev_diffs):+.2f} deg")

    # === DEFINITIVE TEST: instantaneous velocity vs yaw ===
    # Use /fixposition/odometry_enu.twist.linear.{x,y} — driver's own velocity estimate.
    # vx, vy are in ENU frame (m/s). Bearing of velocity vector vs yaw tells us nose/rear directly,
    # with no chord/rotation artifacts.
    print("\n=== DEFINITIVE: instantaneous velocity vs YPR yaw (no chord artifacts) ===")
    print("Frame: ENU. velocity_bearing = atan2(vy, vx). 0=East, +90=North.")
    inst_nose, inst_rear, inst_side = 0, 0, 0
    inst_diffs_fast = []  # only windows with speed > 0.4 m/s
    inst_nose_fast, inst_rear_fast, inst_side_fast = 0, 0, 0
    cmd_l_during_rear, cmd_r_during_rear = [], []
    rear_first_samples = []
    for i in range(len(odom_t)):
        vx, vy = odom_vx[i], odom_vy[i]
        spd = math.hypot(vx, vy)
        if spd < 0.15:
            continue
        # need a cmd_drive at this timestamp
        cl = nearest(cmd_t, cmd_l, odom_t[i])
        cr = nearest(cmd_t, cmd_r, odom_t[i])
        if cl is None or cr is None:
            continue
        if cl <= 0 or cr <= 0:
            continue  # only "forward commanded" samples
        bearing = math.atan2(vy, vx)
        y = odom_yaw[i]  # use odom_enu's own yaw (== ypr yaw — proven earlier)
        diff = math.degrees(math.atan2(math.sin(y - bearing), math.cos(y - bearing)))
        ad = abs(diff)
        if ad < 45:
            inst_nose += 1
            if spd > 0.4:
                inst_nose_fast += 1
                inst_diffs_fast.append(diff)
        elif ad > 135:
            inst_rear += 1
            cmd_l_during_rear.append(cl); cmd_r_during_rear.append(cr)
            rear_first_samples.append((odom_t[i] - t0, spd, cl, cr, math.degrees(y), math.degrees(bearing), diff))
            if spd > 0.4:
                inst_rear_fast += 1
                inst_diffs_fast.append(diff)
        else:
            inst_side += 1
            if spd > 0.4:
                inst_side_fast += 1

    total = inst_nose + inst_rear + inst_side
    total_fast = inst_nose_fast + inst_rear_fast + inst_side_fast
    print(f"\n  Forward-commanded samples (speed>0.15 m/s, cl>0, cr>0):")
    print(f"    nose-first : {inst_nose:5d}  ({100*inst_nose/max(1,total):5.1f}%)")
    print(f"    sideways   : {inst_side:5d}  ({100*inst_side/max(1,total):5.1f}%)")
    print(f"    REAR-FIRST : {inst_rear:5d}  ({100*inst_rear/max(1,total):5.1f}%)")
    print(f"\n  Same buckets restricted to speed > 0.4 m/s (genuine forward driving, not rotation):")
    print(f"    nose-first : {inst_nose_fast:5d}  ({100*inst_nose_fast/max(1,total_fast):5.1f}%)")
    print(f"    sideways   : {inst_side_fast:5d}  ({100*inst_side_fast/max(1,total_fast):5.1f}%)")
    print(f"    REAR-FIRST : {inst_rear_fast:5d}  ({100*inst_rear_fast/max(1,total_fast):5.1f}%)")

    if rear_first_samples:
        print(f"\n  REAR-FIRST instantaneous samples — first 20 (t, speed, cmd_l, cmd_r, ypr_deg, bearing_deg, diff_deg):")
        for s in rear_first_samples[:20]:
            print(f"   t={s[0]:7.2f}  spd={s[1]:5.2f}  cl={s[2]:+5.2f}  cr={s[3]:+5.2f}  yaw={s[4]:+7.2f}  brg={s[5]:+7.2f}  diff={s[6]:+7.2f}")
        # Circular mean
        rear_diffs_only = [s[6] for s in rear_first_samples]
        print(f"  rear-first circular mean diff: {circ_mean_deg(rear_diffs_only):+.2f} deg")

    # Time-resolved view of rear-first windows so we can see WHEN it happens.
    print("\n--- Time-resolved REAR-FIRST windows (|diff|>135 with cmd_l, cmd_r both positive) ---")
    # Rebuild because we lost timing info above.
    print(f"{'t[s]':>6} {'speed[m/s]':>10} {'cmd_l':>7} {'cmd_r':>7} {'ypr[deg]':>9} {'bearing[deg]':>13} {'diff[deg]':>10}")
    rf_count = 0
    for i, t in enumerate(llh_t):
        t_start = t
        t_end = t + win
        j = i
        while j < len(llh_t) and llh_t[j] < t_end:
            j += 1
        if j >= len(llh_t):
            break
        dt = llh_t[j] - llh_t[i]
        if dt < 0.5:
            continue
        e, n = latlon_to_local(llh_lat[j], llh_lon[j], llh_lat[i], llh_lon[i])
        dist = math.hypot(e, n)
        speed = dist / dt
        if speed < 0.20:
            continue
        cmd_l_win = [v for ct, v in zip(cmd_t, cmd_l) if t_start <= ct <= t_end]
        cmd_r_win = [v for ct, v in zip(cmd_t, cmd_r) if t_start <= ct <= t_end]
        if not cmd_l_win or not cmd_r_win:
            continue
        cl = sum(cmd_l_win) / len(cmd_l_win)
        cr = sum(cmd_r_win) / len(cmd_r_win)
        if cl <= 0 or cr <= 0:
            continue  # only positive-cmd windows
        t_mid = 0.5 * (t_start + t_end)
        y = nearest(ypr_t, ypr_yaw, t_mid)
        bearing = math.atan2(n, e)
        diff = math.degrees(math.atan2(math.sin(y - bearing), math.cos(y - bearing)))
        if abs(diff) <= 135:
            continue
        rf_count += 1
        if rf_count <= 40:
            print(f"{t_start - t0:6.2f} {speed:10.3f} {cl:+7.2f} {cr:+7.2f} {math.degrees(y):9.2f} {math.degrees(bearing):13.2f} {diff:+10.2f}")
    print(f"  total rear-first windows: {rf_count}")

    print("\nInterpretation:")
    print("  - YPR yaw is in ENU: 0=East, +90=North, +-180=West, -90=South.")
    print("  - bearing[deg] is the direction the GNSS antenna physically moved in the world (same ENU convention).")
    print("  - FWD circular mean near   0 deg: antenna aligned with physical nose -> NO 180 deg offset.")
    print("  - FWD circular mean near +-180 deg: antenna mounted opposite to nose -> 180 deg offset confirmed.")
    print("  - FWD circular mean near +-90 deg: antenna mounted sideways.")
    return 0


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/manual_heading_tes2t'
    raise SystemExit(main(path))
