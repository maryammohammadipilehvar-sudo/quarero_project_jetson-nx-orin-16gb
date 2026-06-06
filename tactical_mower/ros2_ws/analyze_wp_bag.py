#!/usr/bin/env python3
"""Offline KPI analysis of a tactical_wp_follower bag.

Run inside ros2_jetson:
    docker exec ros2_jetson bash -lc \
        "source /opt/ros/jazzy/setup.bash && \
         source /app/ros2_ws/install_ros2/setup.bash && \
         python3 /app/ros2_ws/analyze_wp_bag.py /app/ros2_ws/ros_bags/<run>"

Writes <run>/analysis_report.md alongside the bag.
"""

import json
import math
import os
import statistics
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt_m) * s
    return x, y, z


def ecef_to_enu(x, y, z, lat0, lon0, alt0):
    x0, y0, z0 = lla_to_ecef(lat0, lon0, alt0)
    dx = x - x0
    dy = y - y0
    dz = z - z0
    s_lat = math.sin(math.radians(lat0))
    c_lat = math.cos(math.radians(lat0))
    s_lon = math.sin(math.radians(lon0))
    c_lon = math.cos(math.radians(lon0))
    e = -s_lon * dx + c_lon * dy
    n = -s_lat * c_lon * dx - s_lat * s_lon * dy + c_lat * dz
    u = c_lat * c_lon * dx + c_lat * s_lon * dy + s_lat * dz
    return e, n, u


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    x, y, z = lla_to_ecef(lat, lon, alt)
    return ecef_to_enu(x, y, z, lat0, lon0, alt0)


def read_bag(bag_dir):
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    streams = {t: [] for t in type_map}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        cls = get_message(type_map[topic])
        try:
            msg = deserialize_message(data, cls)
        except Exception:
            continue
        streams[topic].append((t_ns * 1e-9, msg))
    return streams, type_map


def find_wp_yaml(bag_parent):
    """Read the active route YAML alongside the bag."""
    routes_dir = bag_parent / "routen_routes"
    if not routes_dir.exists():
        return None, []
    # If only one route file, use it. Otherwise, prefer one named in the README.
    files = sorted(p for p in routes_dir.glob("*.yaml") if not p.name.endswith(".bak"))
    if not files:
        return None, []
    chosen = files[0]
    if (routes_dir / "mm.yaml").exists():
        chosen = routes_dir / "mm.yaml"
    import yaml as _yaml
    with chosen.open() as f:
        data = _yaml.safe_load(f)
    wps = data.get("waypoints", [])
    return chosen.name, wps


def navigating_segments(state_stream):
    """Return list of (t_start, t_end) where /tactical/robot/state is NAVIGATING."""
    segs = []
    open_t = None
    for t, msg in state_stream:
        try:
            payload = json.loads(msg.data) if msg.data.startswith("{") else {"state": msg.data}
        except Exception:
            payload = {"state": str(msg.data)}
        state = payload.get("state", payload.get("data", ""))
        is_nav = "NAVIGATING" in state.upper()
        if is_nav and open_t is None:
            open_t = t
        elif not is_nav and open_t is not None:
            segs.append((open_t, t))
            open_t = None
    if open_t is not None and state_stream:
        segs.append((open_t, state_stream[-1][0]))
    return segs


def quantiles(xs):
    if not xs:
        return (None, None, None, None)
    xs = sorted(xs)
    n = len(xs)

    def q(p):
        i = max(0, min(n - 1, int(round(p * (n - 1)))))
        return xs[i]
    return q(0.5), q(0.95), max(xs), n


def perp_distance_to_segment(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    seg_len2 = abx * abx + aby * aby
    if seg_len2 < 1e-9:
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / seg_len2))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy), t


def project_onto_segment(px, py, ax, ay, bx, by):
    """Signed along-track distance from A toward B (negative = behind A,
    > seg_len = past B = overshoot)."""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    seg_len = math.hypot(abx, aby)
    if seg_len < 1e-9:
        return 0.0, 0.0
    s = (apx * abx + apy * aby) / seg_len
    return s, seg_len


def main():
    if len(sys.argv) != 2:
        print("usage: analyze_wp_bag.py <bag_run_dir>", file=sys.stderr)
        sys.exit(2)
    run_dir = Path(sys.argv[1]).resolve()
    bag_dir = run_dir / "bag"
    if not bag_dir.exists():
        print(f"no bag at {bag_dir}", file=sys.stderr)
        sys.exit(2)

    print(f"[analyze] reading bag at {bag_dir} ...")
    streams, type_map = read_bag(str(bag_dir))

    route_name, wp_list_lla = find_wp_yaml(run_dir)
    if not wp_list_lla:
        print("[analyze] no route yaml found alongside bag — KPI section will be empty",
              file=sys.stderr)

    # Pick ENU origin: first odometry_llh fix.
    llh = streams.get("/fixposition/odometry_llh", [])
    if not llh:
        print("[analyze] no /fixposition/odometry_llh in bag", file=sys.stderr)
        sys.exit(3)
    first = llh[0][1]
    lat0, lon0, alt0 = first.latitude, first.longitude, first.altitude

    # Robot trajectory in local ENU (m).
    traj = []  # (t, e, n)
    for t, msg in llh:
        e, n, _ = lla_to_enu(msg.latitude, msg.longitude, msg.altitude, lat0, lon0, alt0)
        traj.append((t, e, n))

    # Waypoints in same ENU.
    wps_enu = []
    for wp in wp_list_lla:
        e, n, _ = lla_to_enu(wp["latitude"], wp["longitude"], wp.get("altitude", 0.0),
                             lat0, lon0, alt0)
        wps_enu.append((e, n))

    # NAVIGATING segments.
    state_stream = streams.get("/tactical/robot/state", [])
    nav_segs = navigating_segments(state_stream)

    # Limit traj to NAVIGATING only for some KPIs.
    def in_nav(t):
        for s, e in nav_segs:
            if s <= t <= e:
                return True
        return False
    nav_traj = [(t, e, n) for (t, e, n) in traj if in_nav(t)]

    # Path length (NAVIGATING-only) and straight-line WP-to-WP sum.
    path_len = 0.0
    for i in range(1, len(nav_traj)):
        path_len += math.hypot(
            nav_traj[i][1] - nav_traj[i - 1][1],
            nav_traj[i][2] - nav_traj[i - 1][2],
        )
    straight_len = 0.0
    for i in range(1, len(wps_enu)):
        straight_len += math.hypot(
            wps_enu[i][0] - wps_enu[i - 1][0],
            wps_enu[i][1] - wps_enu[i - 1][1],
        )
    ratio = (path_len / straight_len) if straight_len > 1e-6 else float("inf")

    # Per-WP closest-approach + overshoot.
    # "Approach to WP_k" = traj samples within the leg (WP_{k-1} → WP_k) window,
    # which we approximate as: between the time we first got within 2 m of WP_k
    # and the time we first got within 2 m of WP_{k+1} (or end).
    wp_metrics = []
    for k, (wx, wy) in enumerate(wps_enu):
        min_d = float("inf")
        t_min = None
        for (t, e, n) in nav_traj:
            d = math.hypot(e - wx, n - wy)
            if d < min_d:
                min_d = d
                t_min = t

        overshoot = None
        if k + 1 < len(wps_enu) and t_min is not None:
            nx, ny = wps_enu[k + 1]
            # Along-track on segment WP_k -> WP_{k+1}: how far past WP_k did we
            # go *before* turning toward WP_{k+1}?  Look at all samples within
            # 2 s after t_min; take the max negative projection (i.e. past WP_k
            # in the direction *opposite* to WP_{k+1} is overshoot).
            # Simpler: among samples in [t_min, t_min + 3s], take max distance
            # *behind* the line perpendicular to (WP_{k-1}→WP_k) at WP_k.
            # Use the incoming leg if available; else use outgoing.
            if k - 1 >= 0:
                ax, ay = wps_enu[k - 1]
            else:
                # No incoming leg → fall back to direction from start of nav.
                if nav_traj:
                    ax, ay = nav_traj[0][1], nav_traj[0][2]
                else:
                    ax, ay = wx - 1.0, wy
            abx, aby = wx - ax, wy - ay
            seg_len = math.hypot(abx, aby)
            if seg_len > 1e-6:
                ux, uy = abx / seg_len, aby / seg_len
                max_over = 0.0
                for (t, e, n) in nav_traj:
                    if t < t_min or t > t_min + 3.0:
                        continue
                    # signed distance past WP_k along incoming direction
                    s = (e - wx) * ux + (n - wy) * uy
                    if s > max_over:
                        max_over = s
                overshoot = max_over

        wp_metrics.append({
            "idx": k,
            "min_dist_m": min_d if min_d < float("inf") else None,
            "overshoot_m": overshoot,
            "reached_within_05m": (min_d < 0.5) if min_d < float("inf") else False,
        })

    # Cross-track error per leg.
    leg_xte = []  # list of (k, p50, p95, max, n)
    for k in range(1, len(wps_enu)):
        ax, ay = wps_enu[k - 1]
        bx, by = wps_enu[k]
        xtes = []
        for (t, e, n) in nav_traj:
            # only samples whose along-track t is on (0, seg_len)
            d, frac = perp_distance_to_segment(e, n, ax, ay, bx, by)
            if 0.0 <= frac <= 1.0:
                xtes.append(d)
        p50, p95, mx, n = quantiles(xtes)
        leg_xte.append((k, p50, p95, mx, n))

    # cmd_drive analysis: rotate-in-place fraction, control rate, omega sign flips.
    cmd = streams.get("/cmd_drive", [])
    wheel_radius = 0.535 / 2.0
    wheel_base = 0.637

    nav_cmd = [(t, m) for (t, m) in cmd if in_nav(t)]
    rot_count = 0
    fwd_count = 0
    omegas = []
    vs = []
    for t, m in nav_cmd:
        v = wheel_radius * (m.left_vel + m.right_vel) / 2.0
        omega = wheel_radius * (m.right_vel - m.left_vel) / wheel_base
        vs.append(v)
        omegas.append((t, omega))
        if abs(v) < 0.05 and abs(omega) > 0.1:
            rot_count += 1
        elif abs(v) > 0.05:
            fwd_count += 1
    rot_frac = rot_count / max(1, rot_count + fwd_count)

    # control rate (NAVIGATING only)
    rates_ms = []
    for i in range(1, len(nav_cmd)):
        rates_ms.append((nav_cmd[i][0] - nav_cmd[i - 1][0]) * 1000.0)
    cmd_p50, cmd_p95, cmd_max, _ = quantiles(rates_ms)
    cmd_rate_hz = (1000.0 / cmd_p50) if cmd_p50 else None

    # omega sign flips per second.
    flips = 0
    for i in range(1, len(omegas)):
        if omegas[i][1] * omegas[i - 1][1] < 0 and abs(omegas[i][1]) > 0.1:
            flips += 1
    nav_duration = sum((e - s) for s, e in nav_segs)
    flip_rate = flips / max(1.0, nav_duration)

    # RTK status (worst).
    fusion = streams.get("/fixposition/fusion", [])
    rtk_lost_count = 0
    for t, msg in fusion:
        try:
            g1 = msg.fpa_gnssant.gnss1_status
            g2 = msg.fpa_gnssant.gnss2_status
        except AttributeError:
            # field path may differ across driver versions
            g1 = getattr(msg, "gnss1_status", 8)
            g2 = getattr(msg, "gnss2_status", 8)
        if g1 != 8 or g2 != 8:
            rtk_lost_count += 1

    # Warn log scan.
    warns = streams.get("/tactical/logging/warn", [])
    info = streams.get("/tactical/logging/info", [])
    warn_lines = [(t, m.data) for t, m in warns]
    info_lines = [(t, m.data) for t, m in info]

    # Route completion event.
    rc = streams.get("/tactical/robot/route_completed", [])
    completed = any(m.data for _, m in rc)

    # --- Write report ----------------------------------------------------------
    out = run_dir / "analysis_report.md"
    lines = []
    lines.append(f"# WP-follower KPI report — `{run_dir.name}`")
    lines.append("")
    lines.append(f"- bag duration (NAVIGATING only): **{nav_duration:.1f} s**")
    lines.append(f"- route: `{route_name or 'unknown'}` ({len(wps_enu)} waypoints)")
    lines.append(f"- route completed: **{'yes' if completed else 'no'}**")
    lines.append(f"- ENU origin (local frame): lat={lat0:.7f}, lon={lon0:.7f}")
    lines.append("")
    lines.append("## Waypoint accuracy")
    lines.append("")
    lines.append("| # | min approach | within 0.5 m? | overshoot past WP |")
    lines.append("|---|-------------:|:-------------:|------------------:|")
    for wp in wp_metrics:
        mn = f"{wp['min_dist_m']:.2f} m" if wp["min_dist_m"] is not None else "—"
        ov = f"{wp['overshoot_m']:.2f} m" if wp["overshoot_m"] is not None else "—"
        ok = "✓" if wp["reached_within_05m"] else "✗"
        lines.append(f"| {wp['idx']} | {mn} | {ok} | {ov} |")
    lines.append("")
    lines.append("## Path efficiency")
    lines.append("")
    lines.append(f"- driven path length (NAVIGATING-only): **{path_len:.2f} m**")
    lines.append(f"- straight-line WP-to-WP sum: **{straight_len:.2f} m**")
    lines.append(f"- ratio: **{ratio:.2f}×**  *(strict customer target ≤ 1.5×)*")
    lines.append("")
    lines.append("## Cross-track error per leg")
    lines.append("")
    lines.append("| leg | p50 | p95 | max | samples |")
    lines.append("|-----|----:|----:|----:|--------:|")
    for k, p50, p95, mx, n in leg_xte:
        p50s = f"{p50:.2f}" if p50 is not None else "—"
        p95s = f"{p95:.2f}" if p95 is not None else "—"
        mxs = f"{mx:.2f}" if mx is not None else "—"
        lines.append(f"| {k-1}→{k} | {p50s} m | {p95s} m | {mxs} m | {n} |")
    lines.append("")
    lines.append("## Controller behaviour")
    lines.append("")
    lines.append(f"- /cmd_drive median tick: **{cmd_p50:.0f} ms** "
                 f"(rate ≈ {cmd_rate_hz:.1f} Hz), p95 {cmd_p95:.0f} ms, max {cmd_max:.0f} ms")
    lines.append(f"- rotate-in-place fraction (of NAVIGATING ticks): **{rot_frac*100:.1f}%**")
    lines.append(f"- ω sign flips per second (steering oscillation proxy): **{flip_rate:.2f}/s**")
    if vs:
        v_p50, v_p95, v_max, _ = quantiles([abs(v) for v in vs])
        lines.append(f"- |linear speed| during NAVIGATING: p50 {v_p50:.2f} m/s, "
                     f"p95 {v_p95:.2f} m/s, max {v_max:.2f} m/s")
    lines.append("")
    lines.append("## Safety / system health")
    lines.append("")
    lines.append(f"- /fixposition/fusion ticks with either GNSS antenna not RTK Fixed (status≠8): "
                 f"**{rtk_lost_count}** (of {len(fusion)})")
    lines.append(f"- /tactical/logging/warn entries: **{len(warn_lines)}**")
    lines.append(f"- /tactical/logging/info entries: {len(info_lines)}")
    if warn_lines:
        lines.append("")
        lines.append("### Warn-log excerpts (first 20)")
        for t, txt in warn_lines[:20]:
            lines.append(f"  - `t+{t - traj[0][0]:.1f}s`: {txt}")
    lines.append("")
    lines.append("## Predictions vs measurements")
    lines.append("")
    lines.append("| # | prediction | result |")
    lines.append("|---|------------|--------|")
    p1_ok = all((wp["overshoot_m"] or 0) < 0.8 for wp in wp_metrics if wp["overshoot_m"] is not None)
    lines.append(f"| P1 | overshoot 0.3–0.8 m past each WP | "
                 f"{'within band' if p1_ok else 'OUT OF BAND — investigate'} |")
    lines.append(f"| P2 | >50% rotate-in-place | "
                 f"{'CONFIRMED' if rot_frac > 0.5 else 'not confirmed (good)'} "
                 f"({rot_frac*100:.0f}%) |")
    lines.append(f"| P3 | path ≥ 5× straight-line | "
                 f"{'CONFIRMED' if ratio >= 5 else 'not confirmed'} ({ratio:.2f}×) |")
    lines.append(f"| P6 | cmd_drive ≥ 9.5 Hz | "
                 f"{'OK' if (cmd_rate_hz or 0) >= 9.5 else 'REGRESSION'} ({cmd_rate_hz:.1f} Hz) |")
    lines.append(f"| P7 | RTK steady at 8/8 | "
                 f"{'OK' if rtk_lost_count == 0 else f'drops: {rtk_lost_count} ticks'} |")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("*Auto-generated by `analyze_wp_bag.py`. Reads the bag, ignores TF, "
                 "uses /fixposition/odometry_llh for position (lat/lon → local ENU "
                 "with first fix as origin).*")

    out.write_text("\n".join(lines))
    print(f"[analyze] wrote {out}")


if __name__ == "__main__":
    main()
