"""Dock alignment from the fused ArUco marker pose.

Mirror of tactical_wp_follower._marker_dock_errors so the web UI shows the SAME
cross / heading / perp the docking controller uses. Lets the operator centre + square
the robot via the app before saving the charge point (no terminal script needed).
"""
import math

CROSS_TOL_M = 0.03      # 3 cm
HEAD_TOL_DEG = 3.0      # 3 deg
STALE_S = 1.0           # marker pose older than this -> not trustworthy


def dock_errors(pose):
    """Robot pose in the dock frame from the marker pose (camera optical frame).

    Returns (cross_m, perp_m, heading_deg):
      cross   = sideways offset from the dock centre-line (+ = robot right of centre)
      perp    = perpendicular distance to the marker plane (= the docked standoff)
      heading = robot heading vs the marker normal (deg, 0 = square/head-on)
    """
    q = pose.orientation
    p = pose.position
    x, y, z, w = q.x, q.y, q.z, q.w
    r00 = 1 - 2 * (y * y + z * z); r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w);     r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w);     r22 = 1 - 2 * (x * x + y * y)
    px, py, pz = p.x, p.y, p.z
    cross = -(r00 * px + r10 * py + r20 * pz)
    perp = abs(-(r02 * px + r12 * py + r22 * pz))
    h = math.atan2(r20, r22) - math.pi
    while h > math.pi:
        h -= 2 * math.pi
    while h < -math.pi:
        h += 2 * math.pi
    return cross, perp, math.degrees(h)


def compute_align(pose, age_s, cross_tol_m=CROSS_TOL_M, head_tol_deg=HEAD_TOL_DEG):
    """Build the alignment dict for the UI. pose=None or stale -> not fused.

    Tolerances are passed in (live from aruco_dock.yaml) so the green 'aligned' light
    can be tuned without a rebuild.
    """
    if pose is None or age_s is None or age_s > STALE_S:
        return {
            "fused": False, "cross_cm": None, "heading_deg": None,
            "perp_m": None, "aligned": False,
        }
    cross, perp, hdeg = dock_errors(pose)
    aligned = abs(cross) < cross_tol_m and abs(hdeg) < head_tol_deg
    return {
        "fused": True,
        "cross_cm": round(cross * 100.0, 1),
        "heading_deg": round(hdeg, 1),
        "perp_m": round(perp, 3),
        "aligned": aligned,
    }
