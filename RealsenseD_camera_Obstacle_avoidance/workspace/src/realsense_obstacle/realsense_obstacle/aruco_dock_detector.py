#!/usr/bin/env python3
"""
ArUco-Marker-Detektor für die Präzisions-Andockung an der Ladestation.

Detects a single ArUco marker (default DICT_4X4_50, ID 0, 150 mm) in the
OAK-D Lite rectified RGB stream and publishes its pose relative to the camera
on /docking/aruco_pose (geometry_msgs/PoseStamped, camera optical frame).

This node is purely additive: it only subscribes to the camera image +
camera_info and publishes a pose topic. It does NOT command motion. The
docking control (tactical_wp_follower) consumes the pose only when ArUco
docking is explicitly enabled; if this node is absent or never sees the
marker, docking falls back to the existing GPS line-follow unchanged.

Pose convention (camera optical frame, REP-103 optical):
  +x = right, +y = down, +z = forward (out of the lens).
  position.z is therefore the range to the marker; position.x its lateral
  offset. orientation is the marker plane orientation (its +z is the marker
  surface normal pointing back toward the camera).

Pose is estimated with cv2.solvePnP on the four marker corners (stable across
OpenCV 4.6 / 4.7+; avoids the deprecated estimatePoseSingleMarkers and the
contrib/main aruco API churn). Requires camera intrinsics from camera_info.
"""

import math

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped


def _rotmat_to_quat(R):
    """Rotation matrix (3x3) -> (x, y, z, w) quaternion. Self-contained so we
    don't pull in tf_transformations / transforms3d as a new dependency."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


class ArucoDockDetector(Node):
    def __init__(self):
        super().__init__('aruco_dock_detector')

        # -----------------------------
        # Parameter
        # -----------------------------
        self.declare_parameter('rgb_topic', '/oak/rgb/image_rect')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('pose_topic', '/docking/aruco_pose')
        self.declare_parameter('marker_id', 0)
        # Optional second marker for a ROBUST dock normal: with two coplanar markers the
        # approach angle comes from the BASELINE between their (clean) positions, not from
        # one marker's noisy rotation. Set to -1 to disable (single-marker mode).
        self.declare_parameter('marker_id2', 1)
        self.declare_parameter('marker_length_m', 0.15)     # black square side, see marker PDF
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_topic', '/docking/aruco_debug')
        self.declare_parameter('max_range_m', 5.0)          # reject implausible pose
        self.declare_parameter('max_reproj_error_px', 4.0)  # reject ambiguous/garbage pose

        self._rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self._cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self._pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self._marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self._marker_id2 = self.get_parameter('marker_id2').get_parameter_value().integer_value
        self._marker_length = self.get_parameter('marker_length_m').get_parameter_value().double_value
        self._dict_name = self.get_parameter('dictionary').get_parameter_value().string_value
        self._publish_debug = self.get_parameter('publish_debug').get_parameter_value().bool_value
        self._debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value
        self._max_range = self.get_parameter('max_range_m').get_parameter_value().double_value
        self._max_reproj = self.get_parameter('max_reproj_error_px').get_parameter_value().double_value

        self._bridge = CvBridge()
        self._K = None          # 3x3 intrinsics
        self._D = None          # distortion coeffs
        self._info_warned = False

        # Marker corner object points (centred at marker origin), matching the
        # OpenCV aruco corner order: top-left, top-right, bottom-right, bottom-left.
        h = self._marker_length / 2.0
        self._obj_points = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

        self._setup_aruco()

        self._pose_pub = self.create_publisher(PoseStamped, self._pose_topic, 10)
        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, self._debug_topic, 1)

        self.create_subscription(CameraInfo, self._cam_info_topic, self._info_cb, 10)
        self.create_subscription(Image, self._rgb_topic, self._image_cb, 10)

        # Throttle the "marker not seen" log so it doesn't spam at frame rate.
        self._seen_log_counter = 0

        self.get_logger().info(
            f"aruco_dock_detector up: dict={self._dict_name} id={self._marker_id} "
            f"len={self._marker_length*1000:.0f}mm rgb={self._rgb_topic} "
            f"info={self._cam_info_topic} -> {self._pose_topic}"
        )

    def _setup_aruco(self):
        """Build the ArUco dictionary + detector, tolerant of the OpenCV
        4.6 (Dictionary_get / DetectorParameters_create) vs 4.7+
        (getPredefinedDictionary / ArucoDetector) API split."""
        aruco = cv2.aruco
        dict_id = getattr(aruco, self._dict_name)
        if hasattr(aruco, 'ArucoDetector') and hasattr(aruco, 'getPredefinedDictionary'):
            self._aruco_mode = 'new'
            self._dictionary = aruco.getPredefinedDictionary(dict_id)
            params = aruco.DetectorParameters()
            self._detector = aruco.ArucoDetector(self._dictionary, params)
        else:
            self._aruco_mode = 'old'
            self._dictionary = aruco.Dictionary_get(dict_id)
            self._params = aruco.DetectorParameters_create()
        # Prefer the planar-square solver when available (best for a single
        # square marker); fall back to the iterative default otherwise.
        self._pnp_flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)

    def _info_cb(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64)
        # /oak/rgb/image_rect is rectified -> distortion is effectively zero,
        # but honour whatever camera_info advertises.
        self._D = d if d.size > 0 else np.zeros((5,), dtype=np.float64)

    def _detect(self, gray):
        if self._aruco_mode == 'new':
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dictionary, parameters=self._params)
        return corners, ids

    def _image_cb(self, msg: Image):
        if self._K is None:
            if not self._info_warned:
                self.get_logger().warn(
                    f"Waiting for camera_info on {self._cam_info_topic} — "
                    f"cannot estimate marker pose without intrinsics.")
                self._info_warned = True
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001 — bad frame must not kill the node
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)

        # Collect corner sets for our marker ID(s).
        found = {}
        if ids is not None:
            ids_flat = ids.flatten()
            for i, mid in enumerate(ids_flat):
                mid = int(mid)
                if mid == self._marker_id or (self._marker_id2 >= 0 and mid == self._marker_id2):
                    found[mid] = corners[i].reshape(4, 2).astype(np.float32)

        if not found:
            self._seen_log_counter += 1
            if self._seen_log_counter % 60 == 0:  # ~ every few seconds
                self.get_logger().debug("dock marker not in view")
            self._publish_debug_image(frame, corners, ids, None, None)
            return

        # Per-marker pose (range + reprojection gated).
        est = {mid: self._estimate(c) for mid, c in found.items()}
        est = {mid: v for mid, v in est.items() if v is not None}
        if not est:
            self._publish_debug_image(frame, corners, ids, None, None)
            return

        t0 = est.get(self._marker_id)
        t1 = est.get(self._marker_id2) if self._marker_id2 >= 0 else None

        # Throttled detection summary (helps verify both markers are seen + which mode).
        self._seen_log_counter += 1
        if self._seen_log_counter % 15 == 0:
            seen = sorted(est.keys())
            mode = "FUSED(2)" if (t0 is not None and t1 is not None) else "single(1)"
            zs = ", ".join(f"id{m}:z={est[m][1][2]:.2f}" for m in seen)
            self.get_logger().info(f"[ARUCODIAG] markers seen={seen} mode={mode} [{zs}]")

        if t0 is not None and t1 is not None:
            # BOTH markers: dock pose from the baseline (robust normal). position = pair
            # midpoint, orientation = dock frame (z = outward normal toward camera).
            fused = self._fuse_two(t0[1], t1[1])
            if fused is not None:
                center, R = fused
                self._publish_pose(msg, center, R)
                # debug: draw both
                self._publish_debug_image(frame, corners, ids, t0[0], t0[1])
                return

        # Two-marker mode: REQUIRE both. A single marker's orientation is noisy and feeds
        # the normal-aligned controller garbage (±45° spins seen 2026-06-06). Publish
        # NOTHING when only one is visible → the controller holds (move-measure-move) and
        # waits for both to come back, instead of acting on noise.
        if self._marker_id2 >= 0:
            if self._seen_log_counter % 15 == 0:
                self.get_logger().info(
                    "[ARUCODIAG] only ONE marker visible in two-marker mode → holding "
                    "(no pose published; need both for a trustworthy normal)")
            self._publish_debug_image(frame, corners, ids, None, None)
            return

        # Single-marker mode (marker_id2 < 0) only: publish the single pose.
        single = t0 if t0 is not None else t1
        rvec, tvec = single
        R, _ = cv2.Rodrigues(rvec)
        self._publish_pose(msg, tvec, R)
        self._publish_debug_image(frame, corners, ids, rvec, tvec)

    def _estimate(self, target_corners):
        """solvePnP for one marker, with range + reprojection gates. Returns (rvec, tvec)
        or None if rejected."""
        ok, rvec, tvec = cv2.solvePnP(
            self._obj_points, target_corners, self._K, self._D, flags=self._pnp_flag)
        if not ok:
            return None
        tvec = tvec.reshape(3)
        rng = float(tvec[2])
        if rng <= 0.0 or rng > self._max_range:
            self.get_logger().warn(f"rejecting marker pose: range {rng:.2f}m out of bounds")
            return None
        proj, _ = cv2.projectPoints(self._obj_points, rvec, tvec.reshape(3, 1), self._K, self._D)
        reproj_err = float(np.mean(np.linalg.norm(proj.reshape(4, 2) - target_corners, axis=1)))
        if reproj_err > self._max_reproj:
            self.get_logger().warn(
                f"rejecting marker pose: reproj error {reproj_err:.1f}px > {self._max_reproj:.1f}px")
            return None
        return rvec, tvec

    def _fuse_two(self, t0, t1):
        """Fuse two coplanar marker positions into a dock pose. The approach NORMAL comes
        from the baseline between the two (clean) positions — far more robust than a single
        planar marker's rotation. Returns (center, R) with R columns [x, y, z], z = outward
        normal pointing toward the camera. None if degenerate."""
        center = (t0 + t1) / 2.0
        baseline = t1 - t0
        nb = float(np.linalg.norm(baseline))
        if nb < 1e-6:
            return None
        baseline_dir = baseline / nb
        up = np.array([0.0, -1.0, 0.0])               # camera 'up' = -y (optical frame)
        z_axis = np.cross(baseline_dir, up)           # normal: perp to baseline, horizontal
        nz = float(np.linalg.norm(z_axis))
        if nz < 1e-6:
            return None
        z_axis /= nz
        if z_axis[2] > 0.0:                            # must point toward camera (-z)
            z_axis = -z_axis
        x_axis = np.cross(up, z_axis)
        x_axis /= float(np.linalg.norm(x_axis))
        y_axis = np.cross(z_axis, x_axis)
        R = np.column_stack([x_axis, y_axis, z_axis])
        return center, R

    def _publish_pose(self, msg, tvec, R):
        qx, qy, qz, qw = _rotmat_to_quat(R)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = msg.header.frame_id or 'oak_rgb_camera_optical_frame'
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pose_pub.publish(pose)

    def _publish_debug_image(self, frame, corners, ids, rvec, tvec):
        if self._debug_pub is None:
            return
        try:
            img = frame
            if ids is not None and len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
            if rvec is not None and tvec is not None:
                cv2.drawFrameAxes(img, self._K, self._D, rvec, tvec.reshape(3, 1),
                                  self._marker_length * 0.5)
                txt = f"x={tvec[0]*100:+.1f}cm z={tvec[2]*100:.1f}cm"
                cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2, cv2.LINE_AA)
            out = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self._debug_pub.publish(out)
        except Exception as e:  # noqa: BLE001 — debug must never break detection
            self.get_logger().debug(f"debug image publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
