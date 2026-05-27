#!/usr/bin/env python3
"""
Erweiterter Hindernis-Detector für die vordere Tiefenkamera
(aktuell: Luxonis OAK-D Lite über depthai_ros_driver).

Funktionen:
- Nutzt Depth-Image, betrachtet eine ROI vor dem Roboter.
- Teilt die ROI in 8 Sektoren:
  - 4 Spalten (links → rechts)
  - 2 Distanzbereiche (nah, weit)
- Loggt, in welchen Sektoren Hindernisse gefunden werden.
- Berechnet einen groben „Links/Rechts"-Offset des Hindernisschwerpunkts.
- Publiziert:
  - /camera/depth/obstacle_debug (sensor_msgs/Image, bgr8) zur Visualisierung in rqt_image_view

!!!!!!!!!! AKTUELL !!!!!!!!!!!!!!
AUF
    /obstacle_sectors ein UInt8MultiArray mit len=10:
  Index 0..4: nah, von links nach rechts
    Index 5..9: weit, von links nach rechts
"""

import rclpy
from rclpy.node import Node
from collections import deque

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Bool, UInt8MultiArray


class SectorObstacleDetector(Node):
    def __init__(self):
        super().__init__('sector_obstacle_detector')

        # -----------------------------
        # Parameter-Definition
        # -----------------------------
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('near_distance_m', 0.4)      # Grenze für "nah"
        self.declare_parameter('far_distance_m', 1.00)       # Grenze für "weit"
        self.declare_parameter('min_obstacle_pixels', 500)   # globale Schwelle
        self.declare_parameter('min_sector_pixels', 300)     # Schwelle pro Feld
        self.declare_parameter('debug_image_topic', '/camera/camera/depth/obstacle_debug')
        self.declare_parameter('num_cols', 5)                # 5 Felder quer

        # Outdoor robustness parameters
        self.declare_parameter('min_valid_ratio', 0.25)
        self.declare_parameter('temporal_window', 5)
        self.declare_parameter('temporal_threshold', 3)
        self.declare_parameter('morphology_kernel_size', 5)

        # Ground plane rejection — tune camera_height_m and camera_tilt_deg
        # to match the physical camera mount on the robot.
        self.declare_parameter('ground_plane.enabled', True)
        self.declare_parameter('ground_plane.camera_height_m', 0.35)  # mount height above ground
        self.declare_parameter('ground_plane.camera_tilt_deg', 25.0)  # degrees below horizontal
        self.declare_parameter('ground_plane.camera_vfov_deg', 57.0)  # vertical FOV (D455 ≈ 57, OAK-D Lite ≈ 58)
        self.declare_parameter('ground_plane.tolerance_m', 0.25)      # depth band around expected ground

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.near_distance_m = self.get_parameter('near_distance_m').get_parameter_value().double_value
        self.far_distance_m = self.get_parameter('far_distance_m').get_parameter_value().double_value
        self.min_obstacle_pixels = self.get_parameter('min_obstacle_pixels').get_parameter_value().integer_value
        self.min_sector_pixels = self.get_parameter('min_sector_pixels').get_parameter_value().integer_value
        debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.num_cols = self.get_parameter('num_cols').get_parameter_value().integer_value

        self.min_valid_ratio = self.get_parameter('min_valid_ratio').get_parameter_value().double_value
        self.temporal_window = self.get_parameter('temporal_window').get_parameter_value().integer_value
        self.temporal_threshold = self.get_parameter('temporal_threshold').get_parameter_value().integer_value
        morphology_ks = self.get_parameter('morphology_kernel_size').get_parameter_value().integer_value

        self.ground_enabled = self.get_parameter('ground_plane.enabled').get_parameter_value().bool_value
        self.ground_height_m = self.get_parameter('ground_plane.camera_height_m').get_parameter_value().double_value
        self.ground_tilt_deg = self.get_parameter('ground_plane.camera_tilt_deg').get_parameter_value().double_value
        self.ground_vfov_deg = self.get_parameter('ground_plane.camera_vfov_deg').get_parameter_value().double_value
        self.ground_tolerance_m = self.get_parameter('ground_plane.tolerance_m').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Pre-computed morphological structuring element
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morphology_ks, morphology_ks)
        )

        # Temporal filter history (only valid frames are appended)
        self._history_near = deque(maxlen=self.temporal_window)
        self._history_far = deque(maxlen=self.temporal_window)

        # Rate-limited logging for degraded-frame streaks
        self._degraded_streak = 0

        # Ground depth map cache (computed once on first frame)
        self._ground_depth_map = None
        self._ground_depth_shape = None

        # Use SENSOR_DATA QoS profile for better reliability over network
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, sensor_qos
        )

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.sector_pub = self.create_publisher(UInt8MultiArray, '/obstacle_sectors', 10)
        self.debug_img_pub = self.create_publisher(Image, debug_image_topic, 10)

        self.get_logger().info(
            f"SectorObstacleDetector läuft. Depth-Topic: {depth_topic}, "
            f"Debug-Image: {debug_image_topic}, "
            f"min_valid_ratio={self.min_valid_ratio}, "
            f"temporal={self.temporal_threshold}/{self.temporal_window}, "
            f"morphology_kernel={morphology_ks}, "
            f"ground_plane={'ON' if self.ground_enabled else 'OFF'}"
            f"{f' h={self.ground_height_m}m tilt={self.ground_tilt_deg}° tol={self.ground_tolerance_m}m' if self.ground_enabled else ''}"
        )

    # ------------------------------------------------------------------
    # Ground plane — adaptive from actual depth data
    # ------------------------------------------------------------------
    def _compute_adaptive_ground(self, roi_m, valid_mask, roi_h, roi_w):
        """Estimate actual ground depth per row from the image itself.

        At each row the majority of pixels show the ground surface, so the
        row median gives us the real ground depth.  A monotonicity check
        (ground depth must decrease from top→bottom) rejects rows that
        contain a real obstacle spanning most of the image width.
        """
        # Row medians (vectorized)
        roi_nan = np.where(valid_mask, roi_m, np.nan)
        with np.errstate(all='ignore'):
            row_medians = np.nanmedian(roi_nan, axis=1).astype(np.float32)

        # Require ≥20 % valid pixels in a row
        valid_per_row = np.sum(valid_mask, axis=1)
        row_medians[valid_per_row < roi_w * 0.2] = np.nan

        # Monotonicity: ground depth must decrease top→bottom
        # (far ground at top rows, close ground at bottom rows).
        # A row whose median is LARGER than the row above likely
        # contains an obstacle — mark it invalid.
        cleaned = row_medians.copy()
        for r in range(1, roi_h):
            if np.isnan(cleaned[r]) or np.isnan(cleaned[r - 1]):
                continue
            if cleaned[r] > cleaned[r - 1] + 0.05:
                cleaned[r] = np.nan

        # Need enough valid rows to form a ground model
        valid_idx = ~np.isnan(cleaned)
        if np.sum(valid_idx) < max(5, roi_h * 0.15):
            return None

        indices = np.arange(roi_h, dtype=np.float32)
        ground_1d = np.interp(indices, indices[valid_idx], cleaned[valid_idx])

        # Smooth with moving average
        k = min(15, max(3, roi_h // 10))
        pad = k // 2
        padded = np.pad(ground_1d, pad, mode='edge')
        ground_1d = np.convolve(padded, np.ones(k, dtype=np.float32) / k,
                                mode='valid')[:roi_h]

        return np.tile(ground_1d.reshape(-1, 1), (1, roi_w))

    def _compute_geometric_ground(self, img_h, img_w, roi_y1, roi_y2, roi_w):
        """Fallback: pinhole-model ground depth map (needs mount params)."""
        fy = img_h / (2.0 * np.tan(np.radians(self.ground_vfov_deg / 2.0)))
        cy = img_h / 2.0
        theta = np.radians(self.ground_tilt_deg)
        sin_t, cos_t = np.sin(theta), np.cos(theta)

        v = np.arange(roi_y1, roi_y2, dtype=np.float32)
        denom = fy * sin_t + (v - cy) * cos_t

        gd = np.full(len(v), np.inf, dtype=np.float32)
        ok = denom > 1e-3
        gd[ok] = (self.ground_height_m * fy) / denom[ok]
        return np.tile(gd.reshape(-1, 1), (1, roi_w))

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def depth_callback(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth is None:
            return

        h, w = depth.shape[:2]

        roi_x1 = int(w * 0.1)
        roi_x2 = int(w * 0.9)
        roi_y1 = int(h * 0.3)
        roi_y2 = int(h * 0.85)

        roi = depth[roi_y1:roi_y2, roi_x1:roi_x2]
        roi_h, roi_w = roi.shape[:2]

        # Depth → Meter
        if msg.encoding == '16UC1':
            roi_m = roi.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            roi_m = roi.astype(np.float32)
        else:
            self.get_logger().warn(
                f"Unbekanntes Depth-Encoding '{msg.encoding}', "
                "interpretiere als Meter (float)."
            )
            roi_m = roi.astype(np.float32)

        valid_mask = roi_m > 0.3

        # ── FILTER 1: frame-level confidence ─────────────────────────
        total_pixels = roi_h * roi_w
        valid_count = int(np.count_nonzero(valid_mask))
        valid_ratio = valid_count / total_pixels if total_pixels > 0 else 0.0

        if valid_ratio < self.min_valid_ratio:
            self._degraded_streak += 1
            if self._degraded_streak == 1 or self._degraded_streak % 30 == 0:
                self.get_logger().warn(
                    f"Depth frame degraded: valid_ratio={valid_ratio:.1%} "
                    f"< {self.min_valid_ratio:.0%} "
                    f"({self._degraded_streak} consecutive) — "
                    f"publishing all-confirmed to preserve LiDAR passthrough"
                )

            sector_msg = UInt8MultiArray()
            sector_msg.data = [1] * (2 * self.num_cols)
            self.sector_pub.publish(sector_msg)

            obstacle_msg = Bool()
            obstacle_msg.data = False
            self.obstacle_pub.publish(obstacle_msg)

            debug_img = self._create_degraded_debug_image(roi_m, valid_ratio)
            if debug_img is not None:
                self.debug_img_pub.publish(
                    self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                )
            return

        if self._degraded_streak > 0:
            self.get_logger().info(
                f"Depth quality restored after {self._degraded_streak} degraded frames"
            )
            self._degraded_streak = 0

        # ── FILTER 2: ground plane rejection ─────────────────────────
        # Adaptive: learn actual ground depth from row medians.
        # Geometric fallback when adaptive has insufficient data.
        if self.ground_enabled:
            ground_depth_2d = self._compute_adaptive_ground(
                roi_m, valid_mask, roi_h, roi_w
            )
            if ground_depth_2d is None:
                need_shape = (roi_h, roi_w)
                if self._ground_depth_map is None or self._ground_depth_shape != need_shape:
                    self._ground_depth_map = self._compute_geometric_ground(
                        h, w, roi_y1, roi_y2, roi_w
                    )
                    self._ground_depth_shape = need_shape
                ground_depth_2d = self._ground_depth_map

            ground_mask = (
                valid_mask
                & (np.abs(roi_m - ground_depth_2d) < self.ground_tolerance_m)
            )
        else:
            ground_mask = np.zeros((roi_h, roi_w), dtype=bool)

        obstacle_valid = valid_mask & ~ground_mask

        # ── FILTER 3: morphological opening ──────────────────────────
        near_mask_raw = (roi_m < self.near_distance_m) & obstacle_valid
        far_mask_raw = (
            (roi_m >= self.near_distance_m)
            & (roi_m < self.far_distance_m)
            & obstacle_valid
        )

        near_mask = cv2.morphologyEx(
            near_mask_raw.astype(np.uint8), cv2.MORPH_OPEN, self._morph_kernel
        ).astype(bool)
        far_mask = cv2.morphologyEx(
            far_mask_raw.astype(np.uint8), cv2.MORPH_OPEN, self._morph_kernel
        ).astype(bool)

        # --------------------------------------------------
        # 5 Spalten × 2 Tiefenbereiche → 10 Felder (Bool)
        # --------------------------------------------------
        xs = np.arange(roi_w, dtype=np.int32)
        xs_grid = np.tile(xs, (roi_h, 1))
        col_indices = (xs_grid * self.num_cols // roi_w).clip(0, self.num_cols - 1)

        sector_near = np.zeros(self.num_cols, dtype=np.int32)
        sector_far = np.zeros(self.num_cols, dtype=np.int32)

        for col in range(self.num_cols):
            mask_col_near = near_mask & (col_indices == col)
            sector_near[col] = int(np.count_nonzero(mask_col_near))

            mask_col_far = far_mask & (col_indices == col)
            sector_far[col] = int(np.count_nonzero(mask_col_far))

        field_near_raw = (sector_near >= self.min_sector_pixels).astype(np.uint8)
        field_far_raw = (sector_far >= self.min_sector_pixels).astype(np.uint8)

        # ── FILTER 4: temporal persistence ───────────────────────────
        self._history_near.append(field_near_raw.copy())
        self._history_far.append(field_far_raw.copy())

        if len(self._history_near) >= self.temporal_threshold:
            stack_near = np.array(self._history_near)
            stack_far = np.array(self._history_far)
            field_near = (stack_near.sum(axis=0) >= self.temporal_threshold).astype(np.uint8)
            field_far = (stack_far.sum(axis=0) >= self.temporal_threshold).astype(np.uint8)
        else:
            field_near = field_near_raw
            field_far = field_far_raw

        # --------------------------------------------------
        # ROS Messages
        # --------------------------------------------------
        obstacle_msg = Bool()
        obstacle_msg.data = bool(np.any(field_near))
        self.obstacle_pub.publish(obstacle_msg)

        sector_msg = UInt8MultiArray()
        sector_msg.data = list(field_near) + list(field_far)
        self.sector_pub.publish(sector_msg)

        # Schwerpunkt-Logging
        obstacle_mask = near_mask
        obstacle_indices = np.where(obstacle_mask)
        if obstacle_indices[0].size > 0:
            mean_x = float(xs_grid[obstacle_indices].mean())
            center_offset_norm = (mean_x - roi_w / 2.0) / (roi_w / 2.0)
        else:
            center_offset_norm = None

        if np.any(field_near) or np.any(field_far):
            active_near = [i for i, v in enumerate(field_near) if v]
            active_far = [i for i, v in enumerate(field_far) if v]
            if center_offset_norm is not None:
                if center_offset_norm < -0.3:
                    direction = "links"
                elif center_offset_norm > 0.3:
                    direction = "rechts"
                else:
                    direction = "zentral"
                self.get_logger().info(
                    f"Aktive Felder nah={active_near}, weit={active_far}, "
                    f"Schwerpunkt: {direction} (Offset={center_offset_norm:.2f})"
                )
            else:
                self.get_logger().info(
                    f"Aktive Felder nah={active_near}, weit={active_far}, "
                    f"kein Schwerpunkt berechenbar."
                )

        # Debug image
        debug_img = self.create_debug_image(
            roi_m, valid_mask, near_mask, far_mask,
            field_near, field_far,
            field_near_raw, field_far_raw,
            valid_ratio, ground_mask
        )
        if debug_img is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_img_pub.publish(debug_msg)

    # ------------------------------------------------------------------
    # Debug images
    # ------------------------------------------------------------------
    def create_debug_image(self, roi_m, valid_mask, near_mask, far_mask,
                           field_near, field_far,
                           field_near_raw, field_far_raw,
                           valid_ratio, ground_mask):

        roi_h, roi_w = roi_m.shape[:2]
        debug_img = cv2.normalize(roi_m, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)

        # Ground pixels → cyan tint (filtered out, not obstacles)
        debug_img[ground_mask] = (180, 140, 0)

        # Obstacle pixels
        debug_img[far_mask] = (0, 100, 0)
        debug_img[near_mask] = (0, 0, 200)

        # Grid lines
        num_rows = 5
        row_height = roi_h / num_rows
        for r in range(num_rows):
            y_start = int(r * row_height)
            y_end = int((r + 1) * row_height) - 1
            cv2.rectangle(debug_img, (0, y_start), (roi_w - 1, y_end), (100, 100, 100), 1)
        num_cols = 10
        col_width_fine = roi_w / num_cols
        for r in range(num_cols):
            x_start = int(r * col_width_fine)
            x_end = int((r + 1) * col_width_fine) - 1
            cv2.rectangle(debug_img, (x_start, 0), (x_end, roi_h - 1), (100, 100, 100), 1)

        # Sector borders
        col_width = roi_w / self.num_cols
        for c in range(self.num_cols):
            x_start = int(c * col_width)
            x_end = int((c + 1) * col_width) - 1
            if field_near[c]:
                color_rect = (0, 0, 255)
            elif field_near_raw[c]:
                color_rect = (0, 200, 255)
            elif field_far[c]:
                color_rect = (0, 255, 0)
            elif field_far_raw[c]:
                color_rect = (0, 200, 255)
            else:
                color_rect = (80, 80, 80)
            cv2.rectangle(debug_img, (x_start, 0), (x_end, roi_h - 1), color_rect, 2)
            cv2.putText(debug_img, f"{c}", (x_start + 5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Status bar
        gnd_pct = np.count_nonzero(ground_mask) / ground_mask.size * 100 if ground_mask.size else 0
        cv2.rectangle(debug_img, (0, roi_h - 22), (roi_w, roi_h), (0, 0, 0), -1)
        cv2.putText(
            debug_img,
            f"valid={valid_ratio:.0%}  ground={gnd_pct:.0f}%  "
            f"temporal={self.temporal_threshold}/{self.temporal_window}",
            (4, roi_h - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1
        )

        return debug_img

    def _create_degraded_debug_image(self, roi_m, valid_ratio):
        roi_h, roi_w = roi_m.shape[:2]
        debug_img = cv2.normalize(roi_m, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)

        overlay = debug_img.copy()
        overlay[:] = (0, 0, 80)
        cv2.addWeighted(overlay, 0.4, debug_img, 0.6, 0, debug_img)

        cv2.rectangle(debug_img, (0, roi_h // 2 - 18), (roi_w, roi_h // 2 + 18), (0, 0, 160), -1)
        cv2.putText(
            debug_img,
            f"LOW CONFIDENCE  valid={valid_ratio:.0%}",
            (10, roi_h // 2 + 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )

        return debug_img


def main(args=None):
    rclpy.init(args=args)
    node = SectorObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
