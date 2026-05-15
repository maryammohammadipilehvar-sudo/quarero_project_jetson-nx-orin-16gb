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
- Berechnet einen groben „Links/Rechts“-Offset des Hindernisschwerpunkts.
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
        self.declare_parameter('min_obstacle_pixels', 200)   # globale Schwelle, relativ klein
        self.declare_parameter('min_sector_pixels', 50)      # Schwelle pro Feld
        self.declare_parameter('debug_image_topic', '/camera/camera/depth/obstacle_debug')
        self.declare_parameter('num_cols', 5)                # 5 Felder quer

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.near_distance_m = self.get_parameter('near_distance_m').get_parameter_value().double_value
        self.far_distance_m = self.get_parameter('far_distance_m').get_parameter_value().double_value
        self.min_obstacle_pixels = self.get_parameter('min_obstacle_pixels').get_parameter_value().integer_value
        self.min_sector_pixels = self.get_parameter('min_sector_pixels').get_parameter_value().integer_value
        debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.num_cols = self.get_parameter('num_cols').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Use SENSOR_DATA QoS profile for better reliability over network
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Faster, accepts frame drops
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # Smaller queue to reduce latency
            durability=DurabilityPolicy.VOLATILE
        )

        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            sensor_qos
        )

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        # Publiziert 10 Bools: [5×nah, 5×weit], 1 = Hindernis in diesem Feld
        self.sector_pub = self.create_publisher(UInt8MultiArray, '/obstacle_sectors', 10)

        self.debug_img_pub = self.create_publisher(Image, debug_image_topic, 10)

        self.get_logger().info(
            f"SectorObstacleDetector läuft. Depth-Topic: {depth_topic}, Debug-Image: {debug_image_topic}"
        )


    def depth_callback(self, msg: Image):
        """Verarbeitet ein Depth-Image, segmentiert es in 5×2 Felder und setzt
        pro Feld ein Bool, wenn genug Hindernis-Pixel in dem Feld sind.
        Zusätzlich wird ein globales /obstacle_detected veröffentlicht.
        """
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth is None:
            return

        h, w = depth.shape[:2]

        # ROI vor dem Roboter
        # roi_x1 = int(w * 0.2)
        # roi_x2 = int(w * 0.8)
        # roi_y1 = int(h * 0.4)
        # roi_y2 = int(h * 0.95)
        roi_x1 = int(w * 0.1)
        roi_x2 = int(w * 0.9)
        roi_y1 = int(h * 0)
        roi_y2 = int(h * 0.5)

        roi = depth[roi_y1:roi_y2, roi_x1:roi_x2]
        roi_h, roi_w = roi.shape[:2]

        # Depth nach Meter:
        #   16UC1 (RealSense): Millimeter → /1000
        #   32FC1 (OAK depthai_ros_driver stereo): bereits in Metern
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

        # Gültige Pixel
        valid_mask = roi_m > 0.1

        # zwei Distanzbereiche
        near_mask = (roi_m < self.near_distance_m) & valid_mask
        far_mask = (roi_m >= self.near_distance_m) & (roi_m < self.far_distance_m) & valid_mask

        # Für das globale Flag werten wir "nah" aus
        obstacle_mask = near_mask
        num_obstacle_pixels = int(np.count_nonzero(obstacle_mask))

        # --------------------------------------------------
        # 5 Spalten × 2 Tiefenbereiche → 10 Felder (Bool)
        # --------------------------------------------------
        xs = np.arange(roi_w, dtype=np.int32)
        xs_grid = np.tile(xs, (roi_h, 1))
        # Grid Aggregation
        col_indices = (xs_grid * self.num_cols // roi_w).clip(0, self.num_cols - 1)

        # Zähler pro Feld
        sector_near = np.zeros(self.num_cols, dtype=np.int32)
        sector_far = np.zeros(self.num_cols, dtype=np.int32)

        for col in range(self.num_cols):
            # Nah-Bereich in Spalte col
            mask_col_near = near_mask & (col_indices == col)
            sector_near[col] = int(np.count_nonzero(mask_col_near))

            # Weit-Bereich in Spalte col
            mask_col_far = far_mask & (col_indices == col)
            sector_far[col] = int(np.count_nonzero(mask_col_far))

        # Boolean-Felder aus den Pixel-Anzahlen ableiten Bool Occupancy
        field_near = (sector_near >= self.min_sector_pixels).astype(np.uint8)
        field_far = (sector_far >= self.min_sector_pixels).astype(np.uint8)

        # --------------------------------------------------
        # ROS Messages
        # --------------------------------------------------
        # Globales Flag: TRUE, wenn mindestens ein nahes Feld aktiv
        obstacle_msg = Bool()
        obstacle_msg.data = bool(np.any(field_near))
        self.obstacle_pub.publish(obstacle_msg)

        # 10-Feld-Grid publizieren: [L..R nah, L..R weit]
        sector_msg = UInt8MultiArray()
        sector_msg.data = list(field_near) + list(field_far)
        self.sector_pub.publish(sector_msg)

        # Schwerpunkt-Logging nur noch zur Info (optional)
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

        # Debug-Image erzeugen
        debug_img = self.create_debug_image(
            roi_m, valid_mask, near_mask, far_mask,
            field_near, field_far
        )
        if debug_img is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_img_pub.publish(debug_msg)

    def create_debug_image(self, roi_m, valid_mask, near_mask, far_mask,
                           field_near, field_far):

        roi_h, roi_w = roi_m.shape[:2]
        # debug_img = np.zeros((roi_h, roi_w, 3), dtype=np.uint8)
        debug_img = cv2.normalize(roi_m, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR) 

        # Hindernis-Pixel einfärben
        debug_img[far_mask] = (0, 100, 0)
        debug_img[near_mask] = (0, 0, 200)

        # Horizontale Unterteilung über Variable z. B. num_rows
        num_rows = 5
        row_height = roi_h / num_rows
        for r in range(num_rows):
            y_start = int(r * row_height)
            y_end = int((r + 1) * row_height) - 1
            cv2.rectangle(debug_img, (0, y_start), (roi_w - 1, y_end), (100, 100, 100), 1)
        num_cols = 10
        col_width = roi_w / num_cols
        for r in range(num_cols):
            x_start = int(r * col_width)
            x_end = int((r + 1) * col_width) - 1
            cv2.rectangle(debug_img, (x_start, 0), (x_end, roi_h - 1), (100, 100, 100), 1)

        # Vertikale Unterteilung (dein vorhandener Code)
        col_width = roi_w / self.num_cols
        for c in range(self.num_cols):
            x_start = int(c * col_width)
            x_end = int((c + 1) * col_width) - 1
            color_rect = (0, 0, 255) if field_near[c] else (0, 255, 0) if field_far[c] else (80, 80, 80)
            cv2.rectangle(debug_img, (x_start, 0), (x_end, roi_h - 1), color_rect, 1)
            cv2.putText(debug_img, f"{c}", (x_start + 5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # # Legend scaled to image size (max 35% width, positioned top-left)
        # legend_width = min(int(roi_w * 0.35), 180)
        # legend_height = legend_width // 3
        # font_scale = min(roi_w / 640.0, 1.0) * 0.4  # Scale font with image size
        
        # cv2.rectangle(debug_img, (5, 5), (5 + legend_width, 5 + legend_height), (0, 0, 0), -1)
        # cv2.putText(debug_img, f"Rot: nah (<{self.near_distance_m}m)", (10, 20),
        #             cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), 1)
        # cv2.putText(debug_img, f"Gruen: weit (<{self.far_distance_m}m)", (10, int(20 + legend_height * 0.36)),
        #             cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), 1)

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
