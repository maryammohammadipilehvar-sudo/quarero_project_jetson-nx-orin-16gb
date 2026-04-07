"""Visualization node for obstacle detection.

Creates a top-down view image showing robot footprint, obstacles (from OccupancyGrid),
and sector analysis.
"""

import math
import threading
import traceback
from typing import List, Tuple, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

from .sector_analysis import SectorAnalyzer, SectorConfigLoader


class ObstacleVizNode(Node):
    """Visualization node for obstacle detection.
    
    Creates a top-down view image showing robot footprint, obstacles (from OccupancyGrid),
    and sector analysis.
    """

    def __init__(self) -> None:
        super().__init__('obstacle_viz')

        # Robot footprint (in base frame)
        self.declare_parameter('robot.footprint.width', 0.8)
        self.declare_parameter('robot.footprint.front', 0.2)
        self.declare_parameter('robot.footprint.rear', 0.9)

        # Topics
        self.declare_parameter('topics.obstacles_input', '/obstacles/lidar')

        # Visualization parameters
        self.declare_parameter('viz.range_m', 2.0)
        self.declare_parameter('viz.resolution_m_per_px', 0.02)
        self.declare_parameter('viz.upsample_factor', 2)
        self.declare_parameter('viz.image_topic', '/obstacles/image')
        
        # Sector analysis parameters
        self.declare_parameter('sector_analysis.max_range', 3.0)
        self.declare_parameter('sector_analysis.config_path', '')  # Path to sector_config.yaml, empty = auto-detect

        try:
            self._load_parameters()
        except Exception as e:
            self.get_logger().error(f"Failed to load parameters: {e}\n{traceback.format_exc()}")
            raise

        # Initialize sector analyzer
        # sectors_config must be set in _load_parameters()
        if not hasattr(self, 'sectors_config') or not self.sectors_config or len(self.sectors_config) == 0:
            self.get_logger().error("No sectors configured! Please define sectors in config file.")
            raise ValueError("No sectors configured")
        
        self._sector_analyzer = SectorAnalyzer(
            sectors_config=self.sectors_config,
            max_range=self.sector_max_range
        )

        self.output_size = self.img_size * self.upsample_factor
        self.get_logger().info(
            f"Obstacle viz node started. Range=±{self.viz_range_m} m, "
            f"base resolution={self.viz_res:.3f} m/px, base image={self.img_size}x{self.img_size}px, "
            f"upsampled output={self.output_size}x{self.output_size}px"
        )

        # Subscribe to obstacle grid
        self.obstacles_sub = self.create_subscription(
            OccupancyGrid,
            self.obstacles_topic,
            self.obstacle_callback,
            10,
        )
        
        # Store latest data for visualization
        self.current_occupancy_grid: Optional[OccupancyGrid] = None
        self.obstacles_lock = threading.Lock()

        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.viz_timer = self.create_timer(0.5, self._update_visualization)

    def _load_parameters(self) -> None:
        try:
            # Robot footprint
            self.footprint_width = float(self.get_parameter('robot.footprint.width').value)
            self.footprint_front = float(self.get_parameter('robot.footprint.front').value)
            self.footprint_rear = float(self.get_parameter('robot.footprint.rear').value)

            # Topics
            self.obstacles_topic = str(self.get_parameter('topics.obstacles_input').value)

            # Viz
            self.viz_range_m = float(self.get_parameter('viz.range_m').value)
            self.viz_res = float(self.get_parameter('viz.resolution_m_per_px').value)
            self.upsample_factor = int(self.get_parameter('viz.upsample_factor').value)
            self.image_topic = str(self.get_parameter('viz.image_topic').value)
            
            # Sector analysis parameters
            self.sector_max_range = float(self.get_parameter('sector_analysis.max_range').value)
            
            # Load sectors from config file
            config_path = str(self.get_parameter('sector_analysis.config_path').value)
            if not config_path:
                config_path = None  # Use auto-detection
            
            try:
                self.sectors_config = SectorConfigLoader.load_sectors(config_path)
                self.get_logger().info(f"Loaded {len(self.sectors_config)} sectors from config")
            except Exception as e:
                self.get_logger().error(f"Failed to load sectors from config: {e}\n{traceback.format_exc()}")
                raise
        except Exception as e:
            self.get_logger().error(f"Error loading parameter: {e}\n{traceback.format_exc()}")
            raise

        # Validate parameters
        if self.viz_res <= 0:
            self.get_logger().error(f"Invalid viz.resolution_m_per_px: {self.viz_res}. Must be > 0. Using default 0.02")
            self.viz_res = 0.02
        if self.viz_range_m <= 0:
            self.get_logger().error(f"Invalid viz.range_m: {self.viz_range_m}. Must be > 0. Using default 2.0")
            self.viz_range_m = 2.0
        if self.upsample_factor < 1:
            self.get_logger().error(f"Invalid viz.upsample_factor: {self.upsample_factor}. Must be >= 1. Using default 2")
            self.upsample_factor = 2

        # Compute image size; we want 2*range on each axis
        extent_m = 2.0 * self.viz_range_m
        size_px = int(round(extent_m / self.viz_res))
        # Ensure odd size so lidar is exactly at the middle pixel
        if size_px % 2 == 0:
            size_px += 1
        # Ensure minimum size
        if size_px < 1:
            size_px = 1
        self.img_size = size_px

    # ------------------------------------------------------------------ #
    # OccupancyGrid callback
    # ------------------------------------------------------------------ #
    def obstacle_callback(self, msg: OccupancyGrid) -> None:
        """Store received OccupancyGrid message.
        
        Args:
            msg: OccupancyGrid message containing obstacle data.
        """
        try:
            with self.obstacles_lock:
                self.current_occupancy_grid = msg
        except Exception as e:
            self.get_logger().error(f"Error processing OccupancyGrid: {e}\n{traceback.format_exc()}")
    
    def _update_visualization(self) -> None:
        """Timer callback to update and publish visualization image."""
        try:
            # Get current occupancy grid
            with self.obstacles_lock:
                occupancy_grid = self.current_occupancy_grid
            
            # Initialize black BGR image
            img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
            
            # Compute helper function to map (x, y) in base frame to image pixels
            center = self.img_size // 2
            
            def to_pixel(x: float, y: float) -> Tuple[int, int]:
                """Map base frame (x,y) [m] to image (u,v) [px].
                
                Robot frame: x = forward (positive), y = left (positive)
                Image frame: u = right (positive), v = down (positive)
                Top-down view: forward (x+) at top (small v), left (y+) at left (small u)
                """
                # u (image x, right): y positive (left) -> u small (left), y negative (right) -> u large (right)
                u = int(round(center - y / self.viz_res))
                # v (image y, down): x positive (forward) -> v small (top), x negative (backward) -> v large (bottom)
                v = int(round(center - x / self.viz_res))
                return u, v

            # ====================================================================
            # SECTOR VISUALIZATION (RECTANGULAR SECTORS)
            # ====================================================================
            # Analyze sectors directly from occupancy grid using SectorAnalyzer
            if occupancy_grid is not None:
                try:
                    sectors = self._sector_analyzer.analyze(occupancy_grid)
                    
                    # Create overlay for sectors
                    sectors_overlay = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
                    
                    # Draw each configured sector
                    for sector_info in sectors:
                        sector_id = sector_info.sector_id
                        
                        # Get sector definition from analyzer
                        sector_def = None
                        for sd in self._sector_analyzer.sectors:
                            if sd.sector_id == sector_id:
                                sector_def = sd
                                break
                        
                        if sector_def is None:
                            continue
                        
                        # Determine fill color based on blocked status and sector_type
                        sector_type = sector_def.sector_type.upper()
                        if not sector_info.blocked:
                            fill_color = (0, 200, 0)  # Green (BGR), slightly dimmed
                        else:  # BLOCKED
                            if sector_type == 'SLOW':
                                fill_color = (0, 255, 255)  # Yellow (BGR)
                            elif sector_type == 'STOP':
                                fill_color = (0, 0, 255)  # Red (BGR)
                            else:
                                fill_color = (0, 0, 255)  # Red (BGR) as default for BLOCKED
                        
                        # Border color is black for all sectors (unified)
                        border_color = (0, 0, 0)  # Black (BGR)
                        border_thickness = 2
                        
                        # Get sector boundaries from config
                        x_min = sector_def.x_min
                        x_max = sector_def.x_max
                        y_min = sector_def.y_min
                        y_max = sector_def.y_max
                        
                        # Convert to pixel coordinates
                        u_min, v_max = to_pixel(x_min, y_min)
                        u_max, v_min = to_pixel(x_max, y_max)
                        
                        # Ensure correct ordering
                        if u_min > u_max:
                            u_min, u_max = u_max, u_min
                        if v_min > v_max:
                            v_min, v_max = v_max, v_min
                        
                        # Clamp to image bounds
                        u_min_clamped = max(0, min(self.img_size - 1, u_min))
                        u_max_clamped = max(0, min(self.img_size - 1, u_max))
                        v_min_clamped = max(0, min(self.img_size - 1, v_min))
                        v_max_clamped = max(0, min(self.img_size - 1, v_max))
                        
                        # Draw filled rectangle for sector
                        if (u_min_clamped < u_max_clamped and v_min_clamped < v_max_clamped):
                            # Draw filled rectangle with classification color
                            cv2.rectangle(
                                sectors_overlay,
                                (u_min_clamped, v_min_clamped),
                                (u_max_clamped, v_max_clamped),
                                fill_color,
                                thickness=-1  # Filled
                            )
                            # Draw border with black color (unified for all sectors)
                            cv2.rectangle(
                                sectors_overlay,
                                (u_min_clamped, v_min_clamped),
                                (u_max_clamped, v_max_clamped),
                                border_color,
                                thickness=border_thickness
                            )
                    
                    # Blend the sectors overlay with main image
                    alpha = 0.2
                    cv2.addWeighted(sectors_overlay, alpha, img, 1.0 - alpha, 0, img)
                    
                except Exception as e:
                    self.get_logger().warn(f"Error drawing sectors: {e}\n{traceback.format_exc()}")

            # Draw robot footprint (gray rectangle)
            robot_min_x = -self.footprint_rear
            robot_max_x = self.footprint_front
            half_w = 0.5 * self.footprint_width
            robot_min_y = -half_w
            robot_max_y = half_w
            
            u_min, v_max = to_pixel(robot_min_x, robot_min_y)
            u_max, v_min = to_pixel(robot_max_x, robot_max_y)
            u_min = max(0, min(self.img_size - 1, u_min))
            u_max = max(0, min(self.img_size - 1, u_max))
            v_min = max(0, min(self.img_size - 1, v_min))
            v_max = max(0, min(self.img_size - 1, v_max))
            cv2.rectangle(img, (u_min, v_min), (u_max, v_max), (128, 128, 128), thickness=-1)
            

            # Draw obstacles from OccupancyGrid
            if occupancy_grid is not None:
                grid_width = occupancy_grid.info.width
                grid_height = occupancy_grid.info.height
                grid_resolution = occupancy_grid.info.resolution
                origin_x = occupancy_grid.info.origin.position.x
                origin_y = occupancy_grid.info.origin.position.y
                
                # Draw each occupied cell as a red rectangle
                for y_idx in range(grid_height):
                    for x_idx in range(grid_width):
                        idx = y_idx * grid_width + x_idx
                        if idx < len(occupancy_grid.data) and occupancy_grid.data[idx] == 100:
                            # Calculate cell bounds in world coordinates
                            cell_min_x = origin_x + x_idx * grid_resolution
                            cell_max_x = origin_x + (x_idx + 1) * grid_resolution
                            cell_min_y = origin_y + y_idx * grid_resolution
                            cell_max_y = origin_y + (y_idx + 1) * grid_resolution
                            
                            # Check if cell is within visualization range
                            cell_center_x = (cell_min_x + cell_max_x) / 2.0
                            cell_center_y = (cell_min_y + cell_max_y) / 2.0
                            cell_dist = math.hypot(cell_center_x, cell_center_y)
                            
                            if cell_dist > self.viz_range_m:
                                continue
                            
                            # Convert to pixel coordinates
                            u_min, v_max = to_pixel(cell_min_x, cell_min_y)
                            u_max, v_min = to_pixel(cell_max_x, cell_max_y)
                            
                            # Ensure u_min < u_max and v_min < v_max
                            if u_min > u_max:
                                u_min, u_max = u_max, u_min
                            if v_min > v_max:
                                v_min, v_max = v_max, v_min
                            
                            # Clamp to image bounds
                            u_min_clamped = max(0, min(self.img_size - 1, u_min))
                            u_max_clamped = max(0, min(self.img_size - 1, u_max))
                            v_min_clamped = max(0, min(self.img_size - 1, v_min))
                            v_max_clamped = max(0, min(self.img_size - 1, v_max))
                            
                            # Only draw if cell is at least partially visible and has valid size
                            if (u_min_clamped < u_max_clamped and v_min_clamped < v_max_clamped and
                                (u_max_clamped - u_min_clamped) >= 1 and (v_max_clamped - v_min_clamped) >= 1):
                                # Draw filled rectangle with dark red fill
                                cv2.rectangle(img, (u_min_clamped, v_min_clamped), (u_max_clamped, v_max_clamped), (0, 0, 128), thickness=-1)
                                cv2.rectangle(img, (u_min_clamped, v_min_clamped), (u_max_clamped, v_max_clamped), (0, 0, 255), thickness=1)

            # Draw LiDAR position as a small green square at the center
            size = 2
            u_min = max(0, center - size)
            u_max = min(self.img_size - 1, center + size)
            v_min = max(0, center - size)
            v_max = min(self.img_size - 1, center + size)
            cv2.rectangle(img, (u_min, v_min), (u_max, v_max), (0, 255, 0), thickness=-1)
            
            # Upsample image for smoother display
            if self.upsample_factor > 1:
                img = cv2.resize(
                    img,
                    (self.output_size, self.output_size),
                    interpolation=cv2.INTER_LINEAR
                )
            
            # Publish image
            now = self.get_clock().now()
            self._publish_image(img, now.to_msg(), "base_link")
            
        except Exception as e:
            self.get_logger().error(f"Error in visualization update: {e}\n{traceback.format_exc()}")
    

    def _publish_image(self, img: np.ndarray, stamp, frame_id: str) -> None:
        """Publish OpenCV image as sensor_msgs/Image.
        
        Args:
            img: OpenCV image in BGR format.
            stamp: ROS2 time stamp for message header.
            frame_id: Frame ID for message header.
        """
        try:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.height, msg.width = img.shape[0], img.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = img.tobytes()
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}\n{traceback.format_exc()}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

