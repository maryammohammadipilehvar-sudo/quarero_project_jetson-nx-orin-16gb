import array
import math
import struct
from collections import defaultdict
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class LivoxObstacleNode(Node):
    """ROS2 node for detecting obstacles from Livox LiDAR using grid-based approach."""

    def __init__(self) -> None:
        super().__init__('livox_mid360_obstacle')

        # Robot footprint (in base frame)
        self.declare_parameter('robot.footprint.width', 0.8)
        self.declare_parameter('robot.footprint.front', 0.2)
        self.declare_parameter('robot.footprint.rear', 0.9)

        # LiDAR pose in base frame
        self.declare_parameter('lidar.offset.x', 0.0)
        self.declare_parameter('lidar.offset.y', 0.0)
        self.declare_parameter('lidar.offset.z', 0.6)
        self.declare_parameter('lidar.flip_z', True)  # Flip Z axis (upside down lidar)

        # Obstacle detection parameters
        self.declare_parameter('obstacle_detection.max_distance', 5.0)
        self.declare_parameter('obstacle_detection.min_height', 0.08)
        self.declare_parameter('obstacle_detection.max_height', 0.6)
        self.declare_parameter('obstacle_detection.slope_start_distance', 1.0)  # [m] distance where slope compensation begins
        self.declare_parameter('obstacle_detection.slope_compensation', 0.3)    # [m] max height offset at max_distance
        # Grid-based obstacle detection parameters
        self.declare_parameter('obstacle_detection.grid_cell_size', 0.1)  # [m] size of each grid cell
        self.declare_parameter('obstacle_detection.min_points_per_cell', 2)  # minimum points in a cell to mark it as obstacle

        # Topics
        self.declare_parameter('topics.lidar_input', '/livox/points')
        self.declare_parameter('topics.obstacles_output', '/obstacles/lidar')
        self.declare_parameter('topics.filtered_points_output', '/obstacles/filtered_points')
        
        # Frame configuration
        self.declare_parameter('frame_id', 'base_footprint')  # Frame for OccupancyGrid (base_footprint for 2D nav, base_link for 3D)
        
        # Rate limiting (max publish rate in Hz, 0 = no limit)
        self.declare_parameter('publish_rate', 0.0)
        
        # Optional: enable/disable filtered points publishing (for performance)
        self.declare_parameter('publish_filtered_points', True)  # Set to False to disable filtered points publishing

        self._load_parameters()

        self.get_logger().info(
            f"Livox obstacle node started: lidar={self.lidar_topic}, "
            f"obstacles={self.obstacles_topic}, filtered_points={self.filtered_points_topic}, "
            f"frame_id={self.frame_id}"
        )

        # Subscribers / publishers
        self.lidar_sub = self.create_subscription(
            CustomMsg,
            self.lidar_topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )

        # Publisher for obstacle grid (OccupancyGrid - much more efficient than individual PolygonStamped)
        # Use RELIABLE QoS to match nav2_navigation subscriber requirements
        # Keep VOLATILE durability and KEEP_LAST history for sensor-like behavior
        # Reduced depth to 5 to prevent queue overflow in RVIZ
        occupancy_grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Required for nav2_navigation compatibility
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # Reduced from 10 to prevent RVIZ queue overflow
            durability=DurabilityPolicy.VOLATILE
        )
        self.obstacles_pub = self.create_publisher(
            OccupancyGrid,
            self.obstacles_topic,
            occupancy_grid_qos,
        )
        
        # Publisher for filtered points (used in obstacle detection)
        self.filtered_points_pub = self.create_publisher(
            PointCloud2,
            self.filtered_points_topic,
            10,
        )

    def _load_parameters(self) -> None:
        # Robot footprint
        self.footprint_width = float(self.get_parameter('robot.footprint.width').value)
        self.footprint_front = float(self.get_parameter('robot.footprint.front').value)
        self.footprint_rear = float(self.get_parameter('robot.footprint.rear').value)

        # Lidar offset
        self.lidar_offset_x = float(self.get_parameter('lidar.offset.x').value)
        self.lidar_offset_y = float(self.get_parameter('lidar.offset.y').value)
        self.lidar_offset_z = float(self.get_parameter('lidar.offset.z').value)
        self.lidar_flip_z = bool(self.get_parameter('lidar.flip_z').value)

        # Obstacle detection
        self.max_distance = float(self.get_parameter('obstacle_detection.max_distance').value)
        self.min_height = float(self.get_parameter('obstacle_detection.min_height').value)
        self.max_height = float(self.get_parameter('obstacle_detection.max_height').value)
        self.slope_start_distance = float(self.get_parameter('obstacle_detection.slope_start_distance').value)
        self.slope_compensation = float(self.get_parameter('obstacle_detection.slope_compensation').value)
        
        # Pre-compute squared distance for faster comparison (avoid sqrt)
        self.max_distance_sq = self.max_distance * self.max_distance
        
        # Pre-compute slope range for efficiency
        self.slope_range = self.max_distance - self.slope_start_distance
        
        # Grid-based parameters
        self.grid_cell_size = float(self.get_parameter('obstacle_detection.grid_cell_size').value)
        self.min_points_per_cell = int(self.get_parameter('obstacle_detection.min_points_per_cell').value)
        
        # Pre-compute cell size inverse for faster division
        self.cell_size_inv = 1.0 / self.grid_cell_size
        
        # OPTIMIZATION: Pre-compute and cache grid dimensions (only change if max_distance or grid_cell_size change)
        self._compute_grid_dimensions()
        
        # Compute robot footprint bounds for filtering
        self.robot_min_x = -self.footprint_rear
        self.robot_max_x = self.footprint_front
        half_width = 0.5 * self.footprint_width
        self.robot_min_y = -half_width
        self.robot_max_y = half_width

        # Topics
        self.lidar_topic = str(self.get_parameter('topics.lidar_input').value)
        self.obstacles_topic = str(self.get_parameter('topics.obstacles_output').value)
        self.filtered_points_topic = str(self.get_parameter('topics.filtered_points_output').value)
        
        # Frame configuration
        self.frame_id = str(self.get_parameter('frame_id').value)
        
        # Rate limiting
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.last_publish_time = None
        if self.publish_rate > 0:
            self.min_publish_interval = 1.0 / self.publish_rate
            self.get_logger().info(f"Rate limiting enabled: max {self.publish_rate} Hz (min interval: {self.min_publish_interval:.3f}s)")
        else:
            self.min_publish_interval = 0.0
            self.get_logger().info("Rate limiting disabled")
        
        # Optional filtered points publishing
        self.publish_filtered_points = bool(self.get_parameter('publish_filtered_points').value)
        if not self.publish_filtered_points:
            self.get_logger().info("Filtered points publishing disabled (performance optimization)")

    def _compute_grid_dimensions(self) -> None:
        """Compute and cache grid dimensions based on max_distance and grid_cell_size."""
        # Grid covers from -max_distance to +max_distance in both x and y
        grid_extent = 2.0 * self.max_distance
        grid_width = int(math.ceil(grid_extent / self.grid_cell_size))
        grid_height = int(math.ceil(grid_extent / self.grid_cell_size))
        
        # Ensure odd dimensions so robot (0,0) is at center
        if grid_width % 2 == 0:
            grid_width += 1
        if grid_height % 2 == 0:
            grid_height += 1
        
        # Cache dimensions
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.grid_center_x = grid_width // 2
        self.grid_center_y = grid_height // 2
        self.grid_origin_x = -self.grid_center_x * self.grid_cell_size
        self.grid_origin_y = -self.grid_center_y * self.grid_cell_size

    def lidar_callback(self, msg: CustomMsg) -> None:
        """Process incoming Livox point cloud message.
        
        Args:
            msg: CustomMsg from Livox LiDAR containing point cloud data.
        """
        # Rate limiting: skip early if too soon since last publish (before processing)
        current_time = self.get_clock().now()
        if self.min_publish_interval > 0 and self.last_publish_time is not None:
            time_since_last = (current_time - self.last_publish_time).nanoseconds / 1e9
            if time_since_last < self.min_publish_interval:
                return  # Skip this message to limit publish rate
        
        # Convert points to numpy array for vectorized processing
        if not msg.points:
            valid_points_array = None
        else:
            # Extract points as numpy array
            points_array = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
            
            # Apply coordinate transformation (vectorized)
            if self.lidar_flip_z:
                points_array[:, 1] = -points_array[:, 1]  # Flip Y
                points_array[:, 2] = -points_array[:, 2]  # Flip Z
            
            # Apply offset (vectorized)
            points_array[:, 0] += self.lidar_offset_x
            points_array[:, 1] += self.lidar_offset_y
            points_array[:, 2] += self.lidar_offset_z
            
            # Filter by max distance using squared distance (vectorized, avoids sqrt)
            x_sq = points_array[:, 0] * points_array[:, 0]
            y_sq = points_array[:, 1] * points_array[:, 1]
            dist_sq = x_sq + y_sq
            max_distance_mask = dist_sq <= self.max_distance_sq
            
            # Calculate dynamic min_height with slope compensation
            # Phase 1 (d <= slope_start_distance): min_height constant
            # Phase 2 (d > slope_start_distance): min_height increases linearly
            if self.slope_compensation > 0.0 and self.slope_range > 0.0:
                horizontal_dist = np.sqrt(dist_sq)
                slope_factor = np.clip(
                    (horizontal_dist - self.slope_start_distance) / self.slope_range,
                    0.0,
                    1.0
                )
                dynamic_min_height = self.min_height + slope_factor * self.slope_compensation
            else:
                dynamic_min_height = self.min_height
            
            # Filter by height with dynamic threshold (vectorized)
            height_mask = (points_array[:, 2] >= dynamic_min_height) & (points_array[:, 2] <= self.max_height)
            
            # Filter out points inside the robot footprint (rectangular exclusion zone)
            inside_footprint = (
                (points_array[:, 0] >= self.robot_min_x) & 
                (points_array[:, 0] <= self.robot_max_x) & 
                (points_array[:, 1] >= self.robot_min_y) & 
                (points_array[:, 1] <= self.robot_max_y)
            )
            footprint_mask = ~inside_footprint  # Keep points OUTSIDE the footprint
            
            # Combine masks
            valid_mask = height_mask & max_distance_mask & footprint_mask
            
            # Extract valid points (keep as numpy array for optimization)
            if np.any(valid_mask):
                valid_points_array = points_array[valid_mask]
            else:
                valid_points_array = None

        scan_timestamp = current_time
        
        if valid_points_array is None or len(valid_points_array) == 0:
            # Publish empty grid even if no points to keep RVIZ updated
            empty_grid = self._create_occupancy_grid([], scan_timestamp)
            self.obstacles_pub.publish(empty_grid)
            self.last_publish_time = current_time
            self.get_logger().debug(f"Published empty OccupancyGrid (no points): {empty_grid.info.width}x{empty_grid.info.height}, frame={empty_grid.header.frame_id}")
            return

        # OPTIMIZATION: Pass numpy array directly instead of converting to list
        occupied_cells = self._build_occupancy_grid(valid_points_array)
        
        # Optionally publish filtered points (can be disabled for performance)
        if self.publish_filtered_points:
            # Convert to list only for filtered points publishing (if needed)
            base_points = [(float(x), float(y), float(z)) for x, y, z in valid_points_array]
            self._publish_filtered_points(base_points, scan_timestamp)

        if not occupied_cells:
            empty_grid = self._create_occupancy_grid([], scan_timestamp)
            self.obstacles_pub.publish(empty_grid)
            self.last_publish_time = current_time
            self.get_logger().debug(f"Published empty OccupancyGrid: {empty_grid.info.width}x{empty_grid.info.height}, frame={empty_grid.header.frame_id}")
            return

        occupancy_grid = self._create_occupancy_grid(occupied_cells, scan_timestamp)
        self.obstacles_pub.publish(occupancy_grid)
        self.last_publish_time = current_time
        self.get_logger().debug(
            f"Published OccupancyGrid: {occupancy_grid.info.width}x{occupancy_grid.info.height}, "
            f"frame={occupancy_grid.header.frame_id}, occupied_cells={len(occupied_cells)}, "
            f"subscribers={self.obstacles_pub.get_subscription_count()}"
        )

    def _build_occupancy_grid(
        self, points: np.ndarray
    ) -> List[Tuple[int, int]]:
        """Build an occupancy grid from points (optimized version).
        
        Args:
            points: NumPy array of shape (N, 3) with (x, y, z) points in base frame.
            
        Returns:
            List of occupied cell coordinates as (grid_x, grid_y) tuples.
        """
        if points is None or len(points) == 0:
            return []
        
        # OPTIMIZATION: Vectorized grid coordinate conversion
        # Use np.floor for negative values to get correct floor behavior
        grid_coords = np.floor(points[:, :2] * self.cell_size_inv).astype(np.int32)
        
        # OPTIMIZATION: Use numpy's unique to count points per cell (much faster than defaultdict)
        # Get unique coordinates and their counts
        unique_coords, counts = np.unique(grid_coords, axis=0, return_counts=True)
        
        # Filter cells that have enough points
        occupied_mask = counts >= self.min_points_per_cell
        occupied_cells = [tuple(coord) for coord in unique_coords[occupied_mask]]
        
        return occupied_cells

    def _create_occupancy_grid(
        self, 
        occupied_cells: List[Tuple[int, int]], 
        timestamp
    ) -> OccupancyGrid:
        """Create an OccupancyGrid message from occupied grid cells (optimized version).
        
        Args:
            occupied_cells: List of (grid_x, grid_y) tuples.
            timestamp: ROS2 time for the message header.
            
        Returns:
            OccupancyGrid message with occupied cells marked.
        """
        # OPTIMIZATION: Use cached grid dimensions
        grid_width = self.grid_width
        grid_height = self.grid_height
        center_x = self.grid_center_x
        center_y = self.grid_center_y
        
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.frame_id
        
        # Grid metadata
        msg.info.resolution = self.grid_cell_size
        msg.info.width = grid_width
        msg.info.height = grid_height
        
        # Set origin (using cached values)
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # No rotation
        
        # OPTIMIZATION: Use NumPy array instead of array.array + tolist()
        # Initialize grid data as zeros (free space)
        grid_data = np.zeros((grid_height, grid_width), dtype=np.int8)
        
        # OPTIMIZATION: Vectorized cell marking
        if occupied_cells:
            # Convert grid coordinates to OccupancyGrid indices (vectorized)
            occupied_coords = np.array(occupied_cells, dtype=np.int32)
            occ_x = center_x + occupied_coords[:, 0]
            occ_y = center_y + occupied_coords[:, 1]
            
            # Filter valid indices (within bounds)
            valid_mask = (occ_x >= 0) & (occ_x < grid_width) & (occ_y >= 0) & (occ_y < grid_height)
            valid_occ_x = occ_x[valid_mask]
            valid_occ_y = occ_y[valid_mask]
            
            # Mark occupied cells (vectorized)
            grid_data[valid_occ_y, valid_occ_x] = 100
        
        # Convert to list (required by ROS2 message)
        msg.data = grid_data.flatten().tolist()
        return msg

    def _publish_filtered_points(self, base_points: List[Tuple[float, float, float]], time) -> None:
        """Publish filtered points as PointCloud2 for visualization.
        
        Args:
            base_points: List of (x, y, z) points in base frame.
            time: ROS2 time for the message header.
        """
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(base_points)
        msg.is_dense = True
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        
        # Define fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack points as binary data (x, y, z for each point)
        # Use numpy for faster array operations
        if base_points:
            # Convert to numpy array and flatten for faster packing
            points_np = np.array(base_points, dtype=np.float32).flatten()
            msg.data = points_np.tobytes()
        else:
            msg.data = b''
        
        self.filtered_points_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


