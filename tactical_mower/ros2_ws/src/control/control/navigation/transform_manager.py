"""Transform management for ROS TF frames and GPS coordinate transformations.

This module provides a unified interface for managing robot frame transformations
and GPS coordinate conversions. It uses CoordinateTransformer internally for
GPS → ENU conversions and handles ROS TF frame transformations.
"""

from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped, Pose, Point
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped
import tf2_ros
import math

from .coordinate_transformer import CoordinateTransformer


class TransformManager:
    """Manages ROS TF frame transformations and GPS coordinate conversions.
    
    This class provides a unified interface for:
    - Publishing static TF transforms (vrtk_link → base_link → base_footprint, lidar_link)
    - Converting GPS coordinates to map frame coordinates
    - Querying robot pose via TF (preferred method)
    - Transforming poses between frames using TF
    
    Frame Conventions:
    - **map**: Global fixed frame (aligned with ENU0 from Fixposition Driver)
    - **odom**: Odometry frame (drifts over time, but continuous)
    - **vrtk_link**: VRTK (GNSS/IMU) sensor frame (from Fixposition Driver)
    - **base_link**: Robot base center frame (physical center of robot)
    - **base_footprint**: Robot base footprint frame (projection of base_link onto ground, z=0, for 2D navigation)
    - **lidar_link**: LiDAR sensor frame (Livox MID-360)
    
    TF Tree Structure:
    ```
    map → odom → vrtk_link → base_link
                              ├── base_footprint
                              └── lidar_link
    ```
    
    Best Practices:
    - Use TF-based pose queries (get_robot_pose_tf()) instead of GPS-based calculations when possible
    - Static transforms are published once (ROS2 transient_local QoS ensures late-joining nodes receive them)
    - TF lookups have configurable timeout (default: 1.0s)
    - GPS-based methods provide fallback if TF is unavailable
    
    Attributes:
        coord_transformer: Internal CoordinateTransformer instance
        node: Optional ROS2 node for TF publishing
        static_broadcaster: Optional static TF broadcaster
    """
    
    def __init__(self, node: Optional[Node] = None):
        """Initialize TransformManager.
        
        Args:
            node: Optional ROS2 node. If provided, enables TF publishing.
                 If None, TransformManager works in non-ROS mode (GPS conversions only).
        """
        self._coord_transformer = CoordinateTransformer()
        self._node = node
        self._static_broadcaster: Optional[StaticTransformBroadcaster] = None
        self._static_transform_timer = None  # Timer for periodic republishing of static transforms
        
        # TF Buffer and Listener for pose queries (initialized if node is provided)
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        self._tf_timeout: float = 1.0  # Default timeout for TF lookups (seconds)
        
        # Static transform republish interval (seconds)
        # Republishing ensures timestamps stay current, preventing "stale transform" issues
        self._static_transform_republish_interval = 2.0  # Republish every 2 seconds (0.5 Hz)
        
        # Frame offsets (configured via parameters or set methods)
        self._vrtk_to_base_offset = (0.0, 0.0, 0.0)  # (x, y, z) in meters
        self._vrtk_to_base_rotation = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w) quaternion
        self._base_to_base_footprint_offset = (0.0, 0.0, 0.0)  # (x, y, z) in meters (z should be negative to project base_link to ground)
        self._base_to_base_footprint_rotation = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w) quaternion (typically Identity)
        # Legacy: kept for backward compatibility
        self._vrtk_to_base_footprint_offset = (0.0, 0.0, 0.0)  # Deprecated: use base_to_base_footprint instead
        self._vrtk_to_base_footprint_rotation = (0.0, 0.0, 0.0, 1.0)  # Deprecated
        self._base_to_lidar_offset = (0.0, 0.0, 0.0)  # (x, y, z) in meters
        self._base_to_lidar_rotation = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w) quaternion
        
        # Map origin (FP_ENU0 from Fixposition Driver) in GPS coordinates
        # NOTE: Map Origin != charge position! Origin comes from FP_ENU0, not from settings.
        self._map_origin_gps: Optional[Tuple[float, float, float]] = None
        
        if node is not None:
            self._static_broadcaster = StaticTransformBroadcaster(node)
            # Initialize TF Buffer and Listener for pose queries
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, node)
            # Note: Static transforms are published via publish_static_transforms()
            # ROS2 transient_local QoS on /tf_static ensures late-joining nodes receive them
            # We republish them periodically to keep timestamps current (prevents "stale transform" issues)
    
    def set_map_origin(self, lat: float, lon: float, alt: float = 0.0):
        """Set the map origin (FP_ENU0) in GPS coordinates.
        
        This origin is used for GPS → Map transformations.
        The CoordinateTransformer's origin is also set to this value.
        
        NOTE: Map Origin != charge position! Origin should come from FP_ENU0 (Fixposition Driver),
        typically set via auto_set_map_origin_from_tf_and_gps(), not from settings.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        self._map_origin_gps = (lat, lon, alt)
        self._coord_transformer.set_origin(lat, lon, alt)
    
    def compute_map_origin_from_tf_and_gps(
        self,
        robot_pose_map: PoseStamped,
        robot_gps: Tuple[float, float, float]
    ) -> Optional[Tuple[float, float, float]]:
        """Compute map origin dynamically from TF robot pose and GPS position.
        
        This method calculates the GPS origin (FP_ENU0) by using the current
        robot position in map frame (from TF) and the current GPS position.
        This ensures the origin matches the actual FP_ENU0 used by the Fixposition Driver.
        
        Formula: Origin_GPS = Robot_GPS - (Robot_Map_Position in ENU)
        
        Args:
            robot_pose_map: Current robot pose in map frame (from TF)
            robot_gps: Current robot GPS position as (latitude, longitude, altitude)
            
        Returns:
            Tuple of (origin_lat, origin_lon, origin_alt) or None if calculation fails
        """
        try:
            # Get map position (ENU coordinates relative to origin)
            map_x = robot_pose_map.pose.position.x  # East
            map_y = robot_pose_map.pose.position.y  # North
            map_z = robot_pose_map.pose.position.z  # Up
            
            # Current GPS position
            gps_lat, gps_lon, gps_alt = robot_gps
            
            # Convert to radians
            gps_lat_rad = math.radians(gps_lat)
            gps_lon_rad = math.radians(gps_lon)
            
            # Inverse ENU transformation to get origin
            # ENU formula: x = R * dlon * cos(lat_origin), y = R * dlat
            # Inverse: dlat = y / R, dlon = x / (R * cos(lat_origin))
            # But we need lat_origin, so we approximate using current lat
            EARTH_RADIUS = 6378137.0
            
            # Calculate latitude difference (North component)
            dlat_rad = map_y / EARTH_RADIUS
            origin_lat_rad = gps_lat_rad - dlat_rad
            
            # Calculate longitude difference (East component)
            # Use the calculated origin_lat for better accuracy
            dlon_rad = map_x / (EARTH_RADIUS * math.cos(origin_lat_rad))
            origin_lon_rad = gps_lon_rad - dlon_rad
            
            # Convert back to degrees
            origin_lat = math.degrees(origin_lat_rad)
            origin_lon = math.degrees(origin_lon_rad)
            
            # Altitude difference
            origin_alt = gps_alt - map_z
            
            return (origin_lat, origin_lon, origin_alt)
            
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().error(
                    f"Failed to compute map origin from TF and GPS: {e}"
                )
            return None
    
    def auto_set_map_origin_from_tf_and_gps(
        self,
        robot_gps: Tuple[float, float, float],
        timeout: float = 2.0
    ) -> bool:
        """Automatically set map origin from TF robot pose and GPS position.
        
        This method gets the current robot pose from TF and calculates the
        map origin dynamically. This ensures the origin matches FP_ENU0.
        
        Args:
            robot_gps: Current robot GPS position as (latitude, longitude, altitude)
            timeout: Timeout for TF lookup in seconds
            
        Returns:
            True if origin was set successfully, False otherwise
        """
        if self._tf_buffer is None or self._node is None:
            if self._node is not None:
                self._node.get_logger().warn(
                    "Cannot auto-set map origin: TF buffer not available"
                )
            return False
        
        try:
            # Get robot pose from TF
            robot_pose_map = self.get_robot_pose_tf(
                robot_base_frame="base_footprint",
                global_frame="map",
                timeout=timeout
            )
            
            if robot_pose_map is None:
                if self._node is not None:
                    self._node.get_logger().warn(
                        "Cannot auto-set map origin: Robot pose from TF not available"
                    )
                return False
            
            # Calculate origin
            origin = self.compute_map_origin_from_tf_and_gps(robot_pose_map, robot_gps)
            
            if origin is None:
                return False
            
            # Set the calculated origin
            self.set_map_origin(origin[0], origin[1], origin[2])
            
            if self._node is not None:
                self._node.get_logger().info(
                    f"Auto-set map origin from TF and GPS: "
                    f"({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.2f})"
                )
            
            return True
            
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().error(
                    f"Failed to auto-set map origin from TF and GPS: {e}"
                )
            return False
    
    def get_map_origin(self) -> Optional[Tuple[float, float, float]]:
        """Get the current map origin in GPS coordinates.
        
        Returns:
            Tuple of (latitude, longitude, altitude) or None if not set
        """
        return self._map_origin_gps
    
    def set_vrtk_to_base_transform(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0
    ):
        """Set the transform from vrtk_link to base_link.
        
        Args:
            x, y, z: Translation in meters
            qx, qy, qz, qw: Rotation quaternion
        """
        self._vrtk_to_base_offset = (x, y, z)
        self._vrtk_to_base_rotation = (qx, qy, qz, qw)
    
    def set_base_to_base_footprint_transform(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0
    ):
        """Set the transform from base_link to base_footprint.
        
        base_footprint is the projection of base_link onto the ground (z=0) for 2D navigation.
        Typically, this is just a translation in z-direction (negative z to project to ground).
        
        Args:
            x, y, z: Translation in meters (z should be negative to project base_link to ground, typically -base_link.z)
            qx, qy, qz, qw: Rotation quaternion (typically Identity: 0,0,0,1)
        """
        self._base_to_base_footprint_offset = (x, y, z)
        self._base_to_base_footprint_rotation = (qx, qy, qz, qw)
    
    def set_vrtk_to_base_footprint_transform(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0
    ):
        """Set the transform from vrtk_link to base_footprint (DEPRECATED).
        
        DEPRECATED: This method is kept for backward compatibility.
        Use set_base_to_base_footprint_transform() instead.
        base_footprint is now a child of base_link, not vrtk_link.
        
        This method automatically converts the vrtk_link → base_footprint transform
        to base_link → base_footprint by subtracting the vrtk_link → base_link offset.
        
        Args:
            x, y, z: Translation in meters (z should be 0.0 for ground level)
            qx, qy, qz, qw: Rotation quaternion
        """
        # Store for backward compatibility
        self._vrtk_to_base_footprint_offset = (x, y, z)
        self._vrtk_to_base_footprint_rotation = (qx, qy, qz, qw)
        
        # Auto-convert to base_link → base_footprint
        # base_footprint = vrtk_to_base_footprint - vrtk_to_base
        base_to_footprint_x = x - self._vrtk_to_base_offset[0]
        base_to_footprint_y = y - self._vrtk_to_base_offset[1]
        base_to_footprint_z = z - self._vrtk_to_base_offset[2]  # Should be -base_link.z
        
        # For rotation: base_footprint_rot = vrtk_to_base_footprint_rot * inv(vrtk_to_base_rot)
        # But typically both are Identity, so we keep Identity
        self._base_to_base_footprint_offset = (base_to_footprint_x, base_to_footprint_y, base_to_footprint_z)
        self._base_to_base_footprint_rotation = (qx, qy, qz, qw)
    
    def set_base_to_lidar_transform(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0
    ):
        """Set the transform from base_link to lidar_link.
        
        Args:
            x, y, z: Translation in meters
            qx, qy, qz, qw: Rotation quaternion
        """
        self._base_to_lidar_offset = (x, y, z)
        self._base_to_lidar_rotation = (qx, qy, qz, qw)
    
    def gps_to_map(self, lat: float, lon: float, alt: float = 0.0) -> Tuple[float, float, float]:
        """Convert GPS coordinates to map frame coordinates.
        
        Uses CoordinateTransformer for GPS → ENU conversion.
        Since map frame is aligned with ENU (via Fixposition Driver),
        the ENU coordinates are directly the map coordinates.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
            
        Returns:
            Tuple of (x, y, z) in map frame (meters)
            
        Raises:
            ValueError: If map origin is not set
        """
        if self._map_origin_gps is None:
            raise ValueError("Map origin not set. Call set_map_origin() first.")
        
        # GPS → ENU (via CoordinateTransformer)
        enu_x, enu_y, enu_z = self._coord_transformer.gps_to_enu(lat, lon, alt)
        
        # ENU coordinates are directly map coordinates
        # (Fixposition Driver publishes map → odom, where map is aligned with ENU0)
        return (enu_x, enu_y, enu_z)
    
    def publish_static_transforms(self):
        """Publish static transforms for robot frames.
        
        Publishes static transforms with current timestamp. ROS2 transient_local QoS on /tf_static
        ensures that late-joining nodes will receive these transforms automatically.
        
        This method is called periodically (via timer) to keep transform timestamps current,
        preventing "stale transform" issues when transforms are queried with old timestamps.
        
        Publishes:
        - vrtk_link → base_link
        - base_link → base_footprint (for 2D navigation, z=0 on ground)
        - base_link → lidar_link
        
        Raises:
            RuntimeError: If ROS node was not provided during initialization
            
        Note:
            This method should be called once after setting transform offsets.
            A timer will automatically republish transforms periodically to keep timestamps current.
            The transforms are persistent via ROS2 transient_local QoS.
        """
        if self._static_broadcaster is None:
            raise RuntimeError(
                "Cannot publish transforms: ROS node not provided. "
                "Initialize TransformManager with a Node instance."
            )
        
        # Always use current time to prevent "stale transform" issues
        # This ensures transforms have recent timestamps even if node has been running for a long time
        now = self._node.get_clock().now().to_msg()
        
        # Publish vrtk_link → base_link
        tf_vrtk_base = TransformStamped()
        tf_vrtk_base.header.stamp = now
        tf_vrtk_base.header.frame_id = "vrtk_link"
        tf_vrtk_base.child_frame_id = "base_link"
        tf_vrtk_base.transform.translation.x = self._vrtk_to_base_offset[0]
        tf_vrtk_base.transform.translation.y = self._vrtk_to_base_offset[1]
        tf_vrtk_base.transform.translation.z = self._vrtk_to_base_offset[2]
        tf_vrtk_base.transform.rotation.x = self._vrtk_to_base_rotation[0]
        tf_vrtk_base.transform.rotation.y = self._vrtk_to_base_rotation[1]
        tf_vrtk_base.transform.rotation.z = self._vrtk_to_base_rotation[2]
        tf_vrtk_base.transform.rotation.w = self._vrtk_to_base_rotation[3]
        self._static_broadcaster.sendTransform(tf_vrtk_base)
        
        # Publish base_link → base_footprint (for 2D navigation)
        # base_footprint is the projection of base_link onto the ground (z=0)
        tf_base_base_footprint = TransformStamped()
        tf_base_base_footprint.header.stamp = now
        tf_base_base_footprint.header.frame_id = "base_link"
        tf_base_base_footprint.child_frame_id = "base_footprint"
        tf_base_base_footprint.transform.translation.x = self._base_to_base_footprint_offset[0]
        tf_base_base_footprint.transform.translation.y = self._base_to_base_footprint_offset[1]
        tf_base_base_footprint.transform.translation.z = self._base_to_base_footprint_offset[2]
        tf_base_base_footprint.transform.rotation.x = self._base_to_base_footprint_rotation[0]
        tf_base_base_footprint.transform.rotation.y = self._base_to_base_footprint_rotation[1]
        tf_base_base_footprint.transform.rotation.z = self._base_to_base_footprint_rotation[2]
        tf_base_base_footprint.transform.rotation.w = self._base_to_base_footprint_rotation[3]
        self._static_broadcaster.sendTransform(tf_base_base_footprint)
        
        # Publish base_link → lidar_link
        tf_base_lidar = TransformStamped()
        tf_base_lidar.header.stamp = now
        tf_base_lidar.header.frame_id = "base_link"
        tf_base_lidar.child_frame_id = "lidar_link"
        tf_base_lidar.transform.translation.x = self._base_to_lidar_offset[0]
        tf_base_lidar.transform.translation.y = self._base_to_lidar_offset[1]
        tf_base_lidar.transform.translation.z = self._base_to_lidar_offset[2]
        tf_base_lidar.transform.rotation.x = self._base_to_lidar_rotation[0]
        tf_base_lidar.transform.rotation.y = self._base_to_lidar_rotation[1]
        tf_base_lidar.transform.rotation.z = self._base_to_lidar_rotation[2]
        tf_base_lidar.transform.rotation.w = self._base_to_lidar_rotation[3]
        self._static_broadcaster.sendTransform(tf_base_lidar)
        
        # Start periodic republishing timer if not already started and interval > 0
        # This keeps transform timestamps current, preventing "stale transform" issues
        if (self._static_transform_timer is None and 
            self._node is not None and 
            self._static_transform_republish_interval > 0.0):
            self._static_transform_timer = self._node.create_timer(
                self._static_transform_republish_interval,
                self.publish_static_transforms
            )
            if self._node is not None:
                self._node.get_logger().info(
                    f'[TransformManager] Started periodic static transform republishing '
                    f'(interval: {self._static_transform_republish_interval}s) to keep timestamps current'
                )
    
    def get_coordinate_transformer(self) -> CoordinateTransformer:
        """Get the internal CoordinateTransformer instance.
        
        Returns:
            The CoordinateTransformer instance used internally
        """
        return self._coord_transformer
    
    def set_tf_timeout(self, timeout: float):
        """Set the timeout for TF lookups.
        
        Args:
            timeout: Timeout in seconds
        """
        self._tf_timeout = timeout
    
    def set_static_transform_republish_interval(self, interval: float):
        """Set the interval for periodic republishing of static transforms.
        
        Static transforms are republished periodically to keep their timestamps current,
        preventing "stale transform" issues when transforms are queried with old timestamps.
        
        Args:
            interval: Republish interval in seconds (default: 10.0)
                     Set to 0.0 to disable periodic republishing
        """
        self._static_transform_republish_interval = interval
        
        # If timer exists, recreate it with new interval
        if self._static_transform_timer is not None and self._node is not None:
            self._static_transform_timer.cancel()
            self._static_transform_timer = None
            
            if interval > 0.0:
                self._static_transform_timer = self._node.create_timer(
                    interval,
                    self.publish_static_transforms
                )
                if self._node is not None:
                    self._node.get_logger().debug(
                        f'[TransformManager] Updated static transform republish interval to {interval}s'
                    )
    
    def get_robot_pose_tf(
        self,
        robot_base_frame: str = "base_footprint",
        global_frame: str = "map",
        timeout: Optional[float] = None
    ) -> Optional[PoseStamped]:
        """Get current robot pose using TF lookup.
        
        Queries the TF tree to get the robot's current pose in the global frame.
        This is the preferred method for getting robot pose as it uses the TF tree
        which is maintained by the localization system.
        
        Args:
            robot_base_frame: Frame ID of the robot base (default: "base_footprint")
            global_frame: Global frame ID (default: "map")
            timeout: Timeout for TF lookup in seconds (uses default if None)
            
        Returns:
            PoseStamped in global_frame, or None if lookup fails or TF not available
            
        Note:
            Requires ROS node to be provided during initialization.
            Falls back gracefully if TF is not available.
        """
        if self._tf_buffer is None or self._node is None:
            return None
        
        try:
            timeout_duration = Duration(seconds=timeout if timeout is not None else self._tf_timeout)
            transform = self._tf_buffer.lookup_transform(
                global_frame,
                robot_base_frame,
                Time(),
                timeout=timeout_duration
            )
            
            # Create pose at origin in robot_base_frame
            position = Point(x=0.0, y=0.0, z=0.0)
            orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            pose = Pose()
            pose.position = position
            pose.orientation = orientation
            
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = robot_base_frame
            robot_pose.header.stamp = transform.header.stamp
            robot_pose.pose = pose
            
            # Transform to global frame
            global_pose = do_transform_pose_stamped(robot_pose, transform)
            global_pose.header.frame_id = global_frame
            global_pose.header.stamp = transform.header.stamp
            
            return global_pose
            
        except TransformException as e:
            if self._node is not None:
                self._node.get_logger().debug(
                    f'[TransformManager] Failed to get robot pose via TF: {e}'
                )
            return None
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(
                    f'[TransformManager] Unexpected error during TF lookup: {e}'
                )
            return None
    
    def transform_pose_tf(
        self,
        pose: PoseStamped,
        target_frame: str,
        timeout: Optional[float] = None
    ) -> Optional[PoseStamped]:
        """Transform a pose to a target frame using TF.
        
        Args:
            pose: PoseStamped to transform
            target_frame: Target frame ID
            timeout: Timeout for TF lookup in seconds (uses default if None)
            
        Returns:
            Transformed PoseStamped in target_frame, or None if lookup fails
            
        Note:
            Requires ROS node to be provided during initialization.
            Falls back gracefully if TF is not available.
        """
        if self._tf_buffer is None or self._node is None:
            return None
        
        try:
            # Ensure pose has a valid timestamp
            if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
                pose.header.stamp = Time().to_msg()
            
            timeout_duration = Duration(seconds=timeout if timeout is not None else self._tf_timeout)
            
            # Try with pose timestamp first, fallback to latest if needed
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    pose.header.frame_id,
                    pose.header.stamp,
                    timeout=timeout_duration
                )
            except TransformException:
                # If exact timestamp fails (extrapolation error), use latest available
                if self._node is not None:
                    self._node.get_logger().debug(
                        f'Exact timestamp lookup failed for {pose.header.frame_id} -> {target_frame}, '
                        f'using latest available transform'
                    )
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    pose.header.frame_id,
                    Time(),
                    timeout=timeout_duration
                )
            
            # Apply transform
            transformed_pose = do_transform_pose_stamped(pose, transform)
            transformed_pose.header.frame_id = target_frame
            transformed_pose.header.stamp = transform.header.stamp
            
            return transformed_pose
            
        except TransformException as e:
            if self._node is not None:
                self._node.get_logger().debug(
                    f'[TransformManager] Failed to transform pose from {pose.header.frame_id} '
                    f'to {target_frame}: {e}'
                )
            return None
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(
                    f'[TransformManager] Unexpected error during pose transformation: {e}'
                )
            return None
    
    def validate_gps_to_map_with_tf(
        self,
        lat: float,
        lon: float,
        alt: float = 0.0,
        tolerance: float = 0.5
    ) -> bool:
        """Validate GPS→Map calculation against TF-based robot pose.
        
        Compares the GPS-based map coordinates with the current TF-based robot pose.
        This is useful for detecting discrepancies between GPS and TF systems.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
            tolerance: Maximum allowed difference in meters (default: 0.5m)
            
        Returns:
            True if difference is within tolerance, False otherwise
            
        Note:
            This is a validation/warning method only. Returns False if TF is not available.
            Does not raise exceptions - logs warnings instead.
        """
        if self._node is None:
            return False
        
        try:
            # Get GPS-based map coordinates
            gps_map_x, gps_map_y, gps_map_z = self.gps_to_map(lat, lon, alt)
            
            # Get TF-based robot pose
            tf_pose = self.get_robot_pose_tf(robot_base_frame="base_footprint", global_frame="map")
            if tf_pose is None:
                self._node.get_logger().debug(
                    "[TransformManager] Cannot validate GPS→Map: TF pose not available"
                )
                return False
            
            # Calculate difference
            diff_x = abs(gps_map_x - tf_pose.pose.position.x)
            diff_y = abs(gps_map_y - tf_pose.pose.position.y)
            diff_z = abs(gps_map_z - tf_pose.pose.position.z)
            distance = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
            
            if distance > tolerance:
                self._node.get_logger().warn(
                    f"[TransformManager] GPS→Map validation failed: "
                    f"difference {distance:.3f}m exceeds tolerance {tolerance:.3f}m. "
                    f"GPS: ({gps_map_x:.3f}, {gps_map_y:.3f}, {gps_map_z:.3f}), "
                    f"TF: ({tf_pose.pose.position.x:.3f}, {tf_pose.pose.position.y:.3f}, "
                    f"{tf_pose.pose.position.z:.3f})"
                )
                return False
            
            return True
            
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(
                    f"[TransformManager] Error during GPS→Map validation: {e}"
                )
            return False

