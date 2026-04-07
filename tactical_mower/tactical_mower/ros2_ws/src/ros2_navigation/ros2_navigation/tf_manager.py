#!/usr/bin/env python3
"""
TF Manager for handling coordinate frame transformations.

This module manages TF2 transforms for robot pose queries and pose transformations.
"""

import math
from typing import Optional
from rclpy.node import Node
from rclpy.time import Duration, Time
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


class TFManager:
    """
    Manages TF2 transforms for robot pose and pose transformations.
    """
    
    def __init__(
        self,
        node: Node,
        robot_base_frame: str,
        global_frame: str,
        tf_timeout: float
    ):
        """
        Initialize TF Manager.
        
        Args:
            node: ROS2 node instance
            robot_base_frame: Frame ID of the robot base for pose queries (e.g., 'base_footprint' for navigation)
            global_frame: Global frame ID (e.g., 'odom' or 'map')
            tf_timeout: Timeout for TF lookups in seconds
        """
        self.node = node
        self.robot_base_frame = robot_base_frame
        self.global_frame = global_frame
        self.tf_timeout = tf_timeout
        
        # Configure TF buffer with cache duration to handle delayed/stale transforms
        # Cache duration of 10 seconds should be enough for most cases
        cache_duration = Duration(seconds=10.0)
        self.tf_buffer = Buffer(cache_time=cache_duration)
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.node.get_logger().info(
            f'TFManager initialized: robot_base_frame={robot_base_frame}, '
            f'global_frame={global_frame}, tf_timeout={tf_timeout}s'
        )
    
    def _transform_pose(self, pose: PoseStamped, transform) -> PoseStamped:
        """
        Transform a pose using TransformStamped.
        
        Uses tf2_geometry_msgs.do_transform_pose for correct transformation.
        
        Args:
            pose: PoseStamped to transform
            transform: TransformStamped containing the transformation
            
        Returns:
            Transformed PoseStamped
        """
        return do_transform_pose_stamped(pose, transform)
    
    def get_robot_pose(self) -> PoseStamped:
        """
        Get the current robot pose in the global frame.
        
        Returns:
            PoseStamped in the global frame
            
        Raises:
            TransformException: If transform lookup fails
        """
        try:
            timeout_duration = Duration(seconds=self.tf_timeout)
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=timeout_duration
            )
            
            # Create pose at origin in robot_base_frame
            position = Point(x=0.0, y=0.0, z=0.0)
            orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            pose = Pose()
            pose.position = position
            pose.orientation = orientation
            
            header = Header()
            header.frame_id = self.robot_base_frame
            header.stamp = transform.header.stamp
            
            robot_pose = PoseStamped()
            robot_pose.header = header
            robot_pose.pose = pose
            
            # Transform to global frame
            global_pose = self._transform_pose(robot_pose, transform)
            global_pose.header.frame_id = self.global_frame
            global_pose.header.stamp = transform.header.stamp
            
            return global_pose
            
        except TransformException as e:
            self.node.get_logger().error(
                f'[TFManager] Failed to get robot pose: {e}'
            )
            raise
    
    def transform_pose(
        self,
        pose: PoseStamped,
        target_frame: str,
        timeout: Optional[float] = None
    ) -> PoseStamped:
        """
        Transform a pose to a target frame.
        
        Args:
            pose: Pose to transform
            target_frame: Target frame ID
            timeout: Optional timeout in seconds (defaults to self.tf_timeout)
            
        Returns:
            Transformed PoseStamped in target_frame
            
        Raises:
            TransformException: If transform lookup fails
        """
        # Use provided timeout or default
        if timeout is None:
            timeout = self.tf_timeout
        
        try:
            # If frames are the same, no transformation needed
            if pose.header.frame_id == target_frame:
                return pose
            
            # Ensure pose has a valid timestamp
            if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
                pose.header.stamp = Time().to_msg()
            
            # Lookup transform - try with pose timestamp first, fallback to latest if needed
            timeout_duration = Duration(seconds=timeout)
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    pose.header.frame_id,
                    pose.header.stamp,
                    timeout=timeout_duration
                )
            except TransformException:
                # If exact timestamp fails (extrapolation error), use latest available
                self.node.get_logger().debug(
                    f'Exact timestamp lookup failed for {pose.header.frame_id} -> {target_frame}, '
                    f'using latest available transform'
                )
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    pose.header.frame_id,
                    Time(),
                    timeout=timeout_duration
                )
            
            # Apply transform
            transformed_pose = self._transform_pose(pose, transform)
            transformed_pose.header.frame_id = target_frame
            transformed_pose.header.stamp = transform.header.stamp
            
            return transformed_pose
            
        except TransformException as e:
            self.node.get_logger().error(
                f'Failed to transform pose from {pose.header.frame_id} to {target_frame}: {e}'
            )
            raise
