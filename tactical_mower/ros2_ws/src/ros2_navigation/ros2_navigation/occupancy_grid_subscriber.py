#!/usr/bin/env python3
"""
OccupancyGrid Subscriber for receiving and managing occupancy grid data.

This module subscribes to occupancy grid topics and provides
thread-safe access to the current grid.
"""

import threading
import time
from typing import Optional
from rclpy.node import Node
from rclpy.time import Time, Duration
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer
from ros2_navigation.grid_utils import validate_grid, transform_occupancy_grid


class OccupancyGridSubscriber:
    """
    Subscribes to occupancy grid topic and manages current grid data.
    """
    
    def __init__(
        self,
        node: Node,
        topic_name: str,
        grid_update_callback=None,
        auto_transform_to_frame: Optional[str] = None,
        tf_buffer: Optional[Buffer] = None,
        tf_timeout: float = 1.0
    ):
        """
        Initialize OccupancyGrid Subscriber.
        
        Args:
            node: ROS2 node instance
            topic_name: Name of the occupancy grid topic to subscribe to
            grid_update_callback: Optional callback function(grid) called when grid is updated
            auto_transform_to_frame: Optional target frame for automatic transformation (e.g., 'odom')
            tf_buffer: Optional TF2 buffer for transformations (required if auto_transform_to_frame is set)
            tf_timeout: Timeout for TF lookups in seconds
        """
        self.node = node
        self.topic_name = topic_name
        self._current_grid = None
        self._lock = threading.Lock()
        self.grid_update_callback = grid_update_callback
        self.auto_transform_to_frame = auto_transform_to_frame
        self.tf_buffer = tf_buffer
        self.tf_timeout = tf_timeout
        
        if auto_transform_to_frame and tf_buffer is None:
            self.node.get_logger().warn(
                f'auto_transform_to_frame is set to "{auto_transform_to_frame}" but tf_buffer is None. '
                f'Grid transformation will be disabled.'
            )
            self.auto_transform_to_frame = None
        
        # Rate limiting for TF warnings (to avoid log spam)
        self._last_tf_warn_time: Optional[float] = None
        self._tf_warn_interval: float = 20.0  # seconds
        
        # Create subscription
        self.subscription = self.node.create_subscription(
            OccupancyGrid,
            topic_name,
            self.occupancy_grid_callback,
            10
        )
        
        transform_info = f', auto-transform to {auto_transform_to_frame}' if auto_transform_to_frame else ''
        self.node.get_logger().info(
            f'OccupancyGridSubscriber initialized, subscribed to: {topic_name}{transform_info}'
        )
        if auto_transform_to_frame:
            self.node.get_logger().info(
                f'[FRAME] Grid auto-transformation enabled: '
                f'grids will be transformed from source frame → {auto_transform_to_frame}'
            )
    
    def occupancy_grid_callback(self, msg: OccupancyGrid):
        """
        Callback for occupancy grid messages.
        
        Transforms grid to target frame if auto-transform is enabled.
        Logs frame information for debugging and clarity.
        
        Args:
            msg: Received occupancy grid message
        """
        # Log received grid frame information
        self.node.get_logger().debug(
            f'[FRAME] Received grid on {self.topic_name}: '
            f'frame_id={msg.header.frame_id}, '
            f'size={msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution:.3f}m'
        )
        
        # Transform grid if auto-transform is enabled and frame mismatch
        grid_to_store = msg
        original_frame = msg.header.frame_id
        
        if self.auto_transform_to_frame and self.tf_buffer:
            if msg.header.frame_id != self.auto_transform_to_frame:
                # Check if target frame exists in TF tree before attempting transformation
                # This helps diagnose issues early (e.g., map frame not yet published by Fixposition Driver)
                try:
                    can_transform = self.tf_buffer.can_transform(
                        self.auto_transform_to_frame,
                        msg.header.frame_id,
                        Time(),
                        timeout=Duration(seconds=0.1)  # Quick check
                    )
                    if not can_transform:
                        # Rate-limit the warning to avoid log spam
                        current_time = time.time()
                        if (self._last_tf_warn_time is None or 
                            (current_time - self._last_tf_warn_time) >= self._tf_warn_interval):
                            self.node.get_logger().warn(
                                f'[FRAME] Target frame "{self.auto_transform_to_frame}" not yet available in TF tree. '
                                f'This may happen if the Fixposition Driver has not yet published all required transforms. '
                                f'Using original grid in frame {original_frame} for now. '
                                f'Transformation will be retried on next grid update.'
                            )
                            self._last_tf_warn_time = current_time
                        # Keep original grid - will retry on next update
                        grid_to_store = msg
                    else:
                        # Frame exists, attempt transformation
                        transform_start = time.time()
                        grid_to_store = transform_occupancy_grid(
                            msg,
                            self.auto_transform_to_frame,
                            self.tf_buffer,
                            node=self.node,
                            tf_timeout=self.tf_timeout
                        )
                        transform_time = (time.time() - transform_start) * 1000
                        # Reset warn timer on success so we log immediately if problems recur
                        self._last_tf_warn_time = None
                except Exception as e:
                    # Log the error with full details
                    error_str = str(e)
                    if "does not exist" in error_str:
                        # Rate-limit the warning to avoid log spam
                        current_time = time.time()
                        if (self._last_tf_warn_time is None or 
                            (current_time - self._last_tf_warn_time) >= self._tf_warn_interval):
                            self.node.get_logger().warn(
                                f'[FRAME] Target frame "{self.auto_transform_to_frame}" not yet available in TF tree: {e}. '
                                f'This is normal during startup. Using original grid in frame {original_frame} for now. '
                                f'Transformation will be retried on next grid update.'
                            )
                            self._last_tf_warn_time = current_time
                    else:
                        self.node.get_logger().error(
                            f'[FRAME] ✗ Failed to transform grid from {original_frame} to '
                            f'{self.auto_transform_to_frame}: {e}'
                        )
                        self.node.get_logger().warn(
                            f'[FRAME] Using original grid in frame {original_frame}. '
                            f'This may cause performance issues during path planning.'
                        )
                    # Keep original grid if transformation fails
                    grid_to_store = msg
            else:
                self.node.get_logger().debug(
                    f'[FRAME] Grid already in target frame {self.auto_transform_to_frame}, '
                    f'skipping transformation'
                )
        else:
            if not self.auto_transform_to_frame:
                self.node.get_logger().debug(
                    f'[FRAME] Auto-transform disabled, keeping grid in original frame {original_frame}'
                )
            elif not self.tf_buffer:
                self.node.get_logger().warn(
                    f'[FRAME] Auto-transform enabled but tf_buffer is None, '
                    f'keeping grid in original frame {original_frame}'
                )
        
        # Store the grid (transformed or original)
        with self._lock:
            self._current_grid = grid_to_store
        
        # Log stored grid frame
        self.node.get_logger().debug(
            f'[FRAME] Stored grid frame: {grid_to_store.header.frame_id}'
        )
        
        if self.grid_update_callback is not None:
            try:
                self.grid_update_callback(grid_to_store)
            except Exception as e:
                self.node.get_logger().warn(f'Grid update callback failed: {e}')
    
    def get_current_grid(self) -> OccupancyGrid:
        """
        Get current occupancy grid.
        
        Returns:
            Current occupancy grid, or None if not available
        """
        with self._lock:
            grid = self._current_grid
            if grid is not None:
                self.node.get_logger().debug(
                    f'[FRAME] Retrieved grid from cache: frame_id={grid.header.frame_id}'
                )
            return grid
    
    def is_grid_available(self) -> bool:
        """
        Check if occupancy grid is available.
        
        Returns:
            True if grid is available, False otherwise
        """
        with self._lock:
            return self._current_grid is not None

