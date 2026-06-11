import os
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, List, Tuple, Optional

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

from interfaces.srv import CaptureEventClips
from .camera_config import EVENT_RECORDING_CAMERA_IDS


@dataclass
class CameraConfig:
    camera_id: str
    topic: str
    frame_rate: float = 10.0


@dataclass
class FrameEntry:
    stamp_sec: float
    frame_bgr: any  # OpenCV image


DEFAULT_CAMERAS = [
    #CameraConfig(camera_id='main_color', topic='/camera/camera/color/image_raw', frame_rate=10.0),
    CameraConfig(camera_id='eneo_thermal', topic='/ip_camera/thermal_raw', frame_rate=10.0),
    # LAZY: dropped to allow the upstream rtsp_image_publisher to pause when idle.
    #CameraConfig(camera_id='eneo_rgb', topic='/ip_camera/rgb_raw', frame_rate=10.0),
]


class VideoRingbufferNode(Node):
    """
    Node that maintains in-memory frame ringbuffers per camera and
    writes MP4 clips for security events on request.
    """

    def __init__(self) -> None:
        super().__init__('video_ringbuffer')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('storage_root', '/app/ros2_ws/data/security_events')
        self.declare_parameter('max_total_size_mb', 20480.0)  # 20 GB
        self.declare_parameter('pre_event_seconds', 60.0)  # Increased to 60 seconds for security events
        self.declare_parameter('post_event_seconds', 10.0)
        self.declare_parameter('max_buffer_memory_mb', 2048.0)  # Max 2GB for all buffers (supports 60s pre-event for 2 cameras)
        self.declare_parameter('max_frames_per_camera', 700)  # Hard limit: 70s total (60s pre + 10s post) * 10 FPS

        storage_root_path = self.get_parameter('storage_root').get_parameter_value().string_value
        self.storage_root = Path(storage_root_path)
        self.storage_root.mkdir(parents=True, exist_ok=True)

        self.max_total_size_mb = float(self.get_parameter('max_total_size_mb').value)
        self.pre_event_seconds = float(self.get_parameter('pre_event_seconds').value)
        self.post_event_seconds = float(self.get_parameter('post_event_seconds').value)
        self.max_buffer_memory_mb = float(self.get_parameter('max_buffer_memory_mb').value)
        self.max_frames_per_camera = int(self.get_parameter('max_frames_per_camera').value)

        # Tunables (kept together to avoid magic numbers below)
        self.buffer_safety_margin_seconds = 5.0
        self.cleanup_extra_buffer_seconds = 10.0
        self.frame_time_tolerance_sec = 1.0
        self.frame_time_expand_tolerance_sec = 2.0
        self.partial_window_warn_slack_sec = 0.5
        self.search_window_min_seconds = 30.0
        self.fps_min = 1.0
        self.fps_max = 30.0
        self.default_fps = 10.0
        self.min_frame_interval_default = 0.1
        self.capture_wait_slack_seconds = 0.5
        self.capture_wait_ceiling_seconds = 20.0
        self.capture_window_safety_margin_seconds = 10.0
        self.memory_cleanup_thresholds = {
            'warn': 0.75,
            'aggressive': 0.85,
            'critical': 0.95,
        }
        self.memory_cleanup_drop = {
            'warn': 0.3,         # keep 70%
            'aggressive': 0.5,   # keep 50%
            'critical_target': 0.7,  # target 70% of limit
        }
        self.memory_usage_warn_pct = 0.8
        self.offline_warn_seconds = 60.0
        self.proactive_cleanup_threshold_pct = 0.7
        self.proactive_cleanup_after_drop_pct = 0.8
        self.proactive_size_reduction_factor = 0.8
        self.timer_log_camera_interval = 60.0
        self.timer_log_memory_interval = 30.0
        self.timer_cleanup_interval = 10.0

        # To keep memory bounded we only store slightly more than pre+post seconds
        # Calculate based on configured pre/post event times (default 70s total for 60s pre + 10s post)
        self.max_buffer_seconds = (
            self.pre_event_seconds
            + self.post_event_seconds
            + self.buffer_safety_margin_seconds
        )

        self.camera_configs: Dict[str, CameraConfig] = {}
        self.frame_buffers: Dict[str, Deque[FrameEntry]] = {}
        self.last_frame_time: Dict[str, float] = {}
        # Frame throttling: track last frame time per camera to only store frames at desired rate
        self.last_buffer_frame_time: Dict[str, float] = {}

        # Callback groups to allow concurrent subscriptions and service processing
        self.image_cb_group = ReentrantCallbackGroup()
        self.service_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()
        
        # Pending capture requests (for async processing)
        self.pending_captures: Dict[str, Dict] = {}

        # QoS profile matching the RTSP publishers (BEST_EFFORT for streaming)
        # CRITICAL: depth=1 to prevent buffering - only latest frame, no blocking
        # The ringbuffer maintains its own internal buffer, so ROS2 depth=1 is sufficient
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only 1 frame - no ROS2 buffering, prevents blocking other subscribers
        )

        # Register default cameras (can later be extended to load from a YAML config if needed)
        for cfg in DEFAULT_CAMERAS:
            self.camera_configs[cfg.camera_id] = cfg
            self.frame_buffers[cfg.camera_id] = deque()
            self.last_frame_time[cfg.camera_id] = 0.0
            self.last_buffer_frame_time[cfg.camera_id] = 0.0

            self.create_subscription(
                Image,
                cfg.topic,
                self._make_image_callback(cfg.camera_id),
                camera_qos,
                callback_group=self.image_cb_group,
            )
            self.get_logger().info(f'Registered camera "{cfg.camera_id}" on topic {cfg.topic}')

        # Service for capturing event clips
        self.capture_service = self.create_service(
            CaptureEventClips,
            'capture_event_clips',
            self.handle_capture_event_clips,
            callback_group=self.service_cb_group,
        )

        # Periodic log of camera activity to see which cameras are online/offline
        self.create_timer(
            self.timer_log_camera_interval,
            self._log_camera_activity,
            callback_group=self.timer_cb_group,
        )
        
        # Periodic memory monitoring and proactive cleanup
        self.create_timer(
            self.timer_log_memory_interval,
            self._log_memory_usage,
            callback_group=self.timer_cb_group,
        )
        # Proactive cleanup timer - runs more frequently to prevent memory buildup
        self.create_timer(
            self.timer_cleanup_interval,
            self._proactive_cleanup,
            callback_group=self.timer_cb_group,
        )

        self.get_logger().info('VideoRingbufferNode initialized')

    def _estimate_frame_size_mb(self, frame: np.ndarray) -> float:
        """Estimate memory size of a frame in MB."""
        if frame is None:
            return 0.0
        # numpy array size in bytes: height * width * channels * dtype_size
        size_bytes = frame.nbytes
        return size_bytes / (1024.0 * 1024.0)
    
    def _get_total_buffer_memory_mb(self) -> float:
        """Calculate total memory used by all frame buffers."""
        total_mb = 0.0
        for buf in self.frame_buffers.values():
            for entry in buf:
                total_mb += self._estimate_frame_size_mb(entry.frame_bgr)
        return total_mb

    def _make_image_callback(self, camera_id: str):
        def callback(msg: Image) -> None:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'CvBridge conversion failed for {camera_id}: {e}')
                return

            now_sec = self._rostime_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else time.time()

            buf = self.frame_buffers.get(camera_id)
            if buf is None:
                return

            # Update last frame timestamp for activity checks
            self.last_frame_time[camera_id] = now_sec

            # Frame throttling: only store frames at the configured frame rate
            cfg = self.camera_configs.get(camera_id)
            if cfg:
                min_interval = (
                    1.0 / cfg.frame_rate if cfg.frame_rate > 0 else self.min_frame_interval_default
                )
                last_buffer_time = self.last_buffer_frame_time.get(camera_id, 0.0)
                if now_sec - last_buffer_time < min_interval:
                    # Skip this frame - throttling to reduce memory usage
                    return

            # Log first frame received for each camera (helps diagnose startup issues)
            if len(buf) == 0:
                self.get_logger().info(f'First frame received for camera "{camera_id}" at {now_sec:.2f}')
            else:
                # CRITICAL: Reject frames with timestamps older than the newest frame in buffer
                # This prevents old frames (that might have been published with wrong timestamps) 
                # from being stored in the buffer. We only keep frames that are newer or equal.
                newest_stamp = buf[-1].stamp_sec
                if now_sec < newest_stamp - 0.1:  # Allow 100ms tolerance for timestamp precision
                    self.get_logger().warn(
                        f'Rejecting frame for {camera_id}: timestamp {now_sec:.2f} is older than '
                        f'newest frame in buffer ({newest_stamp:.2f}, difference: {newest_stamp - now_sec:.2f}s). '
                        f'This indicates an old frame was published. Skipping.'
                    )
                    return

            # Update last buffer frame time for throttling
            self.last_buffer_frame_time[camera_id] = now_sec

            # Check memory limits before adding frame
            frame_size_mb = self._estimate_frame_size_mb(frame)
            current_total_mb = self._get_total_buffer_memory_mb()
            
            # Proactive cleanup at multiple thresholds to prevent hitting hard limit
            if current_total_mb + frame_size_mb > self.max_buffer_memory_mb * self.memory_cleanup_thresholds['warn']:
                # At 75%: aggressive cleanup - drop 30% of oldest frames across all cameras
                self.get_logger().debug(f'Memory usage high ({current_total_mb:.1f}MB), performing proactive cleanup')
                for cam_id, cam_buf in self.frame_buffers.items():
                    target_size = max(1, int(len(cam_buf) * (1 - self.memory_cleanup_drop['warn'])))
                    while len(cam_buf) > target_size:
                        cam_buf.popleft()
                # Recalculate after cleanup
                current_total_mb = self._get_total_buffer_memory_mb()
            
            # More aggressive cleanup at 85%
            if current_total_mb + frame_size_mb > self.max_buffer_memory_mb * self.memory_cleanup_thresholds['aggressive']:
                self.get_logger().warn(f'Memory usage very high ({current_total_mb:.1f}MB), performing aggressive cleanup')
                # Drop 50% of oldest frames across all cameras
                for cam_id, cam_buf in self.frame_buffers.items():
                    target_size = max(1, int(len(cam_buf) * (1 - self.memory_cleanup_drop['aggressive'])))
                    while len(cam_buf) > target_size:
                        cam_buf.popleft()
                current_total_mb = self._get_total_buffer_memory_mb()
            
            # Critical cleanup at 95% - drop frames until we're well below limit
            if current_total_mb + frame_size_mb > self.max_buffer_memory_mb * self.memory_cleanup_thresholds['critical']:
                self.get_logger().error(f'Memory usage critical ({current_total_mb:.1f}MB), performing critical cleanup')
                # Drop frames until we're at 70% of limit
                target_mb = self.max_buffer_memory_mb * self.memory_cleanup_drop['critical_target']
                while current_total_mb > target_mb:
                    # Drop oldest frame from largest buffer
                    largest_cam = max(self.frame_buffers.items(), key=lambda x: len(x[1]) if x[1] else 0)
                    if largest_cam[1] and len(largest_cam[1]) > 0:
                        largest_cam[1].popleft()
                        current_total_mb = self._get_total_buffer_memory_mb()
                    else:
                        break
            
            # Hard limit: don't exceed max frames per camera (frame count limit)
            if len(buf) >= self.max_frames_per_camera:
                buf.popleft()

            # Final check: don't add frame if it would exceed memory limit
            if current_total_mb + frame_size_mb > self.max_buffer_memory_mb:
                self.get_logger().warn(
                    f'Cannot add frame for {camera_id}: would exceed memory limit '
                    f'({current_total_mb + frame_size_mb:.1f}MB > {self.max_buffer_memory_mb:.1f}MB). Skipping frame.'
                )
                return

            buf.append(FrameEntry(stamp_sec=now_sec, frame_bgr=frame))
            
            # Drop old frames beyond buffer window (time-based cleanup)
            # Use a more conservative cleanup to ensure we keep enough frames
            # Keep at least pre_event_seconds + post_event_seconds + 10s buffer
            buffer_window = max(
                self.max_buffer_seconds,
                self.pre_event_seconds + self.post_event_seconds + self.cleanup_extra_buffer_seconds,
            )
            min_time = now_sec - buffer_window
            dropped = 0
            while buf and buf[0].stamp_sec < min_time:
                buf.popleft()
                dropped += 1
            
            # Additional safety: if buffer still too large, drop oldest frames
            while len(buf) > self.max_frames_per_camera:
                buf.popleft()
                dropped += 1

        return callback

    @staticmethod
    def _rostime_to_sec(t: RosTime) -> float:
        return float(t.sec) + float(t.nanosec) * 1e-9

    def _log_camera_activity(self) -> None:
        """Log once per minute which cameras appear offline (no frames recently)."""
        try:
            now = time.time()
            for cam_id, last in self.last_frame_time.items():
                if last == 0.0 or (now - last) > self.offline_warn_seconds:
                    self.get_logger().warn(f'Camera "{cam_id}" appears offline (no frames in last 60s)')
        except Exception as e:
            self.get_logger().error(f'Error while checking camera activity: {e}')
    
    def _log_memory_usage(self) -> None:
        """Log memory usage statistics for monitoring."""
        try:
            total_mb = self._get_total_buffer_memory_mb()
            frame_counts = {cam_id: len(buf) for cam_id, buf in self.frame_buffers.items()}
            total_frames = sum(frame_counts.values())
            
            self.get_logger().info(
                f'Buffer memory: {total_mb:.1f}MB / {self.max_buffer_memory_mb:.1f}MB, '
                f'Total frames: {total_frames}, Per camera: {frame_counts}'
            )
            
            # Warn if memory usage is high
            if total_mb > self.max_buffer_memory_mb * self.memory_usage_warn_pct:
                self.get_logger().debug(f'High memory usage: {total_mb:.1f}MB ({100*total_mb/self.max_buffer_memory_mb:.1f}%)')
        except Exception as e:
            self.get_logger().error(f'Error while logging memory usage: {e}')

    def _proactive_cleanup(self) -> None:
        """Proactively clean up old frames to prevent memory buildup."""
        try:
            import time
            now_sec = time.time()
            total_mb = self._get_total_buffer_memory_mb()
            usage_percent = (total_mb / self.max_buffer_memory_mb) * 100.0
            
            # If usage is above 70%, proactively drop old frames
            if total_mb > self.max_buffer_memory_mb * self.proactive_cleanup_threshold_pct:
                self.get_logger().debug(
                    f'Proactive cleanup: memory at {total_mb:.1f}MB ({usage_percent:.1f}%), '
                    f'dropping frames older than buffer window'
                )
                
                min_time = now_sec - self.max_buffer_seconds
                total_dropped = 0
                
                # Drop old frames from all cameras
                for cam_id, buf in self.frame_buffers.items():
                    dropped = 0
                    while buf and buf[0].stamp_sec < min_time:
                        buf.popleft()
                        dropped += 1
                    total_dropped += dropped
                
                if total_dropped > 0:
                    new_total_mb = self._get_total_buffer_memory_mb()
                    self.get_logger().info(
                        f'Proactive cleanup: dropped {total_dropped} old frames, '
                        f'memory reduced from {total_mb:.1f}MB to {new_total_mb:.1f}MB'
                    )
                
                # If still above 80% after time-based cleanup, reduce buffer sizes
                new_total_mb = self._get_total_buffer_memory_mb()
                if new_total_mb > self.max_buffer_memory_mb * self.proactive_cleanup_after_drop_pct:
                    self.get_logger().warn(
                        f'Memory still high after cleanup ({new_total_mb:.1f}MB), '
                        f'performing size-based reduction'
                    )
                    for cam_id, buf in self.frame_buffers.items():
                        if len(buf) > 0:
                            target_size = max(1, int(len(buf) * self.proactive_size_reduction_factor))  # Reduce by 20%
                            while len(buf) > target_size:
                                buf.popleft()
                    
                    final_mb = self._get_total_buffer_memory_mb()
                    self.get_logger().info(
                        f'Size-based cleanup complete: memory now at {final_mb:.1f}MB'
                    )
        except Exception as e:
            self.get_logger().error(f'Error during proactive cleanup: {e}', exc_info=True)

    def handle_capture_event_clips(self, request: CaptureEventClips.Request, response: CaptureEventClips.Response) -> CaptureEventClips.Response:
        """Service handler - performs capture immediately without blocking executor."""
        try:
            event_id = request.event_id or f'event_{int(time.time())}'
            # Use provided event_time if non-zero, otherwise use now
            if request.event_time.sec or request.event_time.nanosec:
                event_time_sec = self._rostime_to_sec(request.event_time)
            else:
                event_time_sec = time.time()

            pre_s = float(request.pre_event_seconds) if request.pre_event_seconds > 0 else self.pre_event_seconds
            post_s = float(request.post_event_seconds) if request.post_event_seconds > 0 else self.post_event_seconds

            # Filter camera IDs - only allow cameras configured for event recording
            requested_camera_ids = list(request.camera_ids) if request.camera_ids else list(self.camera_configs.keys())
            camera_ids = [cam_id for cam_id in requested_camera_ids if cam_id in EVENT_RECORDING_CAMERA_IDS]
            
            if len(camera_ids) != len(requested_camera_ids):
                self.get_logger().info(f'Filtered camera IDs: {requested_camera_ids} -> {camera_ids}')

            self.get_logger().info(
                f'Capture request for event {event_id}: event_time={event_time_sec:.2f}, '
                f'window=[{event_time_sec - pre_s:.2f}, {event_time_sec + post_s:.2f}]'
            )

            # CRITICAL: Service handlers block the executor, so we must complete quickly.
            # We perform the capture immediately using available frames.
            # Since callbacks continue processing while we work, we'll get frames up to "now".
            # Post-event frames may be missing, but we capture what we have.
            result = self._capture_event_clips_sync(
                event_id, event_time_sec, camera_ids, pre_s, post_s
            )
            
            response.success = result['success']
            response.message = result['message']
            response.file_paths = result['file_paths']
            response.camera_ids_out = result['camera_ids_out']
            return response
        except Exception as e:
            self.get_logger().error(f'Error in handle_capture_event_clips: {e}', exc_info=True)
            response.success = False
            response.message = str(e)
            response.file_paths = []
            response.camera_ids_out = []
            return response

    def _capture_event_clips_sync(self, event_id: str, event_time_sec: float, 
                                  camera_ids: List[str], pre_s: float, post_s: float) -> Dict:
        """Perform capture synchronously - must complete quickly to avoid blocking executor."""
        try:
            # Ensure buffer is large enough for requested time window
            required_buffer_seconds = pre_s + post_s + self.capture_window_safety_margin_seconds
            if required_buffer_seconds > self.max_buffer_seconds:
                self.get_logger().warn(
                    f'Requested time window ({pre_s + post_s}s) requires buffer of {required_buffer_seconds}s, '
                    f'but current buffer is only {self.max_buffer_seconds}s. Some frames may be missing. '
                    f'Consider increasing max_buffer_seconds parameter.'
                )

            start_time = event_time_sec - pre_s
            end_time = event_time_sec + post_s
            now = time.time()
            # Wait briefly to collect post-event frames while the executor keeps handling camera callbacks
            self._wait_for_post_event_frames(end_time=end_time, post_s=post_s)
            now = time.time()

            self.get_logger().info(
                f'Capturing event {event_id}: event_time={event_time_sec:.2f}, '
                f'window=[{start_time:.2f}, {end_time:.2f}], now={now:.2f}'
            )

            # We don't wait here because service handlers block the executor.
            # We capture immediately using all available frames up to "now".
            # Post-event frames that arrive after the service call won't be captured,
            # but this is the best we can do without blocking the executor.

            event_dir = self.storage_root / event_id
            event_dir.mkdir(parents=True, exist_ok=True)

            file_paths: List[str] = []
            camera_ids_out: List[str] = []

            for cam_id in camera_ids:
                cfg = self.camera_configs.get(cam_id)
                buf = self.frame_buffers.get(cam_id)
                if cfg is None or buf is None:
                    self.get_logger().warn(f'Unknown camera_id "{cam_id}" in capture_event_clips')
                    continue

                # Log buffer state AFTER waiting - this shows what we actually have
                buf_size = len(buf)
                if buf_size > 0:
                    oldest_stamp = buf[0].stamp_sec
                    newest_stamp = buf[-1].stamp_sec
                    self.get_logger().info(
                        f'Camera "{cam_id}": buffer has {buf_size} frames, '
                        f'time range: {oldest_stamp:.2f} to {newest_stamp:.2f}, '
                        f'requested: {start_time:.2f} to {end_time:.2f}, now={now:.2f}'
                    )
                    
                    # Warn if newest frame is significantly older than end_time
                    if newest_stamp < end_time - 2.0:
                        self.get_logger().warn(
                            f'Camera "{cam_id}": newest frame ({newest_stamp:.2f}) is {end_time - newest_stamp:.2f}s '
                            f'older than requested end_time ({end_time:.2f}). Missing post-event frames!'
                        )
                else:
                    self.get_logger().warn(
                        f'Camera "{cam_id}": buffer is empty (no frames received)'
                    )
                    # Check if camera appears offline
                    last_frame = self.last_frame_time.get(cam_id, 0.0)
                    if last_frame == 0.0:
                        self.get_logger().warn(f'Camera "{cam_id}": never received any frames')
                    else:
                        time_since_last = time.time() - last_frame
                        self.get_logger().warn(
                            f'Camera "{cam_id}": last frame was {time_since_last:.1f}s ago'
                        )
                    continue

                # Filter frames within time window and sort by timestamp
                # Use larger tolerance (1.0s) to account for timestamp precision issues and frame timing variations
                tolerance = self.frame_time_tolerance_sec
                
                # If we waited and newest frame is still before end_time, use all available frames up to now
                # This ensures we capture all post-event frames that are actually in the buffer
                actual_end_time = end_time
                if buf_size > 0 and newest_stamp < end_time - tolerance:
                    # Newest frame is significantly before end_time - use all frames up to newest_stamp
                    # This means we didn't get all post-event frames, but we capture what we have
                    actual_end_time = min(newest_stamp + tolerance, now)  # Use newest frame or now, whichever is earlier
                    self.get_logger().warn(
                        f'Camera "{cam_id}": Using available frames up to {actual_end_time:.2f} '
                        f'(requested {end_time:.2f}, newest in buffer: {newest_stamp:.2f})'
                    )
                
                frames = [
                    entry
                    for entry in buf
                    if (start_time - tolerance) <= entry.stamp_sec <= (actual_end_time + tolerance)
                ]
                frames.sort(key=lambda e: e.stamp_sec)  # Ensure frames are sorted by timestamp
                
                # If we have frames but they don't cover the full time window, try to expand the search
                if frames:
                    actual_start = frames[0].stamp_sec
                    actual_end = frames[-1].stamp_sec
                    actual_duration = actual_end - actual_start
                    expected_duration = pre_s + post_s
                    
                    # If we're missing frames at the beginning or end, try to find more
                    if (
                        actual_start > start_time + self.partial_window_warn_slack_sec
                        or actual_end < end_time - self.partial_window_warn_slack_sec
                    ):
                        self.get_logger().warn(
                            f'Frame window incomplete for "{cam_id}": '
                            f'found {len(frames)} frames covering {actual_duration:.2f}s '
                            f'(expected {expected_duration:.2f}s), '
                            f'range: [{actual_start:.2f}, {actual_end:.2f}], '
                            f'requested: [{start_time:.2f}, {end_time:.2f}]'
                        )
                        # Try to find additional frames with larger tolerance
                        expanded_frames = [
                            entry
                            for entry in buf
                            if (start_time - self.frame_time_expand_tolerance_sec)
                            <= entry.stamp_sec
                            <= (end_time + self.frame_time_expand_tolerance_sec)
                        ]
                        if len(expanded_frames) > len(frames):
                            expanded_frames.sort(key=lambda e: e.stamp_sec)
                            # Filter to only include frames within the requested window (with tolerance)
                            frames = [entry for entry in expanded_frames if (start_time - tolerance) <= entry.stamp_sec <= (end_time + tolerance)]
                            frames.sort(key=lambda e: e.stamp_sec)
                            self.get_logger().info(
                                f'Expanded search found {len(frames)} frames for "{cam_id}"'
                            )
                
                if not frames:
                    # Try to find closest frames if exact match fails
                    all_frames = list(buf)
                    if all_frames:
                        # Find all frames within a larger window around the event time
                        search_window = max(pre_s + post_s, self.search_window_min_seconds)
                        candidate_frames = [
                            entry for entry in all_frames 
                            if abs(entry.stamp_sec - event_time_sec) <= search_window
                        ]
                        if candidate_frames:
                            # Sort by timestamp and take frames closest to the requested window
                            candidate_frames.sort(key=lambda e: e.stamp_sec)
                            # Try to get frames that span the requested time window
                            frames_in_window = [
                                entry for entry in candidate_frames
                                if start_time <= entry.stamp_sec <= end_time
                            ]
                            if frames_in_window:
                                frames = frames_in_window
                            else:
                                # If no frames in exact window, take closest frames
                                expected_frame_count = int((pre_s + post_s) * cfg.frame_rate)
                                frames = candidate_frames[:expected_frame_count]
                                self.get_logger().warn(
                                    f'No frames in exact window for "{cam_id}", using {len(frames)} closest frames '
                                    f'(time range: {frames[0].stamp_sec:.2f} to {frames[-1].stamp_sec:.2f}, '
                                    f'requested: {start_time:.2f} to {end_time:.2f})'
                                )
                    
                    if not frames:
                        self.get_logger().warn(
                            f'No frames available for camera "{cam_id}" in requested window '
                            f'({start_time:.2f} to {end_time:.2f}). Buffer has {len(buf)} frames total.'
                        )
                        continue
                
                # Log actual clip duration for debugging
                if frames:
                    actual_start = frames[0].stamp_sec
                    actual_end = frames[-1].stamp_sec
                    actual_duration = actual_end - actual_start
                    expected_duration = pre_s + post_s
                    frame_count = len(frames)
                    self.get_logger().info(
                        f'Camera "{cam_id}": {frame_count} frames, '
                        f'actual duration: {actual_duration:.2f}s (expected: {expected_duration:.2f}s), '
                        f'time range: {actual_start:.2f} to {actual_end:.2f}'
                    )

                # Write MP4 clip using OpenCV VideoWriter
                first_frame = frames[0].frame_bgr
                height, width = first_frame.shape[:2]
                
                # Calculate actual FPS from frame timestamps to ensure correct playback speed
                # This is critical: if we use the wrong FPS, the video will play at wrong speed
                if len(frames) > 1:
                    # Calculate average time between frames
                    time_diffs = []
                    for i in range(1, len(frames)):
                        diff = frames[i].stamp_sec - frames[i-1].stamp_sec
                        if diff > 0:  # Only count positive differences
                            time_diffs.append(diff)
                    
                    if time_diffs:
                        avg_frame_interval = sum(time_diffs) / len(time_diffs)
                        calculated_fps = 1.0 / avg_frame_interval if avg_frame_interval > 0 else cfg.frame_rate
                        # Clamp FPS to reasonable range
                        calculated_fps = max(self.fps_min, min(self.fps_max, calculated_fps))
                        fps = calculated_fps
                        self.get_logger().info(
                            f'Camera "{cam_id}": Calculated FPS from timestamps: {fps:.2f} '
                            f'(avg interval: {avg_frame_interval:.3f}s, {len(time_diffs)} intervals)'
                        )
                    else:
                        fps = cfg.frame_rate if cfg.frame_rate > 0 else self.default_fps
                        self.get_logger().warn(
                            f'Camera "{cam_id}": Could not calculate FPS from timestamps, using configured: {fps:.2f}'
                        )
                else:
                    fps = cfg.frame_rate if cfg.frame_rate > 0 else self.default_fps
                    self.get_logger().warn(
                        f'Camera "{cam_id}": Only {len(frames)} frame(s), using configured FPS: {fps:.2f}'
                    )
                
                out_path = event_dir / f'{cam_id}.mp4'

                # Try H.264 codec first (better browser support), fallback to mp4v
                fourcc = cv2.VideoWriter_fourcc(*'avc1')  # H.264 codec for better browser compatibility
                writer = cv2.VideoWriter(str(out_path), fourcc, fps, (width, height))
                
                # If H.264 fails, try mp4v as fallback
                if not writer.isOpened():
                    self.get_logger().warn(f'H.264 codec not available for {cam_id}, falling back to mp4v')
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (width, height))

                if not writer.isOpened():
                    self.get_logger().error(f'Failed to open VideoWriter for {cam_id} with codec {fourcc}')
                    continue

                for entry in frames:
                    writer.write(entry.frame_bgr)

                writer.release()
                
                # Log final video info
                self.get_logger().info(
                    f'Camera "{cam_id}": Wrote video with {len(frames)} frames at {fps:.2f} FPS, '
                    f'expected duration: {len(frames) / fps:.2f}s'
                )
                file_paths.append(str(out_path))
                camera_ids_out.append(cam_id)

            # Enforce max_total_size_mb by pruning oldest event directories
            self._prune_storage()

            return {
                'success': True,
                'message': 'Event clips captured',
                'file_paths': file_paths,
                'camera_ids_out': camera_ids_out
            }
        except Exception as e:
            self.get_logger().error(f'Error capturing event clips: {e}', exc_info=True)
            return {
                'success': False,
                'message': str(e),
                'file_paths': [],
                'camera_ids_out': []
            }

    def _prune_storage(self) -> None:
        """Ensure total storage size stays below max_total_size_mb by deleting oldest event dirs."""
        try:
            total_bytes = 0
            event_dirs: List[Tuple[Path, float, int]] = []

            for child in self.storage_root.iterdir():
                if not child.is_dir():
                    continue
                dir_mtime = child.stat().st_mtime
                dir_size = self._dir_size_bytes(child)
                total_bytes += dir_size
                event_dirs.append((child, dir_mtime, dir_size))

            max_bytes = self.max_total_size_mb * 1024 * 1024
            if total_bytes <= max_bytes:
                return

            # Sort by modification time (oldest first)
            event_dirs.sort(key=lambda x: x[1])

            for dir_path, _, dir_size in event_dirs:
                if total_bytes <= max_bytes:
                    break
                self.get_logger().info(f'Removing old event directory {dir_path} to free space')
                for root, _, files in os.walk(dir_path, topdown=False):
                    for name in files:
                        try:
                            os.remove(os.path.join(root, name))
                        except OSError:
                            pass
                    try:
                        os.rmdir(root)
                    except OSError:
                        pass
                total_bytes -= dir_size
        except Exception as e:
            self.get_logger().error(f'Error pruning storage: {e}', exc_info=True)

    @staticmethod
    def _dir_size_bytes(path: Path) -> int:
        size = 0
        for root, _, files in os.walk(path):
            for name in files:
                try:
                    size += os.path.getsize(os.path.join(root, name))
                except OSError:
                    pass
        return size

    def _wait_for_post_event_frames(self, end_time: float, post_s: float) -> None:
        """Wait a short time so post-event frames can arrive while executor keeps spinning."""
        now = time.time()
        wait_needed = end_time - now
        if wait_needed <= 0:
            return

        max_wait = min(self.capture_wait_ceiling_seconds, post_s + self.capture_wait_slack_seconds)
        wait_seconds = min(wait_needed, max_wait)

        if wait_seconds <= 0:
            return

        self.get_logger().info(
            f'Waiting {wait_seconds:.2f}s to gather post-event frames '
            f'(target end={end_time:.2f}, now={now:.2f})'
        )

        stop_time = time.time() + wait_seconds
        # Sleep in short slices so shutdown remains responsive
        while True:
            remaining = stop_time - time.time()
            if remaining <= 0:
                break
            time.sleep(min(0.5, remaining))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoRingbufferNode()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


