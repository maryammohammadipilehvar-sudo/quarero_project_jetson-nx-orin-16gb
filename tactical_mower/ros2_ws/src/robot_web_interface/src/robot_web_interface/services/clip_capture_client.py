"""Thin wrapper around the CaptureEventClips ROS service."""

import time
from typing import List, Tuple

from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node

from interfaces.srv import CaptureEventClips


class ClipCaptureClient:
    def __init__(self, node: Node):
        self._node = node

    def capture(self,
                event_id: str,
                event_time: RosTime,
                camera_ids: List[str],
                pre_s: float,
                post_s: float) -> Tuple[bool, List[str], List[str]]:
        self._node.get_logger().info(
            f"ClipCaptureClient.capture called: event_id={event_id}, cameras={camera_ids}, "
            f"pre={pre_s}s, post={post_s}s"
        )
        
        client = self._node.create_client(CaptureEventClips, "capture_event_clips")
        if not client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn(
                f"capture_event_clips service not available for event {event_id}, storing event without videos"
            )
            return False, [], []

        req = CaptureEventClips.Request()
        req.event_id = event_id
        req.event_time = event_time
        req.camera_ids = list(camera_ids)
        req.pre_event_seconds = float(pre_s)
        req.post_event_seconds = float(post_s)

        self._node.get_logger().info(f"Calling capture_event_clips service for event {event_id}...")
        future = client.call_async(req)
        
        # Calculate timeout: pre + post + some buffer for processing
        # The service itself will wait for the post-event window, so we need to account for that
        timeout = pre_s + post_s + 15.0  # Extra buffer for video encoding
        start_time = time.time()
        
        # Wait for the future to complete (node is already spinning in main thread)
        # We poll the future instead of using spin_until_future_complete to avoid blocking the executor
        while not future.done():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self._node.get_logger().error(
                    f"capture_event_clips call timed out for event {event_id} after {timeout}s"
                )
                client.destroy()
                return False, [], []
            time.sleep(0.1)  # Small sleep to avoid busy-waiting
        
        try:
            resp: CaptureEventClips.Response = future.result()
            if not resp.success:
                self._node.get_logger().error(
                    f"capture_event_clips failed for event {event_id}: {resp.message}"
                )
            else:
                self._node.get_logger().info(
                    f"capture_event_clips succeeded for event {event_id}: "
                    f"{len(resp.file_paths)} clips created"
                )
            client.destroy()
            return bool(resp.success), list(resp.file_paths), list(resp.camera_ids_out)
        except Exception as e:
            self._node.get_logger().error(
                f"Exception getting capture_event_clips result for event {event_id}: {e}",
                exc_info=True
            )
            client.destroy()
            return False, [], []


