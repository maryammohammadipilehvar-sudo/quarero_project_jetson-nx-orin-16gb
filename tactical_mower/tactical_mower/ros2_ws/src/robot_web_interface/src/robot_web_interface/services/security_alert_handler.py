"""Coordinates config, clip capture, repository, email, and WS broadcast for a SecurityAlert."""

import asyncio
import threading
import uuid
from datetime import datetime
from typing import List, Dict, Any

from rclpy.node import Node

from interfaces.msg import SecurityAlert

from .clip_capture_client import ClipCaptureClient
from .email_notifier import EmailNotifier, SecurityEmailSettings
from .event_repository import EventRepository
from .security_config import SecurityConfig
from ..utils.file_manager import load_settings


class SecurityAlertHandler:
    # Class-level tracking for timeouts (shared across all instances)
    _last_email_sent = {}  # {event_type: datetime}
    _last_video_capture = None  # datetime
    _timeout_lock = threading.Lock()
    
    def __init__(self, node: Node):
        self._node = node
        self._repo = EventRepository()

    def handle(self, msg: SecurityAlert) -> None:
        event_id = None
        try:
            self._node.get_logger().info(
                f"Received security alert: type={msg.event_type}, device={msg.device_name}, "
                f"time={msg.event_time.sec}.{msg.event_time.nanosec}"
            )
            
            settings = load_settings()
            email_settings = SecurityConfig.parse_email_settings(settings.get("security_email", {}))
            windows_cfg = settings.get("security_notification_windows", [])
            if not isinstance(windows_cfg, list):
                windows_cfg = []
            always_enable_types = settings.get("security_always_enable_event_types", [])
            if not isinstance(always_enable_types, list):
                always_enable_types = []
            defaults = settings.get("security_event_defaults", {})
            pre_s = float(defaults.get("pre_event_seconds", 10.0))
            post_s = float(defaults.get("post_event_seconds", 10.0))
            
            # Get timeout settings
            email_timeout = settings.get("security_email_notification_timeout_seconds", 60)
            video_timeout = settings.get("security_video_capture_timeout_seconds", 60)

            # Determine event time as datetime (ROS time may be zero)
            if msg.event_time.sec or msg.event_time.nanosec:
                event_dt = datetime.utcfromtimestamp(
                    float(msg.event_time.sec) + float(msg.event_time.nanosec) * 1e-9
                )
            else:
                event_dt = datetime.utcnow()

            # Check if email should be sent based on time windows and event type
            event_type = msg.event_type or ""
            should_send = SecurityConfig.should_send_email(event_type, event_dt, windows_cfg, always_enable_types)
            
            # Check email notification timeout
            if should_send:
                with SecurityAlertHandler._timeout_lock:
                    last_email_time = SecurityAlertHandler._last_email_sent.get(event_type)
                    if last_email_time:
                        time_since_last = (event_dt - last_email_time).total_seconds()
                        if time_since_last < email_timeout:
                            should_send = False
                            self._node.get_logger().info(
                                f"Email notification skipped for {event_type} due to timeout "
                                f"({time_since_last:.1f}s < {email_timeout}s)"
                            )

            event_id = str(uuid.uuid4())
            self._node.get_logger().info(f"Generated event_id={event_id} for device={msg.device_name}")

            camera_ids: List[str] = list(msg.camera_ids)
            # Initial metadata without video files so the event appears immediately in the UI
            metadata = {
                "event_id": event_id,
                "event_type": msg.event_type,
                "event_time": event_dt.isoformat() + "Z",
                "device_name": msg.device_name,
                "description": msg.description,
                "camera_ids": camera_ids,
                "video_files": [],
                "video_capture_success": False,
                "email_sent": False,
                "email_error": "",
                "within_window": should_send,  # Keep for backwards compatibility
                "created_at": datetime.utcnow().isoformat() + "Z",
            }

            # Persist event immediately (without video) so it shows up on the Alarme page
            self._node.get_logger().info(f"Saving event {event_id} to repository...")
            self._repo.save_event(metadata)
            self._node.get_logger().info(f"Event {event_id} saved successfully")

            # Send email immediately if enabled and should send based on windows/event type (before video capture)
            if should_send and email_settings.enabled:
                notifier = EmailNotifier(email_settings)
                subject = f"[Robot] Security alert: {msg.event_type} ({msg.device_name})"
                body_lines = [
                    f"Security alert on device: {msg.device_name}",
                    f"Type: {msg.event_type}",
                    f"Time (UTC): {metadata['event_time']}",
                    f"Description: {msg.description}",
                    "",
                    f"Event ID: {event_id}",
                    f"Cameras to be recorded: {', '.join(camera_ids) if camera_ids else 'None'}",
                    "",
                    "Video clips are being captured and will be available shortly.",
                    "Open the event review page to see more details and videos.",
                ]
                ok, err = notifier.send_security_alert(subject, "\n".join(body_lines))
                metadata["email_sent"] = ok
                metadata["email_error"] = "" if ok else err
                
                # Update last email sent time if email was successfully sent
                if ok:
                    with SecurityAlertHandler._timeout_lock:
                        SecurityAlertHandler._last_email_sent[event_type] = event_dt
                
                # Update persisted data with email status
                self._repo.save_event(metadata)
                self._repo.update_email_status(event_id, ok)
                self._node.get_logger().info(
                    f"Email sent for event {event_id}: {ok}" + (f" (error: {err})" if not ok else "")
                )

            # Broadcast lightweight summary to all WebSocket clients so the UI can update immediately
            self._broadcast_new_event({
                "event_id": event_id,
                "event_type": metadata["event_type"],
                "event_time": metadata["event_time"],
                "device_name": metadata["device_name"],
                "has_videos": False,
                "email_sent": metadata.get("email_sent", False),
            })

            # Check video capture timeout
            should_capture_video = True
            with SecurityAlertHandler._timeout_lock:
                if SecurityAlertHandler._last_video_capture:
                    time_since_last = (event_dt - SecurityAlertHandler._last_video_capture).total_seconds()
                    if time_since_last < video_timeout:
                        should_capture_video = False
                        self._node.get_logger().info(
                            f"Video capture skipped for event {event_id} due to timeout "
                            f"({time_since_last:.1f}s < {video_timeout}s)"
                        )
                        # Update metadata to indicate video capture was skipped
                        metadata["video_capture_success"] = False
                        metadata["video_capture_skipped"] = True
                        self._repo.save_event(metadata)
            
            # Start video capture in background thread to avoid blocking the ROS2 executor
            # This allows new security alerts to be processed immediately
            if should_capture_video:
                # Update last video capture time
                with SecurityAlertHandler._timeout_lock:
                    SecurityAlertHandler._last_video_capture = event_dt
                
                capture_thread = threading.Thread(
                    target=self._capture_videos_async,
                    args=(event_id, msg.event_time, camera_ids, pre_s, post_s),
                    daemon=True
                )
                capture_thread.start()
                self._node.get_logger().info(f"Started background video capture thread for event {event_id}")
            else:
                self._node.get_logger().info(f"Video capture skipped for event {event_id} due to timeout")

            self._node.get_logger().info(
                f"Security event {event_id} stored (type={msg.event_type}, device={msg.device_name})"
            )
        except Exception as e:
            self._node.get_logger().error(
                f"Error handling security alert (event_id={event_id}, device={msg.device_name if 'msg' in locals() else 'unknown'}): {e}",
                exc_info=True
            )

    def _capture_videos_async(self, event_id: str, event_time, camera_ids: List[str],
                              pre_s: float, post_s: float) -> None:
        """Capture videos in background thread and update event metadata when done."""
        try:
            self._node.get_logger().info(f"Background thread: Starting video capture for event {event_id}")
            
            # Capture clips (this may take some time due to pre/post windows)
            clip_client = ClipCaptureClient(self._node)
            success, file_paths, camera_ids_out = clip_client.capture(
                event_id=event_id,
                event_time=event_time,
                camera_ids=camera_ids,
                pre_s=pre_s,
                post_s=post_s,
            )

            # Load current metadata
            current_metadata = self._repo.load_event(event_id)
            if current_metadata is None:
                self._node.get_logger().error(f"Event {event_id} not found when updating video files")
                return

            current_metadata["video_files"] = [
                {"camera_id": cid, "path": path} for cid, path in zip(camera_ids_out, file_paths)
            ]
            current_metadata["video_capture_success"] = success
            # Update stored metadata with video information
            self._repo.save_event(current_metadata)
            self._node.get_logger().info(
                f"Background thread: Updated event {event_id} with {len(file_paths)} video files"
            )
        except Exception as e:
            self._node.get_logger().error(
                f"Error in background video capture thread for event {event_id}: {e}",
                exc_info=True
            )

    def _broadcast_new_event(self, event_summary: Dict[str, Any]) -> None:
        """Send a 'security_event' message via the shared ConnectionManager to all WS clients."""
        try:
            connection_manager = getattr(self._node, "connection_manager", None)
            event_loop = getattr(self._node, "event_loop", None)

            if connection_manager is None or event_loop is None or not event_loop.is_running():
                return

            async def send():
                await connection_manager.broadcast({
                    "type": "security_event",
                    "data": event_summary,
                })

            asyncio.run_coroutine_threadsafe(send(), event_loop)
        except Exception as e:
            self._node.get_logger().error(f"Error broadcasting security_event: {e}", exc_info=True)


