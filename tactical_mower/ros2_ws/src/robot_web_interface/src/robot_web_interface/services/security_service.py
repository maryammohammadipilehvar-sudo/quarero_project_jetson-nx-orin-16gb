"""Thin facade module exposing security functions used by the rest of the codebase.

Behind this facade, implementation lives in smaller, focused modules:
- security_config.SecurityConfig
- event_repository.EventRepository
- email_notifier.EmailNotifier
- clip_capture_client.ClipCaptureClient
- security_alert_handler.SecurityAlertHandler
"""

from typing import Any, Dict, Optional

from rclpy.node import Node

from interfaces.msg import SecurityAlert

from .event_repository import EventRepository
from .security_alert_handler import SecurityAlertHandler
from .security_config import SecurityConfig


def get_security_settings() -> Dict[str, Any]:
    """Return security-related settings (email + notification windows)."""
    return SecurityConfig.get()


def save_security_settings(data: Dict[str, Any]) -> None:
    """Persist security-related settings."""
    SecurityConfig.save(data)


def handle_security_alert(node: Node, msg: SecurityAlert) -> None:
    """
    Entry point from ROS subscriber: capture event clips via service, save metadata, send email.
    This function is synchronous and intended to run in the ROS executor thread.
    """
    handler = SecurityAlertHandler(node)
    handler.handle(msg)


def list_events(from_iso: Optional[str] = None, to_iso: Optional[str] = None,
                event_type: Optional[str] = None, device_name: Optional[str] = None,
                limit: int = 100, offset: int = 0) -> Dict[str, Any]:
    """Return filtered list of events for API."""
    repo = EventRepository()
    return repo.list_events(
        from_iso=from_iso,
        to_iso=to_iso,
        event_type=event_type,
        device_name=device_name,
        limit=limit,
        offset=offset,
    )


def load_event(event_id: str) -> Optional[Dict[str, Any]]:
    """Load full metadata for a single event."""
    repo = EventRepository()
    return repo.load_event(event_id)


def delete_event(event_id: str) -> bool:
    """Delete an event and all its associated files."""
    repo = EventRepository()
    return repo.delete_event(event_id)


