"""Notification provider abstraction for security events.

Phase 3 ships only StdoutProvider (logs what would be sent). Phase 4 will add
NtfyProvider and wire it in via NOTIFICATION_PROVIDER env var.

See DESIGN_ARRIVAL_PIPELINE.md §5.5.
"""
import logging
import os
from dataclasses import dataclass
from typing import Optional, Protocol


logger = logging.getLogger("notifications")


@dataclass
class ArrivalAlert:
    event_id: str
    class_label: str
    track_id: int
    confidence: float
    timestamp_iso: str
    frame_path: Optional[str] = None
    clip_path: Optional[str] = None


@dataclass
class NotificationResult:
    sent: bool
    provider: str
    reference: Optional[str] = None  # e.g., ntfy message id when implemented
    error: Optional[str] = None


class NotificationProvider(Protocol):
    """Anything that can deliver an arrival alert to the operator."""

    name: str

    def send(self, alert: ArrivalAlert, topic_or_destination: str) -> NotificationResult: ...


class StdoutProvider:
    """Logs the alert; doesn't actually notify. Used in Phase 3 + dev/test."""

    name = "stdout"

    def send(self, alert: ArrivalAlert, topic_or_destination: str) -> NotificationResult:
        logger.info(
            "[stdout-provider] WOULD-NOTIFY topic=%r event_id=%s class=%s track=%d conf=%.2f frame=%s clip=%s",
            topic_or_destination,
            alert.event_id,
            alert.class_label,
            alert.track_id,
            alert.confidence,
            alert.frame_path,
            alert.clip_path,
        )
        return NotificationResult(sent=True, provider=self.name, reference=f"stdout-{alert.event_id}")


def get_provider() -> NotificationProvider:
    """Factory: pick the provider.

    Resolution order: settings.yaml > env override > default ntfy. Env override
    is for ops/debug ("NOTIFICATION_PROVIDER=stdout docker restart web_app" to
    temporarily mute pushes without touching settings).
    """
    name = ""
    try:
        # Read settings.yaml's security_arrival.notifications.provider
        from .settings_service import get_settings
        notif = (get_settings().get("security_arrival") or {}).get("notifications") or {}
        name = (notif.get("provider") or "").strip().lower()
    except Exception as e:
        logger.warning("could not read provider from settings.yaml: %s", e)

    env_override = os.environ.get("NOTIFICATION_PROVIDER", "").strip().lower()
    if env_override:
        name = env_override

    if not name:
        name = "ntfy"  # default

    if name == "ntfy":
        from .ntfy_provider import NtfyProvider
        return NtfyProvider()
    if name == "stdout":
        return StdoutProvider()
    # Phase 4+ future: sipgate, telegram
    logger.warning("unknown notification provider=%r, falling back to stdout", name)
    return StdoutProvider()
