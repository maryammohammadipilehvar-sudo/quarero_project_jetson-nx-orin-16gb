"""ntfy.sh push-notification provider.

Implements NotificationProvider protocol from notifications.py.
Uses stdlib urllib only — no `requests` dep, no Docker image rebuild.

Per DESIGN_ARRIVAL_PIPELINE.md §5.5: priority=max bypasses silent/DND, Tags
add visual cues, Click jumps the operator's phone (via NetBird) straight to
the event in the web UI. Attach + Actions deferred to a v2 polish (require
either public reachability for ntfy.sh to fetch the image, or self-hosted
ntfy — see §11 Security & GDPR).
"""
import json
import logging
import os
import urllib.error
import urllib.request
from datetime import datetime
from typing import Optional

try:
    # Python 3.9+ stdlib
    from zoneinfo import ZoneInfo
    _HAS_ZONEINFO = True
except ImportError:
    _HAS_ZONEINFO = False

from .notifications import ArrivalAlert, NotificationResult


logger = logging.getLogger("notifications.ntfy")

# Emoji per class label (German first-line UX touch)
EMOJI_BY_CLASS = {
    "person": "🚶",
    "car": "🚗",
    "truck": "🚚",
    "bus": "🚌",
    "motorcycle": "🏍",
    "bicycle": "🚲",
    "test": "🔔",
}


class NtfyProvider:
    """POSTs an arrival alert to ntfy.sh/{topic} with priority=max.

    Reads server URL + optional auth token + public base URL (for Click links)
    from env at init time. The topic is supplied per-call by the caller (read
    from settings.yaml so the operator can rotate via the web UI).
    """

    name = "ntfy"

    def __init__(
        self,
        server: Optional[str] = None,
        auth_token: Optional[str] = None,
        public_base_url: Optional[str] = None,
        timeout_s: float = 5.0,
    ):
        self.server = (server or os.environ.get("NTFY_SERVER", "https://ntfy.sh")).rstrip("/")
        self.auth_token = (auth_token or os.environ.get("NTFY_AUTH_TOKEN", "")).strip()
        self.public_base_url = (
            public_base_url or os.environ.get("NOTIFICATION_BASE_URL", "")
        ).strip().rstrip("/")
        self.timeout_s = timeout_s
        logger.info(
            "NtfyProvider ready: server=%s click_base=%s auth=%s",
            self.server,
            self.public_base_url or "<none>",
            "set" if self.auth_token else "<none>",
        )

    def send(self, alert: ArrivalAlert, topic: str) -> NotificationResult:
        topic = (topic or "").strip()
        if not topic:
            return NotificationResult(sent=False, provider=self.name, error="topic is empty")

        # Use ntfy's JSON publish API (POST to server root) — natively supports UTF-8
        # in title/body. Avoids the latin-1 HTTP-header encoding bug that bites
        # when emojis end up in Title.
        url = self.server  # POST to root with JSON body specifying topic
        emoji = EMOJI_BY_CLASS.get(alert.class_label.lower(), "🚨")

        # Format the timestamp in the operator's local timezone (default Europe/Berlin).
        # Override via env NOTIFICATION_DISPLAY_TZ (e.g. "Europe/Paris", "UTC").
        # The stored timestamp_iso is always UTC — this is just for display.
        ts_short = "?"
        if alert.timestamp_iso:
            try:
                ts_utc = datetime.fromisoformat(alert.timestamp_iso.replace("Z", "+00:00"))
                if _HAS_ZONEINFO:
                    tz_name = os.environ.get("NOTIFICATION_DISPLAY_TZ", "Europe/Berlin")
                    ts_local = ts_utc.astimezone(ZoneInfo(tz_name))
                    ts_short = ts_local.strftime("%Y-%m-%d %H:%M:%S")
                else:
                    ts_short = ts_utc.strftime("%Y-%m-%d %H:%M:%S UTC")
            except Exception:
                ts_short = alert.timestamp_iso[:19].replace("T", " ")

        conf_pct = int(round(alert.confidence * 100)) if alert.confidence else 0
        title = f"{emoji} Sicherheitswarnung — {alert.class_label} erkannt"
        message = (
            f"{alert.class_label.capitalize()} am {ts_short} (Konfidenz {conf_pct}%).\n"
            f"Track-ID: {alert.track_id}. Ereignis: {alert.event_id}"
        )

        body_obj = {
            "topic": topic,
            "title": title,
            "message": message,
            "priority": 5,                          # max
            "tags": ["rotating_light", "warning"],
        }
        if self.public_base_url:
            body_obj["click"] = f"{self.public_base_url}/events?focus={alert.event_id}"

        headers = {"Content-Type": "application/json"}
        if self.auth_token:
            headers["Authorization"] = f"Bearer {self.auth_token}"

        body_bytes = json.dumps(body_obj).encode("utf-8")

        try:
            req = urllib.request.Request(url, data=body_bytes, headers=headers, method="POST")
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                code = resp.getcode()
                body_resp = resp.read(2048).decode("utf-8", errors="replace")
        except urllib.error.HTTPError as e:
            return NotificationResult(sent=False, provider=self.name, error=f"HTTP {e.code} {e.reason}")
        except urllib.error.URLError as e:
            return NotificationResult(sent=False, provider=self.name, error=f"URL error: {e.reason}")
        except Exception as e:
            return NotificationResult(sent=False, provider=self.name, error=f"{type(e).__name__}: {e}")

        if not (200 <= code < 300):
            return NotificationResult(
                sent=False, provider=self.name,
                error=f"HTTP {code}: {body_resp[:200]}"
            )

        # ntfy returns a JSON message envelope with `id` on success
        msg_id = "ok"
        try:
            msg_id = json.loads(body_resp).get("id", "ok")
        except Exception:
            pass

        logger.info(
            "ntfy notification sent: topic=%s event_id=%s msg_id=%s",
            topic, alert.event_id, msg_id,
        )
        return NotificationResult(sent=True, provider=self.name, reference=msg_id)
