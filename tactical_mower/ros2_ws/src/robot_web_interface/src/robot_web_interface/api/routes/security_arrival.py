"""POST /api/security/arrival — receives arrival events from Jetson 2.

On a valid POST:
  1. Validate Bearer token (env ARRIVAL_INGEST_TOKEN).
  2. Validate payload shape (minimal — internal endpoint).
  3. Compute event_id (idempotent: same source+track+minute → same id).
  4. Pre-create the event dir + save frame.jpg + write metadata.json
     (so the dir exists with sane ownership BEFORE ringbuffer writes the clip).
  5. Call /capture_event_clips synchronously (off the event loop) — blocks
     pre+post+15s on the worker thread.
  6. Update metadata with clip status.
  7. Enqueue notification dispatch (Phase 3 = stdout provider).
  8. Return {accepted, event_id, actions}.

See DESIGN_ARRIVAL_PIPELINE.md §5.3.
"""
import asyncio
import base64
import hmac
import json
import logging
import os
import re
import secrets
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

try:
    from zoneinfo import ZoneInfo
    _HAS_ZONEINFO = True
except ImportError:
    _HAS_ZONEINFO = False

from builtin_interfaces.msg import Time as RosTime
from fastapi import APIRouter, HTTPException, Request, status
from fastapi.concurrency import run_in_threadpool

from ...ros_interface.robot_node import RobotNode
from ...services.clip_capture_client import ClipCaptureClient
from ...services.event_repository import EventRepository
from ...services.notifications import ArrivalAlert, get_provider
from ...services.settings_service import get_settings


router = APIRouter(prefix="/api/security", tags=["security"])
log = logging.getLogger("security_arrival")

# Injected by main.py
_ros_node: Optional[RobotNode] = None
_clip_client: Optional[ClipCaptureClient] = None
_event_repo: Optional[EventRepository] = None
_notif_provider = None
_ingest_token: str = ""

# Idempotency cache: (source, track_id, ts_minute) → event_id, monotonic_t
_IDEMPOTENCY_TTL_S = 120.0
# ARRIVAL_COOLDOWN_PATCH_v1 module-level cooldown state
# Per-class cooldown for the heavy ringbuffer capture call. Idempotency
# alone is not enough - a busy detector reuses class_label "person" with
# fresh track_ids every few seconds, bypassing the (source, track_id, minute)
# key. The cooldown is class-wide and only gates the capture step; the
# event metadata and the operator notification still go out.
_CAPTURE_COOLDOWN_S = 60.0
_last_capture_by_class: dict[str, float] = {}
_idempotency_cache: dict[tuple, tuple[str, float]] = {}


def init_security_arrival_router(node: RobotNode):
    """Wire dependencies. Called from main.py lifespan."""
    global _ros_node, _clip_client, _event_repo, _notif_provider, _ingest_token
    _ros_node = node
    _clip_client = ClipCaptureClient(node)
    _event_repo = EventRepository()
    _notif_provider = get_provider()
    _ingest_token = os.environ.get("ARRIVAL_INGEST_TOKEN", "").strip()
    if not _ingest_token:
        log.warning(
            "ARRIVAL_INGEST_TOKEN env var is empty — /api/security/arrival will reject ALL requests. "
            "Set it via tactical_mower/.env (loaded by docker-compose) and restart web_app."
        )
    else:
        log.info("security_arrival router ready; provider=%s, token=set(%d chars)",
                 _notif_provider.name, len(_ingest_token))


def _auth(request: Request):
    """Constant-time Bearer-token check. Raises 401 on mismatch."""
    if not _ingest_token:
        raise HTTPException(status_code=503, detail="ingest token not configured on server")
    auth = request.headers.get("authorization", "")
    if not auth.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="missing bearer token")
    presented = auth[len("Bearer "):].strip()
    if not hmac.compare_digest(presented, _ingest_token):
        raise HTTPException(status_code=401, detail="invalid token")


def _idempotency_key(source: str, track_id: int, ts_iso: str) -> tuple:
    # Bucket by minute so retries within a minute collapse
    minute = ts_iso[:16] if len(ts_iso) >= 16 else ts_iso
    return (source, int(track_id), minute)


def _check_idempotency(key: tuple) -> Optional[str]:
    """Return existing event_id if we've seen this key recently. Also evicts expired entries."""
    now = time.monotonic()
    # Evict expired
    expired = [k for k, (_, t) in _idempotency_cache.items() if now - t > _IDEMPOTENCY_TTL_S]
    for k in expired:
        del _idempotency_cache[k]
    entry = _idempotency_cache.get(key)
    if entry is None:
        return None
    return entry[0]


def _remember_idempotency(key: tuple, event_id: str):
    _idempotency_cache[key] = (event_id, time.monotonic())


_HHMM_RE = re.compile(r"^([01]\d|2[0-3]):([0-5]\d)$")


def _hhmm_to_minutes(s: str) -> Optional[int]:
    """Parse 'HH:MM' to minute-of-day, or None if invalid/empty."""
    if not s:
        return None
    m = _HHMM_RE.match(s.strip())
    if not m:
        return None
    return int(m.group(1)) * 60 + int(m.group(2))


def _is_in_quiet_hours(quiet_from: str, quiet_to: str, tz_name: str = "Europe/Berlin") -> bool:
    """True if local-now is within the [quiet_from, quiet_to) window.

    Both empty → no quiet window → always False (notifications fire 24/7).
    Window CAN wrap midnight (e.g. from='17:00' to='09:00' → quiet all night).
    """
    start = _hhmm_to_minutes(quiet_from)
    end = _hhmm_to_minutes(quiet_to)
    if start is None or end is None:
        return False
    if start == end:
        return False  # degenerate empty window
    if _HAS_ZONEINFO:
        now = datetime.now(ZoneInfo(tz_name))
    else:
        now = datetime.now()  # local-ish; better than UTC
    now_min = now.hour * 60 + now.minute
    if start < end:
        return start <= now_min < end
    # wraps midnight
    return now_min >= start or now_min < end


@router.post("/arrival")
async def receive_arrival(request: Request):
    """Receive an arrival event from Jetson 2's arrival_detection.py."""
    _auth(request)

    try:
        payload = await request.json()
    except Exception:
        raise HTTPException(status_code=400, detail="body is not valid JSON")

    # Minimal validation — defensive, this is an internal endpoint
    required = ["source", "timestamp", "track_id", "class_label", "class_id", "confidence", "bbox", "frame_jpeg_base64"]
    missing = [k for k in required if k not in payload]
    if missing:
        raise HTTPException(status_code=400, detail=f"missing fields: {missing}")

    source = str(payload["source"])
    track_id = int(payload["track_id"])
    ts_iso = str(payload["timestamp"])
    class_label = str(payload["class_label"])
    class_id = int(payload["class_id"])
    confidence = float(payload["confidence"])
    bbox = payload["bbox"]
    frame_b64 = payload["frame_jpeg_base64"]
    camera = str(payload.get("camera", "unknown"))

    # Idempotency
    key = _idempotency_key(source, track_id, ts_iso)
    existing = _check_idempotency(key)
    if existing is not None:
        log.info("idempotent: returning existing event_id=%s for key=%s", existing, key)
        return {"accepted": True, "event_id": existing, "actions": ["idempotent_replay"]}

    # Generate event_id
    try:
        ts_dt = datetime.fromisoformat(ts_iso.replace("Z", "+00:00"))
    except Exception:
        ts_dt = datetime.now(timezone.utc)
    event_id = f"evt_{ts_dt.strftime('%Y%m%dT%H%M%S')}_{track_id}"

    log.info("arrival received: event_id=%s class=%s track=%d source=%s", event_id, class_label, track_id, source)

    # Pre-create event dir + frame.jpg + initial metadata so the dir exists with
    # web_app's ownership before ringbuffer drops the clip into it.
    event_dir = _event_repo._base_dir / event_id
    event_dir.mkdir(parents=True, exist_ok=True)
    frame_path = event_dir / "frame.jpg"
    try:
        frame_path.write_bytes(base64.b64decode(frame_b64))
    except Exception as e:
        log.error("frame.jpg decode/write failed for %s: %s", event_id, e)
        raise HTTPException(status_code=400, detail=f"frame decode failed: {e}")

    settings = get_settings()
    arrival_cfg = (settings.get("security_arrival") or {})
    retention_h = float(arrival_cfg.get("retention_hours", 72))
    pre_s = float(arrival_cfg.get("clip_pre_seconds", 10.0))
    post_s = float(arrival_cfg.get("clip_post_seconds", 10.0))
    expires_iso = (datetime.now(timezone.utc).timestamp() + retention_h * 3600.0)

    metadata = {
        "event_id": event_id,
        "event_type": "arrival",
        "event_time": ts_iso,
        "device_name": camera,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "expires_at": datetime.fromtimestamp(expires_iso, tz=timezone.utc).isoformat(),
        "source": source,
        "class_label": class_label,
        "class_id": class_id,
        "track_id": track_id,
        "confidence": confidence,
        "bbox": bbox,
        "camera": camera,
        "frame_file": "frame.jpg",
        "clip_status": "pending",
        "notification_status": "pending",
        "video_files": [],
    }
    _event_repo.save_event(metadata)
    _remember_idempotency(key, event_id)

    actions = ["frame_saved", "metadata_saved"]

    # 1. Dispatch notification IMMEDIATELY (synchronous but fast, ~500ms).
    #    Operator's phone rings within ~3-5s of detection instead of ~2 min.
    #    Suppressed during quiet hours (event still captured + logged; phone
    #    stays silent until quiet window ends). SMTP fallback for quiet hours
    #    is a future enhancement.
    try:
        notif_cfg = arrival_cfg.get("notifications") or {}
        notif_topic = notif_cfg.get("topic", "")
        notif_enabled = bool(notif_cfg.get("enabled", False))
        quiet_from = notif_cfg.get("quiet_hours_from", "") or ""
        quiet_to = notif_cfg.get("quiet_hours_to", "") or ""
        in_quiet = _is_in_quiet_hours(quiet_from, quiet_to)

        if not notif_enabled:
            metadata["notification_status"] = "disabled"
            actions.append("notification_skipped:disabled")
        elif not notif_topic:
            metadata["notification_status"] = "no_topic"
            actions.append("notification_skipped:no_topic")
        elif in_quiet:
            metadata["notification_status"] = "suppressed_quiet_hours"
            metadata["notification_quiet_window"] = f"{quiet_from}-{quiet_to}"
            actions.append("notification_skipped:quiet_hours")
            log.info(
                "notification suppressed for %s — in quiet window %s-%s (event still captured)",
                event_id, quiet_from, quiet_to,
            )
        else:
            alert = ArrivalAlert(
                event_id=event_id,
                class_label=class_label,
                track_id=track_id,
                confidence=confidence,
                timestamp_iso=ts_iso,
                frame_path=str(frame_path),
                clip_path=None,  # not yet — clip capture runs in background
            )
            result = _notif_provider.send(alert, notif_topic)
            metadata["notification_status"] = "sent" if result.sent else "failed"
            metadata["notification_provider"] = result.provider
            metadata["notification_reference"] = result.reference
            metadata["notification_error"] = result.error
            actions.append("notification_dispatched")
        _event_repo.save_event(metadata)
    except Exception as e:
        log.error("notification dispatch exception for %s: %s", event_id, e)
        metadata["notification_status"] = "error"
        metadata["notification_error"] = str(e)
        _event_repo.save_event(metadata)

    # 2. Fire clip capture in BACKGROUND — don't block the response.
    #    asyncio.create_task runs concurrently with subsequent requests.
    # ARRIVAL_COOLDOWN_PATCH_v1 per-class cooldown gate
    _cd_now = time.monotonic()
    _cd_last = _last_capture_by_class.get(class_label, 0.0)
    if (_cd_now - _cd_last) < _CAPTURE_COOLDOWN_S:
        _cd_remaining = _CAPTURE_COOLDOWN_S - (_cd_now - _cd_last)
        log.info("arrival %s: clip capture skipped - class %s in cooldown for %.1fs more", event_id, class_label, _cd_remaining)
        metadata["clip_status"] = "cooldown_skipped"
        metadata["clip_cooldown_remaining_s"] = round(_cd_remaining, 1)
        _event_repo.save_event(metadata)
        actions.append(f"clip_capture_skipped:cooldown_{int(_cd_remaining)}s")
    else:
        _last_capture_by_class[class_label] = _cd_now
        asyncio.create_task(_capture_clip_background(event_id, ts_dt, pre_s, post_s))
        actions.append("clip_capture_queued")

    return {"accepted": True, "event_id": event_id, "actions": actions}


async def _capture_clip_background(event_id: str, ts_dt: datetime, pre_s: float, post_s: float):
    """Run ringbuffer capture in a background task. Updates metadata when done.

    Side benefit: ringbuffer's empty-file_paths bug (returns success=True with no
    files when one camera fails) is worked around by scanning the event dir for
    actual .mp4 files after capture.
    """
    rt = RosTime()
    rt.sec = int(ts_dt.timestamp())
    rt.nanosec = int((ts_dt.timestamp() % 1) * 1e9)

    log.info("[background] starting clip capture for %s (pre=%.1fs post=%.1fs)", event_id, pre_s, post_s)
    try:
        success, file_paths, camera_ids_out = await run_in_threadpool(
            _clip_client.capture,
            event_id,
            rt,
            ["eneo_rgb", "eneo_thermal"],
            pre_s,
            post_s,
        )
    except Exception as e:
        log.error("[background] clip capture exception for %s: %s", event_id, e)
        meta = _event_repo.load_event(event_id) or {}
        meta["clip_status"] = "error"
        meta["clip_error"] = str(e)
        _event_repo.save_event(meta)
        return

    # Always derive video_files from what's actually on disk in the event_dir
    # — ringbuffer returns paths in its OWN container's view (/routen/...) but
    # web_app sees the same files as /data/.... FileResponse runs in web_app so
    # we need web_app's view. event_dir IS /data/security_events/{event_id}/
    # so glob results are already in the correct form.
    event_dir = _event_repo._base_dir / event_id
    on_disk = sorted(event_dir.glob("*.mp4"))
    video_files = [{"camera_id": p.stem, "path": str(p)} for p in on_disk]

    if not video_files and file_paths:
        log.warning(
            "[background] ringbuffer reported %d file_paths but disk scan finds 0 for %s",
            len(file_paths), event_id,
        )

    meta = _event_repo.load_event(event_id) or {}
    meta["clip_status"] = "captured" if (success and video_files) else ("failed" if not success else "no_clip")
    meta["video_files"] = video_files
    meta["camera_ids_captured"] = [vf["camera_id"] for vf in video_files]
    _event_repo.save_event(meta)
    log.info("[background] clip capture done for %s: status=%s files=%d",
             event_id, meta["clip_status"], len(video_files))


@router.post("/notifications/test")
async def test_notification(force: bool = False):
    """Fire one test notification using current settings topic. No event recorded.

    UI sprint #5: now honors quiet hours by default — if the operator's set a
    quiet window and current time falls inside it, returns sent=False with
    suppressed_reason='quiet_hours'. Operator can override by passing
    ?force=true (UI shows a "Trotzdem senden?" prompt in that case).
    """
    settings = get_settings()
    arrival_cfg = (settings.get("security_arrival") or {})
    notif_cfg = arrival_cfg.get("notifications") or {}
    topic = (notif_cfg.get("topic") or "").strip()
    if not topic:
        raise HTTPException(status_code=400, detail="topic not set; rotate first")

    quiet_from = (notif_cfg.get("quiet_hours_from") or "").strip()
    quiet_to = (notif_cfg.get("quiet_hours_to") or "").strip()
    in_quiet = _is_in_quiet_hours(quiet_from, quiet_to)
    if in_quiet and not force:
        return {
            "sent": False,
            "suppressed_reason": "quiet_hours",
            "quiet_window": f"{quiet_from}-{quiet_to}",
            "topic": topic,
            "hint": "Sende mit ?force=true um trotzdem zu testen.",
        }

    alert = ArrivalAlert(
        event_id="test_notification",
        class_label="test",
        track_id=0,
        confidence=1.0,
        timestamp_iso=datetime.now(timezone.utc).isoformat(),
        frame_path=None,
        clip_path=None,
    )
    result = _notif_provider.send(alert, topic)

    # Persist the test outcome so the UI's connectivity hint can read it.
    # Uses the settings-yaml helpers from settings.py via a late import (avoid
    # circular import at module load time).
    try:
        from .settings import _write_notif_block
        _write_notif_block({
            "last_test_at": datetime.now(timezone.utc).isoformat(),
            "last_test_success": bool(result.sent),
            "last_test_error": result.error or "",
            "last_test_forced": bool(force and in_quiet),
        })
    except Exception as e:
        log.warning("could not persist last_test_* state: %s", e)

    return {
        "sent": result.sent,
        "provider": result.provider,
        "reference": result.reference,
        "error": result.error,
        "topic": topic,
        "forced": bool(force and in_quiet),
    }


def generate_topic() -> str:
    """32-char URL-safe random topic, prefixed for legibility in logs."""
    return f"quarero-arrival-{secrets.token_urlsafe(16)}"
