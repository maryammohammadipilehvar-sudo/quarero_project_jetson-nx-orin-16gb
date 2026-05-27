"""Persistence layer for security events (metadata + index)."""

import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


# IMPORTANT: Keep this in sync with video_ringbuffer's storage_root param in
# system_bringup/launch/controller.launch.py — both containers must resolve to
# the same host directory. /routen/ is bind-mounted as /data/ in web_app and
# as /routen/ in ros2; the host path is ~/gits/routen/security_events/.
# Path candidate list mirrors utils/file_manager._BASE_DIR_CANDIDATES for
# consistency.
_EVENTS_DIR_CANDIDATES = [
    Path("/data/security_events"),    # web_app container mount of ~/gits/routen
    Path("/routen/security_events"),   # ros2 container mount of the same dir
]
EVENTS_DIR = next((p for p in _EVENTS_DIR_CANDIDATES if p.parent.exists()), _EVENTS_DIR_CANDIDATES[0])
EVENTS_DIR.mkdir(parents=True, exist_ok=True)


class EventRepository:
    """File-based storage for security events and their index."""

    def __init__(self, base_dir: Path = EVENTS_DIR, cache_ttl: float = 5.0):
        self._base_dir = base_dir
        self._base_dir.mkdir(parents=True, exist_ok=True)
        self._index_path = self._base_dir / "events_index.json"
        # Cache for index to reduce file I/O
        self._index_cache: Optional[List[Dict[str, Any]]] = None
        self._cache_timestamp: float = 0.0
        self._cache_ttl: float = cache_ttl  # Cache TTL in seconds
        self._cache_lock = threading.Lock()

    def _load_index_from_file(self) -> List[Dict[str, Any]]:
        """Load index from file (internal method, not cached)."""
        if not self._index_path.exists():
            return []
        try:
            with open(self._index_path, "r", encoding="utf-8") as f:
                return json.load(f) or []
        except Exception:
            return []

    def _load_index(self) -> List[Dict[str, Any]]:
        """Load index with caching support."""
        with self._cache_lock:
            now = time.time()
            # Check if cache is valid
            if self._index_cache is not None and (now - self._cache_timestamp) < self._cache_ttl:
                return self._index_cache.copy()
            
            # Cache expired or not set, load from file
            index = self._load_index_from_file()
            self._index_cache = index
            self._cache_timestamp = now
            return index.copy()

    def _invalidate_cache(self) -> None:
        """Invalidate the index cache."""
        with self._cache_lock:
            self._index_cache = None
            self._cache_timestamp = 0.0

    def _save_index(self, events: List[Dict[str, Any]]) -> None:
        with open(self._index_path, "w", encoding="utf-8") as f:
            json.dump(events, f, indent=2, ensure_ascii=False)

    def save_event(self, metadata: Dict[str, Any]) -> None:
        """Store per-event metadata and update index (newest first)."""
        event_id = metadata.get("event_id")
        if not event_id:
            raise ValueError("metadata must contain event_id")

        event_dir = self._base_dir / event_id
        event_dir.mkdir(parents=True, exist_ok=True)
        meta_path = event_dir / "metadata.json"

        try:
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(metadata, f, indent=2, ensure_ascii=False)
        except Exception as e:
            raise IOError(f"Failed to write metadata.json for event {event_id}: {e}") from e

        index = self._load_index()
        # Remove existing entry for this event_id, if any
        index = [e for e in index if e.get("event_id") != event_id]
        index.insert(0, {
            "event_id": event_id,
            "event_type": metadata.get("event_type"),
            "event_time": metadata.get("event_time"),
            "device_name": metadata.get("device_name"),
            "has_videos": bool(metadata.get("video_files")),
            "email_sent": bool(metadata.get("email_sent")),
            "class_label": metadata.get("class_label"),
        })
        try:
            self._save_index(index[:500])  # keep last 500
            self._invalidate_cache()  # Invalidate cache after save
        except Exception as e:
            raise IOError(f"Failed to save index for event {event_id}: {e}") from e

    def update_email_status(self, event_id: str, email_sent: bool) -> None:
        index = self._load_index()
        for item in index:
            if item.get("event_id") == event_id:
                item["email_sent"] = email_sent
                break
        self._save_index(index[:500])
        self._invalidate_cache()  # Invalidate cache after update

    def list_events(self,
                    from_iso: Optional[str] = None,
                    to_iso: Optional[str] = None,
                    event_type: Optional[str] = None,
                    device_name: Optional[str] = None,
                    limit: int = 100,
                    offset: int = 0) -> Dict[str, Any]:
        index = self._load_index()

        def parse_iso(s: Optional[str]) -> Optional[datetime]:
            if not s:
                return None
            try:
                return datetime.fromisoformat(s.replace("Z", "+00:00"))
            except Exception:
                return None

        from_dt = parse_iso(from_iso)
        to_dt = parse_iso(to_iso)

        filtered: List[Dict[str, Any]] = []
        for item in index:
            try:
                t = parse_iso(item.get("event_time"))
            except Exception:
                t = None

            if from_dt and t and t < from_dt:
                continue
            if to_dt and t and t > to_dt:
                continue
            if event_type and item.get("event_type") != event_type:
                continue
            if device_name and item.get("device_name") != device_name:
                continue
            filtered.append(item)

        total = len(filtered)
        sliced = filtered[offset: offset + limit]
        return {"total": total, "events": sliced}

    def load_event(self, event_id: str) -> Optional[Dict[str, Any]]:
        event_dir = self._base_dir / event_id
        meta_path = event_dir / "metadata.json"
        if not meta_path.exists():
            return None
        try:
            with open(meta_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data
        except Exception:
            return None

    def delete_event(self, event_id: str) -> bool:
        """Delete an event and its associated files (metadata + videos)."""
        try:
            event_dir = self._base_dir / event_id
            if not event_dir.exists():
                return False

            # Remove event directory and all its contents
            import shutil
            shutil.rmtree(event_dir)

            # Remove from index
            index = self._load_index()
            index = [e for e in index if e.get("event_id") != event_id]
            self._save_index(index[:500])
            self._invalidate_cache()  # Invalidate cache after delete

            return True
        except Exception as e:
            # Log error but don't raise - return False to indicate failure
            return False


