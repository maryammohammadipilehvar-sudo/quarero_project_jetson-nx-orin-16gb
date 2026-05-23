"""72-hour retention service for security_events.

Walks /data/security_events/, deletes every event whose metadata.json
`created_at` is older than retention_hours (default 72), and appends a one-line
audit entry per deletion to deletion_audit.log. The audit log is kept
indefinitely (no PII — only event_id, timestamps, counts) to prove GDPR
compliance.

Two ways to run:
  1. As an asyncio task inside the web_app FastAPI process (wired in main.py
     lifespan). Re-reads retention_hours from settings.yaml on every tick so
     the operator can change it via the UI without restarting.
  2. As a one-shot CLI for dry-run / ad-hoc cleanup:
       python3 -m robot_web_interface.services.retention --dry-run
       python3 -m robot_web_interface.services.retention --retention-hours 0.01

See DESIGN_ARRIVAL_PIPELINE.md §5.6.
"""
import asyncio
import json
import logging
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional


logger = logging.getLogger("retention")

# Files in the root that are NOT events (must not be deleted by retention)
_NON_EVENT_NAMES = {"events_index.json", "deletion_audit.log"}


def walk_and_delete(
    root: Path,
    retention_hours: float,
    dry_run: bool = False,
    audit_log_path: Optional[Path] = None,
) -> dict:
    """Sweep root/ for expired events. Idempotent — safe to re-run.

    Returns a dict of counts + the list of event_ids that were (or would be)
    deleted.
    """
    now = datetime.now(timezone.utc)
    cutoff_ts = now.timestamp() - retention_hours * 3600.0

    stats = {
        "scanned": 0,
        "deleted": 0,
        "would_delete": 0,
        "skipped_no_meta": 0,
        "skipped_bad_meta": 0,
        "skipped_too_young": 0,
        "errors": 0,
        "deleted_event_ids": [],
    }

    if not root.exists():
        logger.warning("retention root %s does not exist", root)
        return stats

    for entry in sorted(root.iterdir()):
        if entry.name in _NON_EVENT_NAMES:
            continue
        if not entry.is_dir():
            continue
        stats["scanned"] += 1
        meta_path = entry / "metadata.json"

        # If no metadata.json, don't delete — could be a partial write or an
        # event from before metadata was a thing. Skip safely.
        if not meta_path.exists():
            stats["skipped_no_meta"] += 1
            logger.debug("skip %s: no metadata.json", entry.name)
            continue

        try:
            meta = json.loads(meta_path.read_text())
        except Exception as e:
            stats["skipped_bad_meta"] += 1
            logger.warning("skip %s: bad metadata.json: %s", entry.name, e)
            continue

        ts_str = meta.get("created_at") or meta.get("event_time")
        if not ts_str:
            stats["skipped_bad_meta"] += 1
            continue

        try:
            ts = datetime.fromisoformat(ts_str.replace("Z", "+00:00"))
        except Exception:
            stats["skipped_bad_meta"] += 1
            continue

        age_h = (now.timestamp() - ts.timestamp()) / 3600.0
        if ts.timestamp() >= cutoff_ts:
            stats["skipped_too_young"] += 1
            continue

        # Eligible for deletion
        if dry_run:
            stats["would_delete"] += 1
            stats["deleted_event_ids"].append(entry.name)
            logger.info("[dry-run] would delete %s (age %.2fh)", entry.name, age_h)
            continue

        files_count = sum(1 for _ in entry.iterdir())
        try:
            # 1. Update events_index.json (atomic via temp + replace) BEFORE
            #    removing the dir, so a crash leaves an orphaned dir (which a
            #    future sweep can clean) rather than a phantom index entry.
            idx_path = root / "events_index.json"
            if idx_path.exists():
                try:
                    idx = json.loads(idx_path.read_text()) or []
                    idx = [e for e in idx if e.get("event_id") != entry.name]
                    tmp = idx_path.with_suffix(".json.tmp")
                    tmp.write_text(json.dumps(idx, indent=2))
                    tmp.replace(idx_path)
                except Exception as e:
                    logger.warning("could not update index for %s: %s", entry.name, e)

            # 2. Remove the event dir.
            shutil.rmtree(entry)
            stats["deleted"] += 1
            stats["deleted_event_ids"].append(entry.name)

            # 3. Audit log (no PII — event_id + counts only).
            if audit_log_path:
                try:
                    audit_log_path.parent.mkdir(parents=True, exist_ok=True)
                    audit_entry = {
                        "deleted_at": now.isoformat(),
                        "event_id": entry.name,
                        "class_label": meta.get("class_label"),
                        "age_hours": round(age_h, 2),
                        "files_deleted_count": files_count,
                        "retention_hours": retention_hours,
                    }
                    with audit_log_path.open("a") as f:
                        f.write(json.dumps(audit_entry) + "\n")
                except Exception as e:
                    logger.warning("could not append audit for %s: %s", entry.name, e)

            logger.info("deleted %s (age %.2fh, %d files)", entry.name, age_h, files_count)
        except Exception as e:
            stats["errors"] += 1
            logger.error("failed to delete %s: %s", entry.name, e, exc_info=True)

    return stats


async def retention_loop(
    root: Path,
    get_retention_hours: Callable[[], float],
    audit_log_path: Path,
    interval_s: float = 3600.0,
):
    """Run walk_and_delete every interval_s seconds. Designed to be a long-lived
    asyncio task started from FastAPI lifespan.

    get_retention_hours is called once per tick so settings.yaml updates are
    picked up without restarting web_app.
    """
    logger.info(
        "retention loop started: root=%s interval=%.0fs",
        root, interval_s,
    )
    while True:
        try:
            retention_hours = float(get_retention_hours())
            if retention_hours <= 0:
                logger.warning(
                    "retention_hours <= 0 (%s) — skipping this tick (set security_arrival.retention_hours in settings.yaml)",
                    retention_hours,
                )
            else:
                stats = await asyncio.to_thread(
                    walk_and_delete,
                    root,
                    retention_hours,
                    False,
                    audit_log_path,
                )
                # Only log noisy summary when something actually happened
                if stats["deleted"] > 0 or stats["errors"] > 0:
                    logger.info("retention sweep: %s", stats)
                else:
                    logger.debug("retention sweep clean: scanned=%d", stats["scanned"])
        except asyncio.CancelledError:
            logger.info("retention loop cancelled — shutting down")
            raise
        except Exception as e:
            logger.error("retention sweep exception: %s", e, exc_info=True)
        await asyncio.sleep(interval_s)


# ─── CLI entrypoint (python3 -m robot_web_interface.services.retention) ────
def _main():
    import argparse
    p = argparse.ArgumentParser(description="72h retention sweep for security_events")
    p.add_argument("--root", default="/data/security_events",
                   help="security_events root dir (container path)")
    p.add_argument("--retention-hours", type=float, default=None,
                   help="override hours (default: read from settings.yaml)")
    p.add_argument("--dry-run", action="store_true",
                   help="list what would be deleted, don't actually delete")
    p.add_argument("--audit-log", default=None,
                   help="path to deletion_audit.log (default: <root>/deletion_audit.log)")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    retention_hours = args.retention_hours
    if retention_hours is None:
        # Read from settings.yaml — same candidate list as file_manager.py
        import yaml
        for sp in (Path("/data/settings/settings.yaml"), Path("/routen/settings/settings.yaml")):
            if sp.exists():
                try:
                    s = yaml.safe_load(sp.read_text()) or {}
                    retention_hours = float(
                        (s.get("security_arrival") or {}).get("retention_hours", 72)
                    )
                    break
                except Exception:
                    pass
        if retention_hours is None:
            retention_hours = 72.0

    audit_log = Path(args.audit_log) if args.audit_log else Path(args.root) / "deletion_audit.log"
    root = Path(args.root)

    print(f"config: root={root} retention_hours={retention_hours} dry_run={args.dry_run} audit={audit_log}")
    stats = walk_and_delete(root, retention_hours, args.dry_run, audit_log)
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    _main()
