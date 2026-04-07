"""Security events API endpoints."""

from datetime import datetime
from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse
from typing import Optional
from pathlib import Path

from ...services.security_service import (
    list_events,
    load_event,
    delete_event,
    get_security_settings,
    save_security_settings,
)

router = APIRouter(prefix="/api/events", tags=["events"])


@router.get("")
async def get_events(
    from_time: Optional[str] = None,
    to_time: Optional[str] = None,
    event_type: Optional[str] = None,
    device_name: Optional[str] = None,
    limit: int = 100,
    offset: int = 0,
):
    """List stored security events with optional filters."""
    try:
        return list_events(
            from_iso=from_time,
            to_iso=to_time,
            event_type=event_type,
            device_name=device_name,
            limit=limit,
            offset=offset,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{event_id}")
async def get_event_detail(event_id: str):
    """Get full metadata for a single event."""
    data = load_event(event_id)
    if not data:
        raise HTTPException(status_code=404, detail="Event not found")
    return data


@router.get("/{event_id}/video/{camera_id}")
async def get_event_video(event_id: str, camera_id: str):
    """Serve the recorded video clip for a given camera.
    
    FileResponse is optimized for efficient file serving and automatically
    handles Range requests for video seeking. Uses efficient chunk sizes (typically 64KB-1MB)
    for much better performance than manual streaming.
    """
    data = load_event(event_id)
    if not data:
        raise HTTPException(status_code=404, detail="Event not found")

    video_files = data.get("video_files", [])
    for vf in video_files:
        if vf.get("camera_id") == camera_id:
            path = vf.get("path")
            if not path:
                break
            p = Path(path)
            if not p.exists():
                break
            
            # Generate filename with timestamp and camera name
            # Format: YYYY_MM_DD-HH_MM-camera_id.ext (matches frontend expectation)
            event_time_str = data.get("event_time", "")
            try:
                # Parse ISO format timestamp (e.g., "2025-12-15T10:30:00Z")
                # Replace Z with +00:00 for proper timezone handling
                normalized_time = event_time_str.replace("Z", "+00:00")
                event_dt = datetime.fromisoformat(normalized_time)
                date_str = event_dt.strftime("%Y_%m_%d")
                time_str = event_dt.strftime("%H_%M")
            except Exception:
                # Fallback to current date/time if parsing fails
                now = datetime.utcnow()
                date_str = now.strftime("%Y_%m_%d")
                time_str = now.strftime("%H_%M")
            
            # Get file extension from original file
            file_ext = p.suffix if p.suffix else ".mp4"
            
            # Create filename: "YYYY_MM_DD-HH_MM-camera_id.ext"
            download_filename = f"{date_str}-{time_str}-{camera_id}{file_ext}"
            
            # FileResponse automatically handles:
            # - Range requests (HTTP 206 Partial Content) for video seeking
            # - Efficient chunked streaming with optimal buffer sizes
            # - Proper Content-Type and Content-Disposition headers
            # - Memory-efficient file serving
            return FileResponse(
                path=str(p),
                filename=download_filename,
                media_type="video/mp4",
                headers={
                    "Accept-Ranges": "bytes",
                }
            )

    raise HTTPException(status_code=404, detail="Video not found for this camera")


@router.get("/settings/security")
async def get_security_settings_endpoint():
    """Expose security-related settings for frontend (email + windows)."""
    try:
        return get_security_settings()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/settings/security")
async def save_security_settings_endpoint(payload: dict):
    """Save security-related settings."""
    import logging
    logger = logging.getLogger("robot_web_interface")
    
    try:
        logger.info(f"Saving security settings: email_enabled={payload.get('email', {}).get('enabled')}, "
                   f"smtp_host={payload.get('email', {}).get('smtp_host')}")
        save_security_settings(payload)
        logger.info("Security settings saved successfully")
        return {"status": "success", "message": "Sicherheits-Einstellungen erfolgreich gespeichert"}
    except Exception as e:
        logger.error(f"Error saving security settings: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/{event_id}")
async def delete_event_endpoint(event_id: str):
    """Delete an event and all its associated files."""
    try:
        success = delete_event(event_id)
        if not success:
            raise HTTPException(status_code=404, detail="Event not found")
        return {"status": "success", "message": f"Event {event_id} deleted"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


