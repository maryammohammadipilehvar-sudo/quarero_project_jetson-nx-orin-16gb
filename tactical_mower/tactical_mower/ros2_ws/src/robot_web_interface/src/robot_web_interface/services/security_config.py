"""Access to security-related settings in settings.yaml (email, time windows, pre/post seconds)."""

from dataclasses import dataclass
from datetime import datetime, time as dt_time
from typing import Any, Dict, List

from ..utils.file_manager import load_settings, save_settings


@dataclass
class SecurityEmailSettings:
    enabled: bool = False
    smtp_host: str = ""
    smtp_port: int = 587
    use_tls: bool = True
    username: str = ""
    password: str = ""
    sender: str = ""
    recipients: List[str] = None


class SecurityConfig:
    """Static helpers to read/write security settings."""

    @staticmethod
    def parse_email_settings(data: Dict[str, Any]) -> SecurityEmailSettings:
        recipients = data.get("recipients") or []
        if isinstance(recipients, str):
            recipients = [r.strip() for r in recipients.split(",") if r.strip()]
        return SecurityEmailSettings(
            enabled=bool(data.get("enabled", False)),
            smtp_host=str(data.get("smtp_host", "")),
            smtp_port=int(data.get("smtp_port", 587)),
            use_tls=bool(data.get("use_tls", True)),
            username=str(data.get("username", "")),
            password=str(data.get("password", "")),
            sender=str(data.get("sender", "")),
            recipients=recipients,
        )

    @staticmethod
    def get() -> Dict[str, Any]:
        settings = load_settings()
        security_email = settings.get("security_email", {})
        # New structure: list of windows, or migrate from old structure
        notification_windows = settings.get("security_notification_windows", [])
        if isinstance(notification_windows, dict):
            # Old structure - return empty list, will need to be migrated by user
            notification_windows = []
        elif not isinstance(notification_windows, list):
            notification_windows = []
        
        event_defaults = settings.get("security_event_defaults", {})
        always_enable_event_types = settings.get("security_always_enable_event_types", [])
        email_notification_timeout = settings.get("security_email_notification_timeout_seconds", 60)
        video_capture_timeout = settings.get("security_video_capture_timeout_seconds", 60)

        email_cfg = SecurityConfig.parse_email_settings(security_email)

        return {
            "email": {
                "enabled": email_cfg.enabled,
                "smtp_host": email_cfg.smtp_host,
                "smtp_port": email_cfg.smtp_port,
                "use_tls": email_cfg.use_tls,
                "username": email_cfg.username,
                "password": "",  # never expose actual password
                "sender": email_cfg.sender,
                "recipients": email_cfg.recipients,
            },
            "notification_windows": notification_windows,
            "event_defaults": {
                "pre_event_seconds": float(event_defaults.get("pre_event_seconds", 10.0)),
                "post_event_seconds": float(event_defaults.get("post_event_seconds", 10.0)),
            },
            "always_enable_event_types": always_enable_event_types if isinstance(always_enable_event_types, list) else [],
            "email_notification_timeout_seconds": int(email_notification_timeout),
            "video_capture_timeout_seconds": int(video_capture_timeout),
        }

    @staticmethod
    def save(data: Dict[str, Any]) -> None:
        settings = load_settings()
        email_data = data.get("email", {})

        current_email = settings.get("security_email", {})
        # Only update password if a new one was provided (not empty)
        password = email_data.get("password", "")
        if not password:
            password = current_email.get("password", "")

        settings["security_email"] = {
            "enabled": bool(email_data.get("enabled", False)),
            "smtp_host": str(email_data.get("smtp_host", "")),
            "smtp_port": int(email_data.get("smtp_port", 587)),
            "use_tls": bool(email_data.get("use_tls", True)),
            "username": str(email_data.get("username", "")),
            "password": password,
            "sender": str(email_data.get("sender", "")),
            "recipients": email_data.get("recipients", []),
        }

        # New structure: list of windows
        notification_windows = data.get("notification_windows", [])
        if isinstance(notification_windows, list):
            settings["security_notification_windows"] = notification_windows
        else:
            settings["security_notification_windows"] = []

        event_defaults = data.get("event_defaults", {})
        settings["security_event_defaults"] = {
            "pre_event_seconds": float(event_defaults.get("pre_event_seconds", 10.0)),
            "post_event_seconds": float(event_defaults.get("post_event_seconds", 10.0)),
        }

        always_enable_event_types = data.get("always_enable_event_types", [])
        if isinstance(always_enable_event_types, list):
            settings["security_always_enable_event_types"] = always_enable_event_types
        else:
            settings["security_always_enable_event_types"] = []

        # Save timeout settings (0-300 seconds)
        email_notification_timeout = data.get("email_notification_timeout_seconds", 60)
        video_capture_timeout = data.get("video_capture_timeout_seconds", 60)
        settings["security_email_notification_timeout_seconds"] = max(0, min(300, int(email_notification_timeout)))
        settings["security_video_capture_timeout_seconds"] = max(0, min(300, int(video_capture_timeout)))

        save_settings(settings)

    @staticmethod
    def should_send_email(event_type: str, event_time: datetime, windows_cfg: List[Dict[str, Any]], 
                          always_enable_types: List[str]) -> bool:
        """
        Check if an email should be sent for the given event type and time.
        
        Args:
            event_type: Type of event (e.g., "person", "fire")
            event_time: Event timestamp (UTC datetime)
            windows_cfg: List of notification window configurations
            always_enable_types: List of event types that should always send emails
        
        Returns:
            True if email should be sent, False otherwise
        """
        # Check if event type is always enabled
        if event_type.lower() in [t.lower() for t in always_enable_types]:
            return True
        
        # Check if any active window matches the event time and type
        weekday_names = ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]
        current_weekday = weekday_names[event_time.weekday()]
        current_t = event_time.time()
        
        for window in windows_cfg:
            # Skip inactive windows
            if not window.get("active", False):
                continue
            
            # Check if this window applies to this event type
            enabled_types = window.get("enabled_event_types", [])
            if not isinstance(enabled_types, list):
                enabled_types = []
            if event_type.lower() not in [t.lower() for t in enabled_types]:
                continue
            
            # Parse window times
            start_day = window.get("start_day", "").lower()
            start_time_str = window.get("start_time", "")
            end_day = window.get("end_day", "").lower()
            end_time_str = window.get("end_time", "")
            
            if not all([start_day, start_time_str, end_day, end_time_str]):
                continue
            
            try:
                start_parts = [int(x) for x in start_time_str.split(":")]
                end_parts = [int(x) for x in end_time_str.split(":")]
                start_t = dt_time(hour=start_parts[0], minute=start_parts[1])
                end_t = dt_time(hour=end_parts[0], minute=end_parts[1])
            except Exception:
                continue
            
            # Calculate if current time is within the window
            start_weekday_idx = weekday_names.index(start_day) if start_day in weekday_names else -1
            end_weekday_idx = weekday_names.index(end_day) if end_day in weekday_names else -1
            current_weekday_idx = weekday_names.index(current_weekday)
            
            if start_weekday_idx == -1 or end_weekday_idx == -1:
                continue
            
            # Check if we're in the same day window
            if start_weekday_idx == end_weekday_idx:
                if current_weekday_idx == start_weekday_idx:
                    if start_t <= end_t:
                        # Same-day window
                        if start_t <= current_t <= end_t:
                            return True
                    else:
                        # Overnight window on same day (shouldn't happen, but handle it)
                        if current_t >= start_t or current_t <= end_t:
                            return True
            else:
                # Multi-day window (e.g., Monday 22:00 - Tuesday 06:00)
                start_day_idx = start_weekday_idx
                end_day_idx = end_weekday_idx
                
                # Check if window wraps around the week (e.g., Sunday 22:00 - Monday 06:00)
                if end_day_idx < start_day_idx:
                    # Window wraps around week boundary
                    # Check if we're on start day with time >= start_time
                    if current_weekday_idx == start_day_idx:
                        if current_t >= start_t:
                            return True
                    # Check if we're on end day with time <= end_time
                    elif current_weekday_idx == end_day_idx:
                        if current_t <= end_t:
                            return True
                    # Check if we're in the days after start (to end of week, e.g., Sunday->Monday)
                    elif current_weekday_idx > start_day_idx:
                        # Any time on these days is in the window
                        return True
                    # Check if we're in the days before end (from start of week, e.g., Monday->Sunday)
                    elif current_weekday_idx < end_day_idx:
                        # Any time on these days is in the window
                        return True
                else:
                    # Normal multi-day window (e.g., Monday 22:00 - Wednesday 06:00)
                    # Check if we're on start day with time >= start_time
                    if current_weekday_idx == start_day_idx:
                        if current_t >= start_t:
                            return True
                    # Check if we're on end day with time <= end_time
                    elif current_weekday_idx == end_day_idx:
                        if current_t <= end_t:
                            return True
                    # Check if we're in the days between start and end
                    elif start_day_idx < current_weekday_idx < end_day_idx:
                        # Any time on these days is in the window
                        return True
        
        return False


