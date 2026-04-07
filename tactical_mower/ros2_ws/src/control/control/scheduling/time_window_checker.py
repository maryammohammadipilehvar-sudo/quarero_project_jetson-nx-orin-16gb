"""Time window checking for schedule triggers."""

from datetime import datetime, time as dt_time
from typing import Dict


class TimeWindowChecker:
    """Checks if current time is within schedule time windows."""
    
    @staticmethod
    def is_in_window(
        current_time: dt_time,
        start_time: dt_time,
        end_time: dt_time
    ) -> bool:
        """Check if current time is within time window.
        
        Handles wrap-around times (e.g., 22:00 to 06:00).
        
        Args:
            current_time: Current time
            start_time: Window start time
            end_time: Window end time
            
        Returns:
            True if in window, False otherwise
        """
        if start_time <= end_time:
            return start_time <= current_time <= end_time
        # Wrap-around case (e.g., 22:00 to 06:00)
        return current_time >= start_time or current_time <= end_time
    
    @staticmethod
    def is_schedule_active(schedule: Dict, current_weekday: int, current_time: dt_time) -> bool:
        """Check if schedule should be active now.
        
        Args:
            schedule: Schedule dictionary
            current_weekday: Current weekday (0=Monday, 6=Sunday)
            current_time: Current time
            
        Returns:
            True if schedule should be active, False otherwise
        """
        # Check if schedule is enabled
        if not schedule.get('active', False):
            return False
        
        # Check weekday
        if current_weekday not in schedule.get('weekdays', []):
            return False
        
        # Check time window
        try:
            start_time = datetime.strptime(schedule['start_time'], '%H:%M').time()
            end_time = datetime.strptime(schedule['end_time'], '%H:%M').time()
            return TimeWindowChecker.is_in_window(current_time, start_time, end_time)
        except (KeyError, ValueError):
            return False

