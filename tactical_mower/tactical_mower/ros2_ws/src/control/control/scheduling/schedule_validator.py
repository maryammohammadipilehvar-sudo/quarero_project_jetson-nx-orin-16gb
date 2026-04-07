"""Schedule validation logic."""

from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple
from ..routes.route_manager import RouteManager


class ScheduleValidator:
    """Validates schedule configuration data."""
    
    def __init__(self, route_manager: RouteManager):
        """Initialize validator.
        
        Args:
            route_manager: Route manager for checking routes
        """
        self.route_manager = route_manager
    
    def validate(self, schedule_data: Dict) -> Tuple[bool, List[str]]:
        """Validate schedule data.
        
        Args:
            schedule_data: Schedule dictionary to validate
            
        Returns:
            Tuple of (is_valid, error_messages)
        """
        errors = []
        
        # Check schedule ID
        if not schedule_data.get('schedule_id'):
            errors.append("Schedule ID is missing")
        
        # Validate time format
        try:
            datetime.strptime(schedule_data.get('start_time', ''), '%H:%M')
        except (ValueError, TypeError):
            errors.append("Invalid start_time format (expected HH:MM)")
        
        try:
            datetime.strptime(schedule_data.get('end_time', ''), '%H:%M')
        except (ValueError, TypeError):
            errors.append("Invalid end_time format (expected HH:MM)")
        
        # Validate weekdays
        weekdays = schedule_data.get('weekdays', [])
        if not weekdays or not isinstance(weekdays, list):
            errors.append("Weekdays must be a non-empty list")
        elif not all(0 <= d <= 6 for d in weekdays):
            errors.append("Weekdays must be values 0-6 (Monday=0, Sunday=6)")
        
        # Validate routes
        route_names = schedule_data.get('route_names', [])
        if not route_names:
            errors.append("No route names specified")
        else:
            # Validate route_repetitions
            route_repetitions = schedule_data.get('route_repetitions', [])
            if len(route_names) != len(route_repetitions):
                errors.append("Route names and repetitions length mismatch")
            
            if not all(rep > 0 for rep in route_repetitions):
                errors.append("All route repetitions must be > 0")
            
            # Validate route files exist
            for route_name in route_names:
                if not self.route_manager.route_exists(route_name):
                    errors.append(f"Route not found: {route_name}")
        
        return len(errors) == 0, errors

