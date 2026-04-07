"""Schedule file management and persistence."""

import yaml
from pathlib import Path
from typing import Dict, Optional
from .schedule_validator import ScheduleValidator
from ..routes.route_manager import RouteManager


class ScheduleManager:
    """Manages schedule storage and loading from disk using a single YAML file."""
    
    def __init__(self, schedules_file: Path, route_manager: RouteManager):
        """Initialize schedule manager.
        
        Args:
            schedules_file: Path to schedules.yaml file (single file with all schedules)
            route_manager: Route manager for validation
        """
        self.schedules_file = Path(schedules_file).expanduser()
        self.validator = ScheduleValidator(route_manager)
        
        # Create parent directory if it doesn't exist
        self.schedules_file.parent.mkdir(parents=True, exist_ok=True)
    
    def _convert_to_internal_format(self, web_format: Dict) -> Dict:
        """Convert web app format to internal format.
        
        Web format: routes=[{route_name, repeat_count}], route_mode="sequential"/"random", loop=bool
        Internal format: route_names=[], route_repetitions=[], route_mode=0/1, loop_mode=bool
        """
        internal = web_format.copy()
        
        # Convert routes list to route_names and route_repetitions arrays
        if 'routes' in internal:
            routes = internal.pop('routes')
            internal['route_names'] = [r['route_name'] for r in routes]
            internal['route_repetitions'] = [r['repeat_count'] for r in routes]
        
        # Convert route_mode string to int
        if 'route_mode' in internal:
            route_mode = internal['route_mode']
            if isinstance(route_mode, str):
                internal['route_mode'] = 0 if route_mode.lower() == 'sequential' else 1
            elif isinstance(route_mode, int):
                # Already in correct format
                pass
        
        # Convert loop to loop_mode (web -> internal field name)
        if 'loop' in internal:
            internal['loop_mode'] = internal.pop('loop')
        
        return internal
    
    def _convert_to_web_format(self, internal_format: Dict) -> Dict:
        """Convert internal format to web app format.
        
        Internal format: route_names=[], route_repetitions=[], route_mode=0/1, loop_mode=bool
        Web format: routes=[{route_name, repeat_count}], route_mode="sequential"/"random", loop=bool
        """
        web = internal_format.copy()
        
        # Convert route_names and route_repetitions to routes list
        if 'route_names' in web and 'route_repetitions' in web:
            route_names = web.pop('route_names', [])
            route_repetitions = web.pop('route_repetitions', [])
            web['routes'] = [
                {'route_name': name, 'repeat_count': count}
                for name, count in zip(route_names, route_repetitions)
            ]
        
        # Convert route_mode int to string
        if 'route_mode' in web:
            route_mode = web['route_mode']
            if isinstance(route_mode, int):
                web['route_mode'] = 'sequential' if route_mode == 0 else 'random'
            elif isinstance(route_mode, str):
                # Already in correct format
                pass
        
        # Convert loop_mode to loop (internal -> web field name)
        if 'loop_mode' in web:
            web['loop'] = web.pop('loop_mode')
        
        return web
    
    def _load_file(self) -> Dict:
        """Load schedules.yaml file and return its contents."""
        if not self.schedules_file.exists():
            return {'schedules': []}
        
        try:
            with open(self.schedules_file, 'r') as f:
                data = yaml.safe_load(f) or {}
                if 'schedules' not in data:
                    data['schedules'] = []
                return data
        except Exception:
            return {'schedules': []}
    
    def _save_file(self, data: Dict) -> bool:
        """Save schedules.yaml file."""
        try:
            with open(self.schedules_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            return True
        except Exception:
            return False
    
    def save(self, schedule_data: Dict) -> bool:
        """Save schedule to disk.
        
        Args:
            schedule_data: Schedule dictionary (in internal format)
            
        Returns:
            True if saved successfully, False otherwise
        """
        try:
            data = self._load_file()
            schedules = data.get('schedules', [])
            
            # Convert to web format for storage
            web_format = self._convert_to_web_format(schedule_data)
            
            # Find and update existing schedule or add new one
            schedule_id = schedule_data['schedule_id']
            found = False
            for i, sched in enumerate(schedules):
                if sched.get('schedule_id') == schedule_id:
                    schedules[i] = web_format
                    found = True
                    break
            
            if not found:
                schedules.append(web_format)
            
            data['schedules'] = schedules
            return self._save_file(data)
        except Exception:
            return False
    
    def load(self, schedule_id: str) -> Optional[Dict]:
        """Load schedule from disk.
        
        Args:
            schedule_id: Schedule identifier
            
        Returns:
            Schedule dictionary (in internal format) or None if not found/invalid
        """
        try:
            data = self._load_file()
            schedules = data.get('schedules', [])
            
            for sched in schedules:
                if sched.get('schedule_id') == schedule_id:
                    # Convert to internal format
                    internal_format = self._convert_to_internal_format(sched)
                    # Validate
                    is_valid, _ = self.validator.validate(internal_format)
                    if is_valid:
                        return internal_format
            return None
        except Exception:
            return None
    
    def load_all(self) -> Dict[str, Dict]:
        """Load all schedules from disk.
        
        Returns:
            Dictionary mapping schedule_id to schedule data (in internal format)
        """
        schedules = {}
        
        try:
            data = self._load_file()
            for sched in data.get('schedules', []):
                schedule_id = sched.get('schedule_id')
                if schedule_id:
                    # Convert to internal format
                    internal_format = self._convert_to_internal_format(sched)
                    # Validate
                    is_valid, _ = self.validator.validate(internal_format)
                    if is_valid:
                        schedules[schedule_id] = internal_format
        except Exception:
            pass
        
        return schedules
    
    def delete(self, schedule_id: str) -> bool:
        """Delete schedule from disk.
        
        Args:
            schedule_id: Schedule identifier
            
        Returns:
            True if deleted successfully, False otherwise
        """
        try:
            data = self._load_file()
            schedules = data.get('schedules', [])
            
            # Remove schedule with matching ID
            original_count = len(schedules)
            schedules = [s for s in schedules if s.get('schedule_id') != schedule_id]
            
            if len(schedules) < original_count:
                data['schedules'] = schedules
                return self._save_file(data)
            
            return False
        except Exception:
            return False
