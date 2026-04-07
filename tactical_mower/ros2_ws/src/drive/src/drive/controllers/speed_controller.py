"""Speed controller for managing max speed settings."""

import yaml
from pathlib import Path
from typing import Optional


class SpeedController:
    """Manages maximum speed settings for the robot.
    
    Supports loading from settings file and runtime updates.
    """
    
    def __init__(self, default_max_speed: float = 1.0):
        """Initialize speed controller.
        
        Args:
            default_max_speed: Default maximum speed in m/s
        """
        self._max_speed = default_max_speed
        self._default_max_speed = default_max_speed
    
    def load_from_settings(self, settings_file: Path) -> bool:
        """Load max speed from settings YAML file.
        
        Args:
            settings_file: Path to settings.yaml file
            
        Returns:
            True if settings loaded successfully, False otherwise
        """
        if not settings_file.exists():
            return False
        
        try:
            with open(settings_file, 'r') as f:
                settings = yaml.safe_load(f)
                if settings and 'speed_factor' in settings:
                    self._max_speed = settings['speed_factor']
                    return True
        except Exception:
            pass
        
        return False
    
    def set_max_speed(self, max_speed: float):
        """Set maximum speed (runtime update).
        
        Args:
            max_speed: Maximum speed in m/s
        """
        if max_speed > 0:
            self._max_speed = max_speed
    
    def get_max_speed(self) -> float:
        """Get current maximum speed.
        
        Returns:
            Maximum speed in m/s
        """
        return self._max_speed
    
    def reset_to_default(self):
        """Reset to default maximum speed."""
        self._max_speed = self._default_max_speed

