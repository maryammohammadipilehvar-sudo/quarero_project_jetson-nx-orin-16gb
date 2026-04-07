"""Helper class for loading sector configuration from YAML file."""

import os
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional
import traceback

try:
    from ament_index_python.packages import get_package_share_directory
    _AMENT_AVAILABLE = True
except ImportError:
    _AMENT_AVAILABLE = False


class SectorConfigLoader:
    """Helper class for loading sector configuration from YAML file."""
    
    @staticmethod
    def load_sectors(config_path: Optional[str] = None) -> List[Dict[str, Any]]:
        """Load sectors from YAML configuration file.
        
        Args:
            config_path: Optional path to the sector config YAML file.
                        If None, tries to find it using ament_index or relative to package.
        
        Returns:
            List of sector dictionaries, each containing:
            - name: str
            - x_min: float
            - x_max: float
            - y_min: float
            - y_max: float
            - priority: int
            - sector_type: str
        
        Raises:
            FileNotFoundError: If config file cannot be found
            ValueError: If config file is invalid or contains no sectors
        """
        # Determine config file path
        if config_path is None:
            # Try multiple methods to find the config file
            config_path = None
            
            # Method 1: Try using ament_index (ROS2 installed package)
            if _AMENT_AVAILABLE:
                try:
                    package_share = get_package_share_directory('control')
                    candidate = Path(package_share) / 'config' / 'sector_config.yaml'
                    if candidate.exists():
                        config_path = candidate
                except Exception:
                    pass  # Package not found via ament_index, try other methods
            
            # Method 2: Try relative to this file (development/source build)
            if config_path is None or not config_path.exists():
                current_file = Path(__file__).resolve()
                # Go up from sector_analysis/ to obstacle_avoidance/ to control/ to control/
                # sector_analysis -> obstacle_avoidance -> control -> control
                package_dir = current_file.parent.parent.parent.parent  # Go up to control/
                candidate = package_dir / 'config' / 'sector_config.yaml'
                if candidate.exists():
                    config_path = candidate
            
            # If still not found, use the relative path as fallback (will raise error if missing)
            if config_path is None or not config_path.exists():
                current_file = Path(__file__).resolve()
                package_dir = current_file.parent.parent.parent.parent
                config_path = package_dir / 'config' / 'sector_config.yaml'
        else:
            config_path = Path(config_path)
        
        # Check if file exists
        if not config_path.exists():
            raise FileNotFoundError(
                f"Sector config file not found: {config_path}\n"
                f"Current working directory: {os.getcwd()}\n"
                f"Config file absolute path: {config_path.resolve()}"
            )
        
        # Load YAML file
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            raise ValueError(
                f"Failed to parse YAML file {config_path}: {e}\n"
                f"{traceback.format_exc()}"
            )
        
        # Validate structure
        if not data:
            raise ValueError(f"Sector config file {config_path} is empty")
        
        if 'sectors' not in data:
            raise ValueError(
                f"Sector config file {config_path} missing 'sectors' key. "
                f"Found keys: {list(data.keys())}"
            )
        
        sectors = data['sectors']
        
        if not isinstance(sectors, list):
            raise ValueError(
                f"Sector config file {config_path}: 'sectors' must be a list, "
                f"got {type(sectors)}"
            )
        
        if len(sectors) == 0:
            raise ValueError(
                f"Sector config file {config_path}: 'sectors' list is empty"
            )
        
        # Validate and normalize each sector
        normalized_sectors = []
        for i, sector in enumerate(sectors):
            if not isinstance(sector, dict):
                raise ValueError(
                    f"Sector {i} in config file {config_path} must be a dictionary, "
                    f"got {type(sector)}"
                )
            
            # Required fields
            required_fields = ['name', 'x_min', 'x_max', 'y_min', 'y_max', 'priority', 'sector_type']
            missing_fields = [field for field in required_fields if field not in sector]
            if missing_fields:
                raise ValueError(
                    f"Sector {i} in config file {config_path} missing required fields: "
                    f"{missing_fields}"
                )
            
            # Normalize sector dictionary
            normalized_sector = {
                'name': str(sector['name']),
                'x_min': float(sector['x_min']),
                'x_max': float(sector['x_max']),
                'y_min': float(sector['y_min']),
                'y_max': float(sector['y_max']),
                'priority': int(sector['priority']),
                'sector_type': str(sector['sector_type'])
            }
            
            # Validate bounds
            if normalized_sector['x_min'] >= normalized_sector['x_max']:
                raise ValueError(
                    f"Sector {i} ('{normalized_sector['name']}'): "
                    f"x_min ({normalized_sector['x_min']}) must be < x_max ({normalized_sector['x_max']})"
                )
            
            if normalized_sector['y_min'] >= normalized_sector['y_max']:
                raise ValueError(
                    f"Sector {i} ('{normalized_sector['name']}'): "
                    f"y_min ({normalized_sector['y_min']}) must be < y_max ({normalized_sector['y_max']})"
                )
            
            normalized_sectors.append(normalized_sector)
        
        return normalized_sectors

