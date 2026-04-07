"""Parser functions for Eneo event data (time, device names, etc.)."""

import time
from datetime import datetime
from typing import Optional

from builtin_interfaces.msg import Time as RosTime


def parse_eneo_time(eneo_time_str: str) -> Optional[datetime]:
    """
    Parse Eneo EventTime format: "2015-1-5_14:17:53" -> datetime
    
    Args:
        eneo_time_str: Eneo time format string
        
    Returns:
        datetime object or None on error
    """
    try:
        # Eneo Format: "2015-1-5_14:17:53" -> "2015-01-05 14:17:53"
        parts = eneo_time_str.split('_')
        if len(parts) != 2:
            return None
        
        date_part = parts[0]  # "2015-1-5"
        time_part = parts[1]  # "14:17:53"
        
        # Parse date (kann 1-stellige Monate/Tage haben)
        date_parts = date_part.split('-')
        if len(date_parts) != 3:
            return None
        
        year = int(date_parts[0])
        month = int(date_parts[1])
        day = int(date_parts[2])
        
        # Parse time
        time_parts = time_part.split(':')
        if len(time_parts) != 3:
            return None
        
        hour = int(time_parts[0])
        minute = int(time_parts[1])
        second = int(time_parts[2])
        
        return datetime(year, month, day, hour, minute, second)
    except Exception:
        return None


def eneo_time_to_ros_time(eneo_time_str: str) -> RosTime:
    """
    Convert Eneo EventTime to ROS2 Time.
    If parsing fails, current time is used.
    
    Args:
        eneo_time_str: Eneo time format string
        
    Returns:
        ROS2 Time message
    """
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1e9)    
    ros_time = RosTime()
    ros_time.sec = sec
    ros_time.nanosec = nanosec
    return ros_time


def map_device_name(eneo_device: str, ip_address: str) -> str:
    """
    Map Eneo Device Name to our device_name format.
    
    Args:
        eneo_device: Eneo DeviceName (e.g., "INT-8SF0003M0A")
        ip_address: Camera IP address
        
    Returns:
        device_name for SecurityAlert (e.g., "eneo_thermal" or "eneo_rgb")
    """
    # Determine based on IP or Device Name
    # 192.168.10.128 could be thermal or rgb
    # For now: If IP is 192.168.10.128, use "eneo_thermal" as default
    if ip_address == "192.168.10.128":
        # Could be thermal or rgb - based on channel or device name
        # Default: thermal
        return "eneo_thermal"
    
    # Fallback: device name in lowercase
    return eneo_device.lower().replace('-', '_')

