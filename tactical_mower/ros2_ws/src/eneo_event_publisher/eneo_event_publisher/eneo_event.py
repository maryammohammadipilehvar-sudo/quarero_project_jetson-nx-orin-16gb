"""Data class for Eneo camera events."""


class EneoEvent:
    """Data class for an Eneo event."""
    
    def __init__(self, event_type: str, event_time: str, event_action: str,
                 channels: list, device_name: str, ip_address: str, mac_address: str):
        self.event_type = event_type
        self.event_time = event_time
        self.event_action = event_action
        self.channels = channels
        self.device_name = device_name
        self.ip_address = ip_address
        self.mac_address = mac_address
    
    def __repr__(self):
        return f"EneoEvent({self.event_type}, {self.event_action}, {self.channels})"
    
    @property
    def is_fire(self) -> bool:
        return self.event_type == "FireDetect"
    
    @property
    def is_person(self) -> bool:
        return self.event_type == "PersonDetect"
    
    @property
    def is_motion(self) -> bool:
        return self.event_type == "MotionDetect"
    
    @property
    def is_start(self) -> bool:
        return self.event_action == "start"
    
    @property
    def is_stop(self) -> bool:
        return self.event_action == "stop"

