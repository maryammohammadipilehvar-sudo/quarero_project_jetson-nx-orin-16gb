# Security alert event from camera or sensor system
string event_type         # Event type: "person", "fire", "motion", etc.
builtin_interfaces/Time event_time  # Timestamp when event occurred
string device_name        # Camera/device identifier: "eneo_thermal", "eneo_rgb", "realsense", etc.
string description        # Optional human-readable description of the event
string[] camera_ids       # Optional list of camera IDs involved in the event (for video recording)


