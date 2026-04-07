"""Mapping constants for Eneo event types to SecurityAlert event_type."""

# Mapping from Eneo event types to SecurityAlert event_type
ENEO_TO_SECURITY_TYPE = {
    "FireDetect": "fire",
    "PersonDetect": "PD_VD",
    "MotionDetect": "motion",
    "FaceDetect": "face",
    "VehicleDetect": "vehicle",
    "LineCross": "line_cross",
    "RegionEnter": "region_enter",
    "RegionExit": "region_exit",
}

