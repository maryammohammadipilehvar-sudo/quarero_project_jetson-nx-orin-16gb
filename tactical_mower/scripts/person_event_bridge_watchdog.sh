#!/bin/bash
# Watchdog: keeps person_event_bridge.py running.
trap 'exit 0' EXIT INT TERM HUP
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /app/ros2_ws/install_ros2/setup.bash 2>/dev/null
while true; do
    echo "[$(date +%T)] starting person_event_bridge.py"
    python3 /usr/local/bin/person_event_bridge.py
    rc=$?
    echo "[$(date +%T)] person_event_bridge exited (rc=$rc) — restart in 3s"
    sleep 3
done
