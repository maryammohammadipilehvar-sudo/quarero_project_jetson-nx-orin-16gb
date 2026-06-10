#!/bin/bash
# Watchdog for joy_controller. Restarts if it dies. Ensures /dev/esp_joy symlink.

trap 'exit 0' EXIT INT TERM HUP

while true; do
  # Refresh symlink (docker may have lost it on container restart)
  ln -sf /dev/ttyUSB0 /dev/esp_joy 2>/dev/null

  echo "[$(date +%T)] starting joy_controller"
  source /opt/ros/jazzy/setup.bash 2>/dev/null
  source /app/ros2_ws/install_ros2/setup.bash 2>/dev/null

  /app/ros2_ws/install_ros2/control/lib/control/joy_controller \
    --ros-args -r __node:=joy_controller \
    --params-file /app/ros2_ws/install_ros2/control/share/control/config/params.yaml

  rc=$?
  echo "[$(date +%T)] joy_controller exited (rc=$rc) — restart in 3s"
  sleep 3
done
