#!/usr/bin/env bash
# Script bricht bei Fehlern ab
set -e

# Name des Containers
CONTAINER_NAME="ros2_jetson"

# Vollqualifizierter Pfad zu docker (systemd hat oft einen minimalen PATH)
DOCKER_BIN="/usr/bin/docker"

# Container starten, falls er gestoppt ist
# (Fehler ignorieren, wenn er bereits läuft oder nicht existiert)
"$DOCKER_BIN" start "$CONTAINER_NAME" >/dev/null 2>&1 || true

# ROS2-Launch-File im Container starten (OHNE -t, kein interaktives TTY!)
"$DOCKER_BIN" exec "$CONTAINER_NAME" bash -lc '
  # ROS2-Umgebung in Container sourcen
  source /opt/ros/jazzy/setup.bash
  source /ros2_ws/install/setup.bash 2>/dev/null || true

  # Launch-File starten
  ros2 launch system_bringup controller.launch.py 
'
