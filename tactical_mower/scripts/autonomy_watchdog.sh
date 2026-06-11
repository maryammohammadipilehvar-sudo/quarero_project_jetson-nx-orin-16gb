#!/bin/bash
# Watchdog: keeps the autonomy_supervisor alive. If it dies, restart it within 2s.
# Defensive: on watchdog exit, resume all autonomy nodes (safety net).

NODES="livox_obstacle_node simple_obstacle_detector livox_ros_driver2_node tactical_wp_follower nav2_navigation_node"

resume_all() {
  for PAT in $NODES; do
    for PID in $(pgrep -f "$PAT" 2>/dev/null); do
      kill -CONT "$PID" 2>/dev/null
    done
  done
}

trap 'echo "[$(date +%T)] watchdog exiting -- defensive resume"; resume_all; exit 0' EXIT INT TERM HUP

source /opt/ros/jazzy/setup.bash 2>/dev/null
source /app/ros2_ws/install_ros2/setup.bash 2>/dev/null

while true; do
  echo "[$(date +%T)] starting autonomy_supervisor.py"
  python3 /usr/local/bin/autonomy_supervisor.py
  rc=$?
  echo "[$(date +%T)] supervisor exited (rc=$rc) — restarting in 2s"
  resume_all   # always ensure nodes alive before next attempt
  sleep 2
done
