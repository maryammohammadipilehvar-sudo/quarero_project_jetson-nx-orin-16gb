#!/usr/bin/env bash
# analyze_wp_bag.sh — run the offline KPI analysis on a recorded bag.
#
# Usage:
#   ./analyze_wp_bag.sh <run_id>
#   e.g. ./analyze_wp_bag.sh wp_accuracy_test_20260518_141522
#
# Looks the run up under ~/gits/tactical_mower/ros2_ws/ros_bags/<run_id>/
# and runs analyze_wp_bag.py inside ros2_jetson against /app/ros2_ws/.

set -euo pipefail

if [ $# -ne 1 ]; then
  echo "usage: $0 <run_id>" >&2
  echo "example: $0 wp_accuracy_test_20260518_141522" >&2
  exit 2
fi

RUN="$1"
HOST_RUN="$HOME/gits/tactical_mower/ros2_ws/ros_bags/${RUN}"
CTR_RUN="/app/ros2_ws/ros_bags/${RUN}"

if [ ! -d "$HOST_RUN/bag" ]; then
  echo "no bag at $HOST_RUN/bag" >&2
  exit 3
fi

CONTAINER=ros2_jetson
ros2_env="source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install_ros2/setup.bash"

docker exec "$CONTAINER" bash -lc "
  $ros2_env && \
  python3 /app/ros2_ws/analyze_wp_bag.py '${CTR_RUN}'
"

echo
echo "[analyze] open: ${HOST_RUN}/analysis_report.md"
