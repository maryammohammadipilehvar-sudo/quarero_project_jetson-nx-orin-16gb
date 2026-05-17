#!/bin/bash
# After re-IPing to .226, restart containers so they bind to the new host IP cleanly.
# Run this AFTER you SSH back in to quarero02@192.168.10.226.

set -e

echo "[post-swap] Verifying we are on .226..."
if ! ip -4 addr show | grep -q "192.168.10.226"; then
  echo "!!! ABORT: this machine is NOT on 192.168.10.226. Re-run swap_ip.sh first."
  ip -4 addr show
  exit 1
fi
echo "[post-swap] OK, IP is .226."
echo ""

echo "[post-swap] Waiting up to 60s for colcon build inside ros2_jetson to finish (if still running)..."
for i in $(seq 1 30); do
  if ! docker exec ros2_jetson pgrep -x colcon >/dev/null 2>&1; then
    echo "[post-swap] colcon not running — safe to restart."
    break
  fi
  echo "[post-swap] colcon still busy ($((i*2))s/60s)..."
  sleep 2
done
echo ""

echo "[post-swap] Restarting containers (motors may twitch — operator + e-stop required per CLAUDE.md §1)..."
cd ~/gits/livox_mid_360_obstacle_detection && docker compose restart
cd ~/gits/RealsenseD_camera_Obstacle_avoidance && docker compose restart
cd ~/gits/tactical_mower && docker compose restart   # web_app + ros2_jetson

echo ""
echo "[post-swap] Done. Check:"
echo "  docker ps"
echo "  docker logs --tail 50 livox_ros2_jazzy   # should pass wait_for_lidar now"
echo "  docker exec ros2_jetson bash -lc 'source /app/ros2_ws/install_ros2/setup.bash && ros2 topic list' | sort"
echo ""
echo "Then JETSON_TRANSFER.md §12 validation: PS5 deadman, web UI joystick, e-stop, cameras, person-detect, short autonomous waypoint."
