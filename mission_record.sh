#!/usr/bin/env bash
# mission_record.sh — targeted bag for full autonomous mission analysis
#
# Records all decision-relevant topics (state machine, drive commands, GPS,
# obstacles, RTK, speed, logs) WITHOUT camera/depth streams. Runs for the
# full mission duration (~50 MB/min vs firehose's 2-3 GB/min).
#
# Usage:
#   ./mission_record.sh [label]
#   Stop with Ctrl-C when mission is complete.

set -euo pipefail

LABEL="${1:-mission}"
TS=$(date +%Y%m%d_%H%M%S)
RUN="${LABEL}_${TS}"

HOST_BASE="$HOME/gits/tactical_mower/ros2_ws/ros_bags"
HOST_DIR="${HOST_BASE}/${RUN}"
CTR_DIR="/app/ros2_ws/ros_bags/${RUN}"
BAG_DIR_CTR="${CTR_DIR}/bag"
BAG_DIR_HOST="${HOST_DIR}/bag"

CONTAINER=ros2_jetson
ros2_env="source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install_ros2/setup.bash"

mkdir -p "$HOST_DIR"
echo "[mission] run id     : $RUN"
echo "[mission] output dir : $HOST_DIR"
echo

# ---------- pre-flight snapshot ----------
echo "[mission] pre-flight snapshot..."

{
  echo "# Autonomous Mission Recording"
  echo "- label    : $LABEL"
  echo "- ts       : $TS"
  echo "- host     : $(hostname)"
  echo "- uptime   : $(uptime)"
  echo
  echo "## Operator notes"
  echo "_(fill in: route name, expected behavior, what to watch for)_"
} > "$HOST_DIR/README.md"

{
  echo "=== git rev-parse HEAD ==="
  git -C "$HOME/gits" rev-parse HEAD
  echo
  echo "=== git log -n 3 --oneline ==="
  git -C "$HOME/gits" log -n 3 --oneline
} > "$HOST_DIR/git_status.txt" 2>&1 || true

cp -r "$HOME/gits/routen/routes"             "$HOST_DIR/routen_routes"        2>/dev/null || true
cp    "$HOME/gits/routen/settings/settings.yaml"  "$HOST_DIR/settings.yaml"   2>/dev/null || true
cp    "$HOME/gits/routen/settings/schedules.yaml" "$HOST_DIR/schedules.yaml"  2>/dev/null || true
cp    "$HOME/gits/tactical_mower/ros2_ws/src/control/config/params.yaml" "$HOST_DIR/control_params.yaml" 2>/dev/null || true
cp    "$HOME/gits/tactical_mower/ros2_ws/src/drive/config/params.yaml"   "$HOST_DIR/drive_params.yaml"   2>/dev/null || true

# Runtime params
for node in /tactical_wp_follower /tactical_scheduler /robot_controller /roboclaw_drive; do
  fn=$(echo "$node" | tr / _).yaml
  timeout 10 docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 param dump $node" > "$HOST_DIR/params${fn}" 2>&1 \
    || echo "(ros2 param dump $node: timed out or failed)" > "$HOST_DIR/params${fn}"
done

# Robot state at start
docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 topic echo --full-length /tactical/robot/state --once 2>/dev/null" > "$HOST_DIR/state_at_start.txt" 2>&1 || true
docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 topic echo --field data /robot/state --once 2>/dev/null" > "$HOST_DIR/robot_state_at_start.txt" 2>&1 || true

echo "[mission] pre-flight done"
echo

# ---------- topic list for the targeted bag ----------
# All decision-relevant topics for autonomous mission analysis:
TOPICS=(
  # State machine + high-level decisions
  /tactical/robot/state
  /robot/state
  /tactical/robot/route_completed
  /tactical/robot/charging_status
  /tactical/robot/speed_kmh
  /control/autonomous_operation

  # Drive commands (what motors are told)
  /cmd_drive
  /joy_drive_raw
  /joy_web

  # GPS / RTK / heading (localization)
  /fixposition/odometry_enu
  /fixposition/odometry_llh
  /fixposition/ypr
  /fixposition/fusion

  # Obstacle detection (decisions, not raw sensor)
  /obstacles/lidar
  /obstacles/sectors
  /obstacles/fused
  /obstacle_detected
  /control/obstacle_avoidance_enabled

  # Speed + control inputs
  /control/set_speed

  # Charging / docking
  /tactical/control/charging/dock
  /tactical/control/charging/undock
  /tactical/control/charging/requested
  /control/enable_charging
  # ArUco precision docking — marker pose (range=position.z) drives the final
  # approach + stop. Small (PoseStamped); /docking/aruco_debug image is omitted
  # to stay light — view it live in the web UI if needed.
  /docking/aruco_pose

  # Logging (captures warnings/errors during mission)
  /tactical/logging/info
  /tactical/logging/warn
  /tactical/logging/error

  # TF (for coordinate transforms replay)
  /tf
  /tf_static
)

TOPIC_ARGS=""
for t in "${TOPICS[@]}"; do
  TOPIC_ARGS="$TOPIC_ARGS $t"
done

# ---------- start bag ----------
echo "============================================================"
echo "[mission] TARGETED recording (no cameras, ~50 MB/min)"
echo "[mission] disk free: $(df -h --output=avail "$HOST_BASE" | tail -1 | xargs)"
echo "[mission] topics: ${#TOPICS[@]}"
echo
echo "  ====> WAIT for 'Recording...' line before starting mission <===="
echo "        Ctrl-C to stop cleanly."
echo "============================================================"
echo

BAG_LOG="$HOST_DIR/bag_record.log"
: > "$BAG_LOG"

stopped=0
on_stop() {
  if [ "$stopped" -eq 1 ]; then return; fi
  stopped=1
  echo
  echo "[mission] stopping bag cleanly..."
  docker exec "$CONTAINER" pkill -INT -f 'ros2 bag record' 2>/dev/null || true
}
trap on_stop INT TERM

docker exec "$CONTAINER" bash -lc "
  $ros2_env &&
  mkdir -p '${CTR_DIR}' &&
  cd '${CTR_DIR}' &&
  exec ros2 bag record \
    --storage mcap \
    --storage-preset-profile fastwrite \
    --disable-keyboard-controls \
    -o bag \
    $TOPIC_ARGS
" >>"$BAG_LOG" 2>&1 &
DOCKER_PID=$!

tail -f "$BAG_LOG" --pid="$DOCKER_PID" &
TAIL_PID=$!

wait "$DOCKER_PID" 2>/dev/null || true
kill "$TAIL_PID" 2>/dev/null || true
trap - INT TERM

echo
echo "[mission] post-flight snapshot..."

# State at end
docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 topic echo --full-length /tactical/robot/state --once 2>/dev/null" > "$HOST_DIR/state_at_end.txt" 2>&1 || true
docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 topic echo --field data /robot/state --once 2>/dev/null" > "$HOST_DIR/robot_state_at_end.txt" 2>&1 || true
docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 bag info '${BAG_DIR_CTR}'" > "$HOST_DIR/bag_info.txt" 2>&1 || true

{
  echo "# Mission Recording Summary"
  echo "- Run: $RUN"
  echo "- Duration: $(date)"
  echo
  echo "## Bag size"
  du -sh "$BAG_DIR_HOST" 2>/dev/null || echo "  (not found)"
  echo
  echo "## Topics + msg counts"
  cat "$HOST_DIR/bag_info.txt" 2>/dev/null | sed 's/^/    /'
} > "$HOST_DIR/SUMMARY.md"

echo
echo "[mission] done."
echo "[mission] bag: $HOST_DIR"
echo "[mission] size: $(du -sh "$BAG_DIR_HOST" 2>/dev/null | awk '{print $1}')"
echo
echo "To analyze: python3 ~/gits/tactical_mower/ros2_ws/analyze_wp_bag.py $BAG_DIR_HOST"
