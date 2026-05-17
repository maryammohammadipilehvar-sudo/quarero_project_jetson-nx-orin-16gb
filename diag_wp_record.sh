#!/usr/bin/env bash
# diag_wp_record.sh
#
# Capture a full ROS bag + config snapshot for a waypoint-following test.
# Records ALL topics (firehose: cameras, depth, lidar included) plus a
# sidecar of the YAML configs and ROS topology so a future reader can
# re-derive what the WP follower saw and why it decided what it did.
#
# Usage:
#   ./diag_wp_record.sh [label]
# Stop with Ctrl-C (sends SIGINT to ros2 bag inside the container; never
# SIGKILL — the mcap must close cleanly).
#
# Output: ~/gits/tactical_mower/ros2_ws/ros_bags/<label>_<ts>/
#         (ros_bags/ is already in ~/gits/.gitignore)

set -euo pipefail

LABEL="${1:-wp_test}"
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
echo "[diag] run id     : $RUN"
echo "[diag] output dir : $HOST_DIR"
echo

# ---------- pre-flight snapshot ----------
echo "[diag] pre-flight snapshot..."

{
  echo "# WP follower diagnostic capture"
  echo
  echo "- label    : $LABEL"
  echo "- ts       : $TS"
  echo "- host     : $(hostname)"
  echo "- uptime   : $(uptime)"
  echo
  echo "## Operator notes"
  echo "_(fill in after the run: what route was loaded, what start condition, what failed)_"
  echo
  echo "## Disk free at start"
  df -h "$HOST_BASE" | sed 's/^/    /'
} > "$HOST_DIR/README.md"

{
  echo "=== git -C ~/gits rev-parse HEAD ==="
  git -C "$HOME/gits" rev-parse HEAD
  echo
  echo "=== git -C ~/gits status --short ==="
  git -C "$HOME/gits" status --short
  echo
  echo "=== git -C ~/gits log -n 5 --oneline ==="
  git -C "$HOME/gits" log -n 5 --oneline
} > "$HOST_DIR/git_status.txt" 2>&1 || true

# Configs (bind-mounted into container — copy from host)
cp -r "$HOME/gits/routen/routes"             "$HOST_DIR/routen_routes"        2>/dev/null || true
cp    "$HOME/gits/routen/settings/settings.yaml"  "$HOST_DIR/settings.yaml"   2>/dev/null || true
cp    "$HOME/gits/routen/settings/schedules.yaml" "$HOST_DIR/schedules.yaml"  2>/dev/null || true
cp    "$HOME/gits/tactical_mower/ros2_ws/src/control/config/params.yaml" "$HOST_DIR/control_params.yaml" 2>/dev/null || true
cp    "$HOME/gits/tactical_mower/ros2_ws/src/drive/config/params.yaml"   "$HOST_DIR/drive_params.yaml"   2>/dev/null || true

# ROS topology (live). timeout protects against unresponsive nodes.
timeout 10 docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 topic list -t" > "$HOST_DIR/topic_list_before.txt" 2>&1 || echo "(timed out)" >> "$HOST_DIR/topic_list_before.txt"
timeout 10 docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 node list"     > "$HOST_DIR/node_list_before.txt"  2>&1 || echo "(timed out)" >> "$HOST_DIR/node_list_before.txt"

# Runtime params (after launch overrides have been applied).
# 10 s per node — some nodes (nav2_navigation_node has been seen) don't respond
# to the parameter service under load; we skip them rather than hang the script.
for node in /tactical_wp_follower /tactical_scheduler /robot_controller /roboclaw_drive /fixposition_driver_ros2 /joy_controller /nav2_navigation_node; do
  fn=$(echo "$node" | tr / _).yaml
  timeout 10 docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 param dump $node" > "$HOST_DIR/params${fn}" 2>&1 \
    || echo "(ros2 param dump $node: timed out or failed)" > "$HOST_DIR/params${fn}"
done

# Container + host snapshot
docker ps --format '{{.Names}}\t{{.Status}}'         > "$HOST_DIR/docker_ps_before.txt" 2>&1 || true
df -h                                                 > "$HOST_DIR/df_before.txt"        2>&1 || true

echo "[diag] pre-flight snapshot complete"
echo

# ---------- start bag ----------
echo "============================================================"
echo "[diag] starting bag (FIREHOSE: cameras + depth + lidar)"
echo "[diag] expected ~2-3 GB/min. Keep runs under ~10 min."
echo "[diag] disk: $(df -h --output=avail "$HOST_BASE" | tail -1 | xargs) free"
echo
echo "  ====> WAIT for the line  '[INFO] ... Recording...'  <===="
echo "        That can take ~10 seconds. Do NOT Ctrl-C before it."
echo "        Once you see 'Recording...', start your test."
echo
echo "[diag] Ctrl-C to stop cleanly. DO NOT kill -9."
echo "============================================================"
echo

# Bag log goes to host-side file we can tail.
BAG_LOG="$HOST_DIR/bag_record.log"
: > "$BAG_LOG"

# Cleanup on Ctrl-C — sends SIGINT into the container so mcap closes cleanly.
stopped=0
on_stop() {
  if [ "$stopped" -eq 1 ]; then return; fi
  stopped=1
  echo
  echo "[diag] stopping bag cleanly (pkill -INT inside container)..."
  docker exec "$CONTAINER" pkill -INT -f 'ros2 bag record' 2>/dev/null || true
}
trap on_stop INT TERM

# Start bag in background. NO -it (some terminals don't have a TTY and -it then
# fails fast, leaving no bag). Output is captured to BAG_LOG which we tail.
docker exec "$CONTAINER" bash -lc "
  $ros2_env &&
  mkdir -p '${CTR_DIR}' &&
  cd '${CTR_DIR}' &&
  exec ros2 bag record -a \
    --storage mcap \
    --storage-preset-profile fastwrite \
    --disable-keyboard-controls \
    -o bag
" >>"$BAG_LOG" 2>&1 &
DOCKER_PID=$!

# Tail the bag log so the user can SEE subscribe progress and the
# 'Recording...' line. Tail dies when ros2 bag exits.
tail -f "$BAG_LOG" --pid="$DOCKER_PID" &
TAIL_PID=$!

# Wait for the docker exec to finish (i.e. ros2 bag exited)
wait "$DOCKER_PID" 2>/dev/null || true
kill "$TAIL_PID" 2>/dev/null || true
trap - INT TERM

echo
echo "[diag] post-flight snapshot..."

# Active route + state at stop time (bind-mounted, so on host already)
# Copy whichever route was active at the time of stop — read it from
# robot state if we can; otherwise just dump the whole routes/ dir again
# (we already snapshot pre-flight; this captures any UI-edits during the run).
cp -r "$HOME/gits/routen/routes" "$HOST_DIR/routen_routes_after" 2>/dev/null || true

docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 topic list -t" > "$HOST_DIR/topic_list_after.txt" 2>&1 || true
docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 node list"     > "$HOST_DIR/node_list_after.txt"  2>&1 || true
docker exec "$CONTAINER" bash -lc "$ros2_env && ros2 bag info '${BAG_DIR_CTR}'" > "$HOST_DIR/bag_info.txt" 2>&1 || true

docker ps --format '{{.Names}}\t{{.Status}}' > "$HOST_DIR/docker_ps_after.txt" 2>&1 || true
df -h                                         > "$HOST_DIR/df_after.txt"        2>&1 || true

# Compact size summary
{
  echo "# Post-run summary"
  echo
  echo "## Bag size"
  du -sh "$BAG_DIR_HOST" 2>/dev/null || echo "  (bag dir not found at $BAG_DIR_HOST)"
  echo
  echo "## Topics + msg counts"
  cat "$HOST_DIR/bag_info.txt" | sed 's/^/    /'
} > "$HOST_DIR/SUMMARY.md"

echo
echo "[diag] done."
echo "[diag] bag + sidecar: $HOST_DIR"
echo "[diag] size: $(du -sh "$BAG_DIR_HOST" 2>/dev/null | awk '{print $1}')"
echo "[diag] open $HOST_DIR/SUMMARY.md for topic/msg counts."
