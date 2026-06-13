#!/bin/bash
# Host-side watchdog for the LiDAR processing pipeline.
#
# Hooked to docker events: every time ros2_jetson starts (initial boot or
# any restart), wait 30s for the autonomy stack to settle, then read the
# latest wp_follower NAVTEL line and pull `lidar_age` from it. If lidar_age
# is greater than 10 seconds the cross-container DDS discovery did not
# pick up the livox publisher; restart livox_ros2_jazzy so it re-announces
# and wp_follower re-discovers.
#
# Idempotent and safe to run alongside the existing ros2_jetson restart
# policy (restart: unless-stopped). The cooldown after a restart prevents
# loops if livox can't recover.
#
# Install:
#   crontab -e
#   @reboot /home/quarero02/scripts/livox_health_watchdog.sh >> /home/quarero02/livox-watchdog.log 2>&1 &

set -u

LOG_PREFIX="[$(basename "$0")]"
WAIT_AFTER_START=30      # seconds to give the autonomy stack to settle
LIDAR_STALE_THRESHOLD=10 # seconds, above which we treat the pipeline as broken
COOLDOWN_AFTER_RESTART=60

log() { echo "$LOG_PREFIX [$(date +'%F %T')] $*"; }

check_and_heal() {
    # Pull the most recent NAVTEL line from the ros2_jetson container.
    local last_line
    last_line=$(docker logs --tail 300 ros2_jetson 2>&1 | grep "NAVTEL" | tail -1 || true)
    if [ -z "$last_line" ]; then
        log "no NAVTEL line yet — wp_follower probably still starting; skipping"
        return
    fi

    # Extract lidar_age value (works with the existing NAVTEL emit format).
    local lidar_age
    lidar_age=$(echo "$last_line" | grep -oP 'lidar_age=\K[0-9.]+' || true)
    if [ -z "$lidar_age" ]; then
        log "could not parse lidar_age from NAVTEL line; skipping"
        return
    fi

    # Compare via awk so floats work.
    if awk -v a="$lidar_age" -v t="$LIDAR_STALE_THRESHOLD" 'BEGIN { exit !(a > t) }'; then
        log "lidar_age=$lidar_age > ${LIDAR_STALE_THRESHOLD}s — restarting livox_ros2_jazzy"
        if docker restart livox_ros2_jazzy >/dev/null 2>&1; then
            log "livox_ros2_jazzy restarted; cooldown ${COOLDOWN_AFTER_RESTART}s"
            sleep "$COOLDOWN_AFTER_RESTART"
        else
            log "ERROR: docker restart livox_ros2_jazzy failed"
        fi
    else
        log "lidar_age=$lidar_age — healthy, no action"
    fi
}

log "starting LiDAR pipeline watchdog (PID $$)"

# First-pass check at startup of the watchdog itself, in case ros2_jetson is
# already running (typical when this script starts via @reboot after host boot).
sleep "$WAIT_AFTER_START"
check_and_heal

# Then react to every subsequent ros2_jetson start event.
docker events --filter container=ros2_jetson --filter event=start \
    --format '{{.Time}} {{.Action}}' 2>/dev/null | while read -r line; do
    log "ros2_jetson start event: $line — waiting ${WAIT_AFTER_START}s"
    sleep "$WAIT_AFTER_START"
    check_and_heal
done

log "docker events stream ended unexpectedly; exiting"
