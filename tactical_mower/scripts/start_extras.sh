#!/bin/bash
# start_extras.sh - Quarero startup hook. Runs at ros2_jetson container init.
# 1) Copies scripts/ contents into /usr/local/bin so legacy paths keep working
# 2) Restores /dev/esp_joy symlink (lost on container restart)
# 3) Detaches watchdogs as background processes
# 4) Boot pause hook: publishes /control/autonomous_operation=false once the
#    autonomy nodes have come up, so the robot starts in MANUAL.
# Idempotent - safe to run repeatedly.
#
# NOTE 2026-06-11: joy_controller_watchdog.sh removed from the launch list.
# joy_controller is now spawned solely by controller.launch.py. Running both
# caused two python3 instances to fight over the ESP UART and produced visible
# PS5 input latency. ros2 launch handles its own respawn.

set -e

SCRIPTS_DIR="${SCRIPTS_DIR:-/app/scripts}"

# 1) Install / refresh scripts in /usr/local/bin
for f in autonomy_supervisor.py autonomy_watchdog.sh \
         person_event_bridge.py person_event_bridge_watchdog.sh; do
    if [ -f "$SCRIPTS_DIR/$f" ]; then
        install -m 755 "$SCRIPTS_DIR/$f" "/usr/local/bin/$f"
    fi
done

# 2) Restore /dev/esp_joy symlink so joy_controller finds the ESP32
if [ -e /dev/ttyUSB0 ]; then
    ln -sf /dev/ttyUSB0 /dev/esp_joy
fi

# 3) Detach watchdogs (idempotent - kill any old ones first).
#    joy_controller_watchdog.sh is intentionally NOT in this list - see header.
for w in autonomy_watchdog.sh joy_controller_watchdog.sh person_event_bridge_watchdog.sh; do
    pkill -f "$w" 2>/dev/null || true
done
sleep 1
for w in autonomy_watchdog.sh person_event_bridge_watchdog.sh; do
    if [ -x "/usr/local/bin/$w" ]; then
        setsid nohup "/usr/local/bin/$w" > "/tmp/${w%.sh}.log" 2>&1 < /dev/null &
        echo "[start_extras] launched $w"
    fi
done

# 4) Boot pause hook - wait until both autonomy nodes have spawned, then
#    publish autonomous_operation=false so the supervisor SIGSTOPs them.
#    Background; survives parent exit via setsid nohup.
setsid nohup bash -c '
    for i in $(seq 1 180); do
        if pgrep -f livox_obstacle_node >/dev/null \
           && pgrep -f simple_obstacle_detector >/dev/null; then
            sleep 3   # give the supervisor a moment to be subscribed
            source /opt/ros/jazzy/setup.bash 2>/dev/null
            source /app/ros2_ws/install_ros2/setup.bash 2>/dev/null
            echo "[boot_pause] $(date +%T) autonomy nodes up, publishing autonomy=false"
            ros2 topic pub -r 1 --times 3 /control/autonomous_operation \
                std_msgs/msg/Bool "{data: false}"
            echo "[boot_pause] $(date +%T) done"
            exit 0
        fi
        sleep 1
    done
    echo "[boot_pause] $(date +%T) timeout waiting for autonomy nodes"
' > /tmp/boot_pause.log 2>&1 < /dev/null &
echo "[start_extras] boot_pause hook armed (log: /tmp/boot_pause.log)"

echo "[start_extras] done"
exit 0
