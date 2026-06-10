#!/bin/bash
# start_extras.sh — Quarero startup hook. Runs at ros2_jetson container init.
# 1) Copies scripts/ contents into /usr/local/bin so legacy paths keep working
# 2) Restores /dev/esp_joy symlink (lost on container restart)
# 3) Detaches watchdogs as background processes
# Idempotent — safe to run repeatedly.

set -e

SCRIPTS_DIR="${SCRIPTS_DIR:-/app/scripts}"

# 1) Install / refresh scripts in /usr/local/bin
for f in autonomy_supervisor.py autonomy_watchdog.sh joy_controller_watchdog.sh \
         person_event_bridge.py person_event_bridge_watchdog.sh; do
    if [ -f "$SCRIPTS_DIR/$f" ]; then
        install -m 755 "$SCRIPTS_DIR/$f" "/usr/local/bin/$f"
    fi
done

# 2) Restore /dev/esp_joy symlink so joy_controller finds the ESP32
if [ -e /dev/ttyUSB0 ]; then
    ln -sf /dev/ttyUSB0 /dev/esp_joy
fi

# 3) Detach watchdogs (idempotent — kill any old ones first)
for w in autonomy_watchdog.sh joy_controller_watchdog.sh person_event_bridge_watchdog.sh; do
    pkill -f "$w" 2>/dev/null || true
done
sleep 1
for w in autonomy_watchdog.sh joy_controller_watchdog.sh person_event_bridge_watchdog.sh; do
    if [ -x "/usr/local/bin/$w" ]; then
        setsid nohup "/usr/local/bin/$w" > "/tmp/${w%.sh}.log" 2>&1 < /dev/null &
        echo "[start_extras] launched $w"
    fi
done

echo "[start_extras] done"
exit 0
