#!/usr/bin/env bash
# Bring-up script for a second Jetson Orin Nano, second site/LAN.
# Run on the TARGET Jetson AFTER you have flashed it to JetPack 6.x (L4T R36.x)
# and logged in as the user that will own the robot stack (suggested: quarero).
#
# Idempotent where practical. Stops on error. Re-run is safe.

set -euo pipefail

REPO_URL_HTTPS="https://github.com/maryammohammadipilehvar-sudo/quarero-projects.git"
REPO_BRANCH="agent/auto-dev"   # match source — push the latest commit on this branch BEFORE running
GIT_DIR="$HOME/gits"

log()  { printf '\n=== %s ===\n' "$*"; }
warn() { printf '\nWARN: %s\n' "$*" >&2; }
die()  { printf '\nERROR: %s\n' "$*" >&2; exit 1; }

require_user_not_root() {
    [[ $EUID -ne 0 ]] || die "Run as your regular user, not root. The script will sudo where needed."
}

require_l4t_r36() {
    local rel
    rel=$(grep -oE 'R[0-9]+' /etc/nv_tegra_release | head -1 || true)
    [[ "$rel" == "R36" ]] || die "Expected L4T R36.x (JetPack 6.x). Found: $(cat /etc/nv_tegra_release | head -1). Re-flash with SDK Manager first."
    log "L4T check OK: $(cat /etc/nv_tegra_release | head -1 | cut -c1-80)"
}

apt_install() {
    log "apt update + install base tools"
    sudo apt-get update
    sudo apt-get install -y \
        git curl ca-certificates jq nano tree screen \
        python3-pip python3-venv
}

install_docker_if_missing() {
    if command -v docker >/dev/null 2>&1; then
        log "docker already installed: $(docker --version)"
        return
    fi
    log "installing docker"
    sudo apt-get install -y docker.io docker-compose-plugin
    sudo systemctl enable --now docker
}

install_nvidia_container_toolkit() {
    log "installing nvidia-container-toolkit"
    # JetPack 6 base image ships this; install or upgrade to be safe.
    sudo apt-get install -y nvidia-container-toolkit nvidia-container-toolkit-base libnvidia-container-tools || \
        warn "nvidia-container-toolkit install failed via apt; on a fresh JP6 image it is usually preinstalled. Verify with: docker info | grep -i runtime"
    sudo nvidia-ctk runtime configure --runtime=docker 2>/dev/null || true
    sudo systemctl restart docker
    docker info 2>/dev/null | grep -i 'Runtimes:.*nvidia' >/dev/null || \
        warn "nvidia runtime not visible to docker. Containers using runtime: nvidia will fail."
}

add_user_to_groups() {
    log "adding $USER to docker, dialout, gpio, i2c, video, render, plugdev"
    sudo usermod -aG docker,dialout,gpio,i2c,video,render,plugdev "$USER"
    warn "group changes take effect on next login. Log out + back in (or reboot) before continuing."
}

disable_nvgetty() {
    # nvgetty grabs /dev/ttyTHS* for a serial console. The containers map ttyTHS1/2 to the ESP32 etc., so it must be inactive.
    log "disabling nvgetty (frees /dev/ttyTHS*)"
    sudo systemctl disable --now nvgetty.service 2>/dev/null || true
    sudo systemctl mask nvgetty.service 2>/dev/null || true
}

install_netbird() {
    if command -v netbird >/dev/null 2>&1; then
        log "netbird already installed: $(netbird version | head -1 || echo unknown)"
        return
    fi
    log "installing netbird"
    curl -fsSL https://pkgs.netbird.io/install.sh | sudo bash
    warn "After this script: run 'sudo netbird up --setup-key <KEY>' with a setup key from the operator's NetBird tenant."
}

clone_repo() {
    if [[ -d "$GIT_DIR/.git" ]]; then
        log "repo already present at $GIT_DIR — fetching"
        git -C "$GIT_DIR" fetch origin
        git -C "$GIT_DIR" checkout "$REPO_BRANCH"
        git -C "$GIT_DIR" pull --ff-only origin "$REPO_BRANCH" || warn "pull not fast-forward; resolve manually"
        return
    fi
    log "cloning repo into $GIT_DIR (branch: $REPO_BRANCH)"
    git clone --branch "$REPO_BRANCH" "$REPO_URL_HTTPS" "$GIT_DIR"
}

build_and_start_stack() {
    log "building and starting the four compose projects"
    # Order matters slightly: cameras can come up independently, but bring web_app last so it can
    # see the others' topics on first start.
    local projects=(
        "$GIT_DIR/livox_mid_360_obstacle_detection"
        "$GIT_DIR/RealsenseD_camera_Obstacle_avoidance"
        "$GIT_DIR/Tactical-Thermal-Stream"
        "$GIT_DIR/tactical_mower"
    )
    for p in "${projects[@]}"; do
        if [[ ! -d "$p" ]]; then
            warn "missing project dir: $p (skipping)"
            continue
        fi
        log "compose up: $p"
        (cd "$p" && docker compose up -d --build) || warn "compose up failed for $p"
    done
}

print_final_checklist() {
    cat <<'EOF'

=================================================================
Bring-up script finished. Manual steps remaining
(none of these are safe to automate — see BRINGUP.md for detail):

  1. NetBird enrollment:
       sudo netbird up --setup-key <KEY-FROM-OPERATOR>

  2. Edit site-specific config for the NEW site:
       ~/gits/routen/settings/settings.yaml
         - home_point.latitude / longitude / yaw
         - charge_point.latitude / longitude / yaw
         - speed_factor, battery_threshold, enable_obstacle_avoidance
       ~/gits/routen/routes/                # write new route YAMLs for the new site
       ~/gits/routen/settings/schedules.yaml   # rebuild schedules referencing new routes

  3. If the new eneo camera lives at a different IP than 192.168.10.203:
       ~/gits/tactical_mower/ros2_ws/src/eneo_event_publisher/eneo_event_publisher/eneo_parser.py
       (and rebuild the ros2_jetson image: cd ~/gits/tactical_mower && docker compose build ros2_jetson)

  4. Plug peripherals:
       - Roboclaw on USB        -> /dev/ttyACM0
       - ESP32 on UART1         -> /dev/ttyTHS1
       - Spare UART             -> /dev/ttyTHS2
       Confirm with: ls -l /dev/ttyACM* /dev/ttyTHS*

  5. Verify groups took effect: groups | grep -E 'docker|dialout'
     If not present, log out + log back in.

  6. Verify containers: docker ps
     Expected: web_app, livox_ros2_jazzy, theramal_camera_jazzy, intel_realsense_ros2

  7. Robot e-stop, manual drive, then routed run — only with operator present.
     See CLAUDE.md sections 1 and 8 before any autonomous test.
=================================================================
EOF
}

main() {
    require_user_not_root
    require_l4t_r36
    apt_install
    install_docker_if_missing
    install_nvidia_container_toolkit
    add_user_to_groups
    disable_nvgetty
    install_netbird
    clone_repo
    build_and_start_stack
    print_final_checklist
}

main "$@"
