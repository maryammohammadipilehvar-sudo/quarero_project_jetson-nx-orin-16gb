#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> Initializing git submodules (Livox driver, etc.)"
git -C "${REPO_ROOT}" submodule update --init --recursive

echo "==> Creating ROS 2 workspace directory structure if needed"
mkdir -p "${REPO_ROOT}/ros2_ws/src"

echo
echo "Setup finished."
echo
echo "Next steps:"
echo "  1) Build and start the Docker container:"
echo "       cd \"${REPO_ROOT}\""
echo "       docker compose up --build -d ros2_livox"
echo "  2) The container will automatically run colcon build and launch the obstacle detection on startup."
echo "  3) Enter the container if you want an interactive shell:"
echo "       docker exec -it livox_ros2_jazzy bash"


