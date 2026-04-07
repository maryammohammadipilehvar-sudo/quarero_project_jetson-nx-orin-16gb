## livox_mid_360_obstacle_detection

ROS 2 Jazzy workspace for the Livox MID-360 LiDAR driver and obstacle detection logic, running in a Docker container (similar style to the `tactical_mower` repository).

### Prerequisites

- Docker and Docker Compose installed.
- NVIDIA drivers and `nvidia-container-toolkit` set up (for Jetson / GPU usage).
- This repository cloned with submodules:
  - Either via `git clone --recurse-submodules ...`
  - Or run `git submodule update --init --recursive` after cloning.

### Quick setup

From the repo root:

```bash
cd livox_mid_360_obstacle_detection
chmod +x setup.sh   # first time only
./setup.sh
```

What `setup.sh` does:

- Initializes git submodules (e.g. `ros2_ws/src/livox_ros_driver2`).
- Ensures `ros2_ws/src` exists.

It does **not** start Docker or build anything, to keep responsibilities clear.

Next steps after `./setup.sh`:

```bash
cd livox_mid_360_obstacle_detection
docker compose up --build -d ros2_livox
```

The container entrypoint (defined in `docker-compose.yaml`) will:

- Source ROS 2 Jazzy.
- Run `colcon build` in `/livox/ros2_ws` to build all packages.
- Source the workspace install setup.
- Automatically launch the obstacle detection node (`livox_mid360_obstacle/obstacle_detection.launch.py`).

### Using the container

Enter the running container:

```bash
docker exec -it livox_ros2_jazzy bash
```

Inside the container (if you need an interactive shell):

```bash
source /opt/ros/jazzy/setup.bash
source /livox/ros2_ws/install/setup.bash   # after colcon build has completed
```

You now have:

- `livox_ros_driver2` (from the git submodule) available in the workspace.
- Your own package `livox_mid360_obstacle` available for obstacle detection logic.

### Livox MID-360 configuration (Ethernet)

1. Configure networking on the host so you can reach the LiDAR:

```bash
ping <LIDAR_IP>
```

2. In the container, edit the MID-360 config:

```bash
cd /livox/ros2_ws/src/livox_ros_driver2/config
nano MID360_config.json
```

Set at least:

- **`host_ip`**: IP address of the host/container interface on the LiDAR network.
- **`lidar_ip`**: IP address of the MID-360.

### Running the Livox driver

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /livox/ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

You should see topics like `/livox/points` (or similar) and point clouds in RViz.

### Obstacle detection package

Your Python obstacle package lives in:

- `ros2_ws/src/livox_mid360_obstacle`

It should:

- Subscribe to the Livox pointcloud topic.
- Perform filtering / obstacle detection.
- Publish navigation or obstacle messages (e.g. to your main drive stack via ROS 2 topics).


