# RealsenseD_camera_Obstacle_avoidance

ROS 2 **Jazzy** workspace for Intel **RealSense** streaming + a simple **depth-based obstacle detector**.

This repo provides the ROS 2 package `realsense_obstacle` which:

- Launches `realsense2_camera` (color + depth, aligned depth, filters enabled)
- Subscribes to a depth image topic
- Computes a coarse “obstacle occupancy grid” in front of the robot
- Publishes:
  - a global boolean flag (`/obstacle_detected`)
  - per-sector occupancy (`/obstacle_sectors`, 10 values)
  - a debug image for visualization (`/camera/camera/depth/obstacle_debug`)

---

### Repo layout

- `workspace/src/realsense_obstacle/`: ROS 2 Python package
- `workspace/src/realsense_obstacle/launch/realsense_obstacle.launch.py`: launches RealSense + detector
- `workspace/src/realsense_obstacle/config/params.yaml`: detector parameters
- `Dockerfile` / `docker-compose.yml`: container runner (arm64 ROS 2 Jazzy)

---

### Prerequisites (hardware / platform)

- Intel RealSense camera (code comments mention **D455**)
- USB permissions for RealSense (Docker uses `privileged: true` and mounts `/dev`)
- This Docker image is **arm64**-based (`arm64v8/ros:jazzy-ros-base`) → ideal for Jetson.
  - On x86_64 you’ll want a different base image (or build on an arm64 host).

---

### Run with Docker (recommended)

From the repo root:

```bash
docker compose up --build
```

What happens on container start:

- `/ros_entrypoint.sh` runs `colcon build` in `/workspace`
- then `docker-compose.yml` launches:
  - `ros2 launch realsense_obstacle realsense_obstacle.launch.py`

Networking note:

- Current compose uses `network_mode: bridge`.
- If you want to subscribe to topics **from the host** (outside the container), switching to **`network_mode: host`** often makes ROS 2 discovery much simpler.

---

### Run natively (without Docker)

Install ROS 2 Jazzy + RealSense ROS packages, then:

```bash
cd workspace
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch realsense_obstacle realsense_obstacle.launch.py
```

---

### Launch contents (what it starts)

`realsense_obstacle.launch.py`:

- Includes `realsense2_camera/launch/rs_launch.py` with:
  - color + depth enabled
  - depth profile set to `640x480x30`
  - depth aligned to color
  - decimation/spatial/temporal/hole-filling filters enabled
- Starts `realsense_obstacle/simple_obstacle_detector.py` as node name `sector_obstacle_detector`

---

### Topics

Detector subscribes (default):

- **Depth input**: `/camera/camera/depth/image_rect_raw` (`sensor_msgs/msg/Image`, often `16UC1`)

Detector publishes:

- **Global obstacle flag**: `/obstacle_detected` (`std_msgs/msg/Bool`)
- **Sector occupancy**: `/obstacle_sectors` (`std_msgs/msg/UInt8MultiArray`, length = 10)
  - indices `0..4`: **near** sectors left→right
  - indices `5..9`: **far** sectors left→right
- **Debug image**: `/camera/camera/depth/obstacle_debug` (`sensor_msgs/msg/Image`, `bgr8`)

---

### Parameters

Edit:

- `workspace/src/realsense_obstacle/config/params.yaml`

Important parameters:

- **`depth_topic`**: depth input topic
- **`near_distance_m`**: threshold for “near” obstacles
- **`far_distance_m`**: threshold for “far” obstacles
- **`min_sector_pixels`**: pixels required to mark a sector occupied
- **`num_cols`**: number of horizontal sectors (default 5)

---

### Visualize the debug image

On a machine with GUI support:

```bash
ros2 run rqt_image_view rqt_image_view
```

If you’re running inside Docker on a headless device, you’ll typically need either:

- X11 forwarding (`ssh -X`), or
- a local display server + `DISPLAY` forwarding into the container, or
- just subscribe to the debug image from another ROS 2 machine on the network.

---

### Troubleshooting

- **No camera / no topics**
  - Check USB visibility: `lsusb`
  - In Docker, confirm `/dev` is mounted and the container is privileged.
- **Depth topic name differs**
  - List topics: `ros2 topic list | grep depth`
  - Update `depth_topic` in `params.yaml`.
- **Colcon build creates `build/`, `install/`, `log/`**
  - Normal ROS 2 workspace artifacts. Clean rebuild:
    - `rm -rf workspace/build workspace/install workspace/log`
