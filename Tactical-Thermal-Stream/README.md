# Tactical-Thermal-Stream

ROS 2 **Jazzy** workspace for streaming an **ENEO RGB + Thermal IP camera** (RTSP) into ROS as `sensor_msgs/Image` with **low latency / no buffering** settings.

This repo contains the ROS 2 package `eneo_ip_therm_camera` which:

- Connects to the camera via RTSP (OpenCV + FFmpeg backend)
- Publishes images to ROS topics for:
  - **RGB stream** (web-app / lower quality)
  - **Thermal stream** (web-app / lower quality)
  - **RGB record** (event recording / higher quality)

---

### Repo layout

- `workspace/src/eneo_ip_therm_camera/`: ROS 2 Python package
- `workspace/src/eneo_ip_therm_camera/config/thermal_camera.yml`: camera IP/port/channels
- `workspace/src/eneo_ip_therm_camera/launch/dual_rtsp.launch.py`: launches 3 publishers
- `docker-compose.yml`: convenience runner (ROS 2 Jazzy container)

---

### Prerequisites (native / non-Docker)

- **ROS 2 Jazzy**
- Packages:
  - `cv_bridge` (ROS package, not `pip`)
  - OpenCV with FFmpeg backend (often `python3-opencv` on Ubuntu)

If you already have a ROS 2 Jazzy desktop install, you typically just need:

- `ros-jazzy-cv-bridge`

---

### Configure the camera

Edit:

- `workspace/src/eneo_ip_therm_camera/config/thermal_camera.yml`

Key fields:

- **`ip`**: camera IP (default in repo: `192.168.10.128`)
- **`port`**: RTSP port (usually `554`)
- **`channel.rgb` / `channel.thermal`**: channel numbers used in the RTSP URL template

RTSP URL format used by the launch file:

- `rtsp://admin:<PASSWORD>@<IP>:<PORT>/rtsp/streaming?channel=<CHANNEL>&subtype=<SUBTYPE>&tcp&buffer_size=0`

Note: credentials are currently **hard-coded** in the launch template. If you need different credentials, update `dual_rtsp.launch.py`.

---

### Run with Docker (recommended on Jetson / clean environments)

This repo’s `docker-compose.yml` runs a ROS 2 Jazzy container and executes:

- `colcon build`
- `ros2 launch eneo_ip_therm_camera dual_rtsp.launch.py`

From the repo root:

```bash
docker compose up --build
```

Notes:

- **Docker image**: the compose currently references an image named `realsensed_camera_obstacle_avoidance-realsense` (built by default when running `docker compose build` in the `RealsenseD_camera_Obstacle_avoidance` repo). If you don’t have that image locally yet, build it once there or adjust the compose file to use your preferred base image.
- **Networking**: this compose uses `network_mode: host` to make ROS 2 DDS discovery easy.
- **Workspace mount**: it mounts `./workspace` into `/workspace` and builds inside the container on startup.

---

### Run natively (without Docker)

From the repo root:

```bash
cd workspace
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch eneo_ip_therm_camera dual_rtsp.launch.py
```

---

### Published topics

The launch file starts three nodes (all the same executable `rtsp_image_publisher`) and remaps output topics:

- **RGB (stream)**: `ip_camera/rgb_raw`
- **Thermal (stream)**: `ip_camera/thermal_raw`
- **RGB (record)**: `ip_camera/rgb_raw_record`

All messages are `sensor_msgs/msg/Image` with encoding `bgr8`.

---

### Parameters (per publisher node)

Each `rtsp_image_publisher` instance accepts:

- **`rtsp_url`** (`string`): full RTSP URL
- **`frame_rate`** (`double`): timer/publish rate
- **`topic_name`** (`string`): publisher topic name (the launch file remaps this)

Low-latency behavior is achieved via:

- QoS `KEEP_LAST depth=1` (only newest frame kept)
- OpenCV `CAP_PROP_BUFFERSIZE = 1`
- `grab()`/`retrieve()` flushing to drop buffered frames

---

### Quick verification

List topics:

```bash
ros2 topic list | grep ip_camera
```

View stream (pick one):

```bash
ros2 run rqt_image_view rqt_image_view
```

---

### Troubleshooting

- **RTSP stream won’t open**
  - Verify camera IP reachability: `ping <ip>`
  - Verify RTSP works outside ROS: `ffplay "<rtsp-url>"`
  - Check credentials and channels in `thermal_camera.yml` / `dual_rtsp.launch.py`
- **High latency / “old frames”**
  - This node is designed to drop frames aggressively; if you still see lag, reduce network jitter, and keep QoS depth small on subscribers too.
- **Colcon build creates `build/`, `install/`, `log/`**
  - Those are normal ROS 2 workspace artifacts. If you want a clean rebuild:
    - `rm -rf workspace/build workspace/install workspace/log`
