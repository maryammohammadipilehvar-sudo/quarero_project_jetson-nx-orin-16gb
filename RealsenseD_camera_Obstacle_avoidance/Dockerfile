# syntax=docker/dockerfile:1

FROM arm64v8/ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=jazzy \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Install Luxonis OAK (depthai-ros) and useful udev/usb tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-depthai-ros-driver \
    ros-${ROS_DISTRO}-depthai-descriptions \
    ros-${ROS_DISTRO}-topic-tools \
    usbutils udev v4l-utils \
    curl \
    ros-${ROS_DISTRO}-rqt-image-view \
    htop \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

# ArUco lives in opencv_contrib. Ubuntu 24.04 / ROS Jazzy ships OpenCV 4.6 via
# apt where the apt python3-opencv omits the contrib aruco module, so install
# the contrib wheel (headless = no GUI libs; nodes only publish images, never
# imshow). Needed by realsense_obstacle/aruco_dock_detector.py (cv2.aruco).
#
# Pin 4.10.0.84 (last line that supports numpy 1.x) and --no-deps so the image's
# Debian numpy 1.26 — which ros-jazzy cv_bridge is compiled against — is left
# untouched. Newer wheels force numpy>=2 and break that ABI.
RUN pip3 install --no-cache-dir --break-system-packages --no-deps \
    opencv-contrib-python-headless==4.10.0.84

 
WORKDIR /workspace

SHELL ["/bin/bash", "-c"]

# Binary install path: rosdep not required inside image

# Minimal entrypoint that sources ROS env then execs the given command
COPY workspace/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
