# syntax=docker/dockerfile:1

FROM arm64v8/ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=jazzy \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Install Intel RealSense ROS packages and useful udev/usb tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    usbutils udev v4l-utils \
    curl \
    ros-jazzy-rqt-image-view \
    ros-${ROS_DISTRO}-rqt-image-view \
    htop \
 && rm -rf /var/lib/apt/lists/*

 
WORKDIR /workspace

SHELL ["/bin/bash", "-c"]

# Binary install path: rosdep not required inside image

# Minimal entrypoint that sources ROS env then execs the given command
COPY workspace/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
