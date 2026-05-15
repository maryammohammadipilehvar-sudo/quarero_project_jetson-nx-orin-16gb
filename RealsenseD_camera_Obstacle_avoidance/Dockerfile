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
 && rm -rf /var/lib/apt/lists/*

 
WORKDIR /workspace

SHELL ["/bin/bash", "-c"]

# Binary install path: rosdep not required inside image

# Minimal entrypoint that sources ROS env then execs the given command
COPY workspace/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
