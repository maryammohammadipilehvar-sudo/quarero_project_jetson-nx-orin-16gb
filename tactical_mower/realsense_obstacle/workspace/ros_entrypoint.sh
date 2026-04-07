#!/bin/bash

source /opt/ros/jazzy/setup.bash
colcon build
source install/local_setup.bash

exec "$@"
