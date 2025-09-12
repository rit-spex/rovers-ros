#!/bin/bash
# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# Purpose  : Autorun ROS on startup
# ------------------------------------------------------------------

# Source ROS 2 setup (adjust 'humble' if needed)
source /opt/ros/humble/setup.bash

# Source your workspace setup
source ./source.sh

# Launch ROS 2
ros2 launch main main_launch.xml
