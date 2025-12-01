#!/bin/bash
# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# Purpose  : Autorun ROS on startup
# ------------------------------------------------------------------

#export HOME=/home/savage22/SPEX/rovers-ros;


# source /home/savage22/ros/rovers-ros/install/setup.bash;
# #source /home/savage22/ros/rovers-ros/.venv/bin/activate;

# dir="/home/savage22/SPEX/rovers-ros/"
# export PYTHONPATH="$dir/install/xbee/lib/python3.10/site-packages:$dir/install/main/lib/python3.10/site-packages:$dir/install/gps/lib/python3.10/site-packages:$dir/install/drive_base/lib/python3.10/site-packages:$dir/install/custom_interfaces/local/lib/python3.10/dist-packages:$dir/install/constants/lib/python3.10/site-packages:$dir/install/can_comms/lib/python3.10/site-packages:$dir/install/cameras/lib/python3.10/site-packages:$dir/install/arm/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$dir/.venv/lib/python3.10/site-packages"

#ros2 launch /home/savage22/SPEX/rovers-ros/src/main/launch main_launch.xml
# source source.sh

source .venv/bin/activate
source install/setup.sh

dir=$(pwd)
export PYTHONPATH="$dir/install/xbee/lib/python3.10/site-packages:$dir/install/main/lib/python3.10/site-packages:$dir/install/gps/lib/python3.10/site-packages:$dir/install/chassis/lib/python3.10/site-packages:$dir/install/custom_interfaces/local/lib/python3.10/dist-packages:$dir/install/constants/lib/python3.10/site-packages:$dir/install/can_comms/lib/python3.10/site-packages:$dir/install/cameras/lib/python3.10/site-packages:$dir/install/arm/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$dir/.venv/lib/python3.10/site-packages:$dir/install/xbee_udp/lib/python3.10/site-packages:$dir/install/basestation/lib/python3.10/site-packages:$dir/install/can_udp/lib/python3.10/site-packages:$dir/install/status_led/lib/python3.10/site-packages"

echo 'source.sh ran'

ros2 launch main main_launch.xml
