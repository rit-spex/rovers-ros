source install/setup.sh
source .venv/bin/activate
dir=$(pwd)
export PYTHONPATH="
$dir/install/xbee/lib/python3.10/site-packages:
$dir/install/main/lib/python3.10/site-packages:
$dir/install/gps/lib/python3.10/site-packages:
$dir/install/chassis/lib/python3.10/site-packages:
$dir/install/custom_interfaces/local/lib/python3.10/dist-packages:
$dir/install/constants/lib/python3.10/site-packages:
$dir/install/can_comms/lib/python3.10/site-packages:
$dir/install/cameras/lib/python3.10/site-packages:
$dir/install/arm/lib/python3.10/site-packages:
$dir/install/xbee_udp/lib/python3.10/site-packages:
$dir/install/basestation/lib/python3.10/site-packages:
$dir/install/can_udp/lib/python3.10/site-packages:
$dir/install/pathfinding/lib/python3.10/site-packages:
$dir/install/pointcloud_to_laserscan/lib/python3.10/site-packages:
$dir/install/autonomous/lib/python3.10/site-packages:${PYTHONPATH}"