if [ ! -f ./install/local_setup.sh ]; then
	source /opt/ros/humble/local_setup.sh
	colcon build
fi
source source.sh
