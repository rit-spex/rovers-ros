# Rovers ROS

This is the ROS package for our 2024 - 2025 rover, Scorpio. This runs on our Nvidia Jetson Orin Nano.

## Building

To build the project, first make sure you have ROS2 installed.

Next, build the workspace with:
```bash
colcon build
```

Finally, source the environment:
```bash
source source.sh
```

## Running

To run the project, launch the main package with the command:
```bash
ros2 launch main main_launch.xml
```

To run the simulator,
```bash
ros2 launch main simulation_launch.xml
```

How to launch the rover:

## Connection
To ssh to the rover
```bash
ssh rovers@129.21.91.140
```

If it asks for password enter "rovers"

Open a new terminal and enter the following commands

cd ~/ros/rovers-ros
source source.zsh
ros2 launch main main_launch.xml

After you see "starting xbee..." enter the password rovers
