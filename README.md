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
or
```bash
source source.zsh
```
depending on your shell.

## Running

To run the project, launch the main package with the command:
```bash
ros2 launch main main_launch.xml
```
