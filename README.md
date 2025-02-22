# Rovers ROS

This is the ROS package for our 2024 - 2025 rover, Scorpio. This runs on our Nvidia Jetson Orin Nano.

## Building

To build the project, first source the environment:
```bash
source source.sh
```
or
```bash
source source.zsh
```
depending on your shell.

Then you can run:
```bash
colcon build
```

## Running

To run the project, launch the main package with the command:
```bash
ros2 launch main main_launch.xml
```
