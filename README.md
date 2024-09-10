# Rovers

This repository contains the source code for RIT's University Rover Challenge rover.

The software and firmware architecture is broken down into the following components and their respective responsibilities:

- Python high level ROS application (Jetson Nano)
    - Arm kinematics
    - Base station communication
    - Camera control
    - Drive base kinematics
    - Perception

- Embedded Firmware
    - Main Body Board (Teensy 4.1)
        - Drive base control
        - Temperature control

    - Science Board (Teensy 4.1)
        - Soil sampling
        - Flourometry

    - Arm Board (Teensy 4.1)
        - Joint control
        - End effector control

Each board communicates via Controller Area Network (CAN), utilizing the FlexCAN library for Teensy 4.1 and python-can on the Jetson Nano.