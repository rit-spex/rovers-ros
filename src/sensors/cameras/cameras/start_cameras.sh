#!/bin/bash

# Wait for cameras to start up
sleep 5

# Launch Rover Camera
nvgstcapture-1.0 --camsrc=0 --cap-dev-node=0
