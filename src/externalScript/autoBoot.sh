#!/bin/bash
# Autorun ROS on startup for the rover.

_ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$_ws_dir" || exit 1
# shellcheck disable=SC1091
source scripts/source.sh
ros2 launch main main_launch.xml
