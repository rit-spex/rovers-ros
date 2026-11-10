#!/usr/bin/env bash
# Source the workspace environment.
#
#   source scripts/source.sh
#
# Works from any directory. Sources ROS 2, the colcon overlay, and the
# optional Python virtual environment when present.

_ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [ ! -f "$_ros_setup" ]; then
    echo "[!] ROS setup not found: $_ros_setup" >&2
    echo "    Set ROS_SETUP to your ROS 2 setup.bash." >&2
    return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1090
source "$_ros_setup"
if [ -f "$_ws_dir/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$_ws_dir/install/setup.bash"
fi
if [ -f "$_ws_dir/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$_ws_dir/.venv/bin/activate"
fi

unset _ws_dir _ros_setup
