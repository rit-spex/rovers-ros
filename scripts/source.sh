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
_loaded="ROS 2 (${ROS_DISTRO:-unknown})"
_build_info=""

if [ -f "$_ws_dir/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$_ws_dir/install/setup.bash"
    _loaded="$_loaded + overlay"

    _build_ref="$_ws_dir/log/latest_build"
    [ -e "$_build_ref" ] || _build_ref="$_ws_dir/install/setup.bash"

    # Collect date information, so that you are always aware of last build time
    _build_epoch="$(date -r "$_build_ref" +%s 2>/dev/null)"
    if [ -n "$_build_epoch" ]; then
        _build_time="$(date -d "@$_build_epoch" "+%Y-%m-%d %H:%M:%S" 2>/dev/null)"
        _diff=$(( $(date +%s) - _build_epoch ))
        [ "$_diff" -lt 0 ] && _diff=0

        if [ "$_diff" -lt 60 ]; then
            _ago=$'\033[1;32m'"${_diff}s ago"$'\033[0m'
        elif [ "$_diff" -lt 3600 ]; then
            _ago=$'\033[1;33m'"$((_diff / 60))m ago"$'\033[0m'
        elif [ "$_diff" -lt 43200 ]; then
            _ago=$'\033[1;33m'"$((_diff / 3600))h $(( (_diff % 3600) / 60 ))m ago"$'\033[0m'
        elif [ "$_diff" -lt 86400 ]; then
            _ago=$'\033[1;31m'"$((_diff / 3600))h $(( (_diff % 3600) / 60 ))m ago"$'\033[0m'
        else
            _ago=$'\033[1;31m'"$((_diff / 86400))d ago"$'\033[0m'
        fi

        _build_info="$_build_time ($_ago)"
    fi
else
    echo "[!] Workspace not built yet (missing install/setup.bash)"
fi

if [ -f "$_ws_dir/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$_ws_dir/.venv/bin/activate"
    _loaded="$_loaded + .venv"
fi

# Display message to user, with useful command reminders
echo ''
echo $'|\033[1;38;5;208m============ Rover ROS2 Sourcing Summary ============\033[0m|'
printf '|  %-46s [\033[1;32m✓\033[0m] |\n' "Sourced     |  $_loaded"
printf '|  Last build  |  %-46s |\n' "${_build_info:-$'\033[1;31mNot built yet\033[0m'}"
echo $'|\033[1;38;5;208m============ Rover ROS2 Useful Commands =============\033[0m|'
echo '|  Build       |  colcon build --symlink-install      |'
echo '|  Launch      |  ros2 launch main main_launch.xml    |'
echo '|  Run Node    |  ros2 run <pkg> <executable>         |'
echo '|  List Nodes  |  ros2 node list                      |'
echo '|  List Topics |  ros2 topic list -t                  |'
echo '|  Echo Topic  |  ros2 topic echo <topic>             |'
echo $'|\033[1;38;5;208m=====================================================\033[0m|'
echo ''

unset _ws_dir _ros_setup _loaded _build_ref _build_epoch _build_time _diff _ago _build_info