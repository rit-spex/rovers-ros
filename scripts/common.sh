#!/usr/bin/env bash
# Shared paths and helpers for the rover simulation scripts.
#
# Source this file; do not execute it.

_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
# Derive the workspace from this file so the scripts work on the rover
# (~/ros/rovers-ros), a dev laptop, or CI without extra configuration.
ROS_WS="${ROS_WS:-$(cd "$_COMMON_DIR/.." && pwd)}"
BASE_WS="${BASE_WS:-$(cd "$_COMMON_DIR/../.." && pwd)/rovers-basestation}"
WORLD="${WORLD:-empty.world}"
SIM_STATE_DIR="${XDG_RUNTIME_DIR:-/tmp}/rovers-sim"
CONFIG_FILE="${XDG_CONFIG_HOME:-$HOME/.config}/rovers-ros/sim_config"

# Processes that belong to the simulation. The workspace install path catches
# every node started by colcon; the extra names catch helpers launched from
# /opt/ros that `ros2 launch` may orphan when it is killed.
SIM_PATTERNS=(
    "ros2 launch rover_simulation launch_sim.launch.py"
    "ros2 launch rover_simulation simulation_launch.xml"
    "ros2 launch main main_launch.xml"
    "armSimulator4\\.py"
    "python[0-9.]* -m basestation"
    "gz sim"
    "gzserver"
    "gzclient"
    "robot_state_publisher"
    "static_transform_publisher"
    "pointcloud_to_laserscan"
    "controller_manager/spawner"
    "$ROS_WS/install/"
)

# Subset used by the in-Gazebo restart. The external basestation is a separate
# process and is intentionally left running.
SIM_STACK_PATTERNS=(
    "ros2 launch rover_simulation launch_sim.launch.py"
    "ros2 launch rover_simulation simulation_launch.xml"
    "armSimulator4\\.py"
    "gz sim"
    "gzserver"
    "gzclient"
    "robot_state_publisher"
    "static_transform_publisher"
    "pointcloud_to_laserscan"
    "controller_manager/spawner"
    "$ROS_WS/install/"
)

load_robot_config() {
    if [ -z "${ROBOT_CONFIG:-}" ] && [ -f "$CONFIG_FILE" ]; then
        IFS= read -r ROBOT_CONFIG <"$CONFIG_FILE" || true
    fi
    ROBOT_CONFIG="${ROBOT_CONFIG:-arm_drivebase}"
    case "$ROBOT_CONFIG" in
        drivebase|arm_drivebase|life_drivebase) ;;
        *)
            echo "Unknown ROBOT_CONFIG '$ROBOT_CONFIG'; using arm_drivebase" >&2
            ROBOT_CONFIG=arm_drivebase
            ;;
    esac
    export ROBOT_CONFIG
}

# Remove build/install trees whose CMake cache points at a source directory
# that no longer exists.  This happens when a package is moved in the repo
# (for example src/autonomous -> src/subsystems/autonomous): colcon reuses the
# old build directory and CMake aborts with "source directory ... does not
# exist".  Only the affected package is cleaned, so the rest of the workspace
# stays warm.
clean_stale_cmake_builds() {
    local cache pkg source_dir
    for cache in "$ROS_WS"/build/*/CMakeCache.txt; do
        [ -f "$cache" ] || continue
        source_dir="$(sed -n 's/^CMAKE_HOME_DIRECTORY:[^=]*=//p' "$cache" | head -n 1)"
        [ -n "$source_dir" ] || continue
        [ -d "$source_dir" ] && continue

        pkg="$(basename "$(dirname "$cache")")"
        echo "[+] Removing stale build cache for $pkg"
        echo "    cached source: $source_dir"
        rm -rf "$ROS_WS/build/$pkg" "$ROS_WS/install/$pkg"
    done
}

# Remove symlinks left behind when a source file or whole package was moved
# or deleted.  setuptools/--symlink-install keeps pointing at the old path,
# and the next build fails trying to copy it (for example a removed
# twist_mux.yaml under build/rover_simulation/config).
clean_stale_build_links() {
    [ -d "$ROS_WS/build" ] || return 0
    find "$ROS_WS/build" -xtype l -delete 2>/dev/null || true
}

build_workspace() {
    if [ ! -f "$ROS_SETUP" ]; then
        echo "[!] ROS setup not found: $ROS_SETUP" >&2
        echo "    Set ROS_SETUP to your ROS 2 setup.bash." >&2
        return 1
    fi
    # shellcheck disable=SC1090
    source "$ROS_SETUP" || return 1
    cd "$ROS_WS" || return 1
    clean_stale_cmake_builds
    clean_stale_build_links
    echo "[+] Building rover workspace..."
    # Keep this list in sync with simulation_launch.xml and
    # launch_sim.launch.py. Missing packages here means `sim.sh start` can
    # launch stale or unbuilt code on a fresh checkout. gazebo_sim_control is
    # optional at CMake level: it installs an empty package when Gazebo is
    # absent, so this list is safe on hardware-only machines too.
    colcon build \
        --packages-select \
        constants custom_interfaces \
        basestation can_udp xbee_udp chassis arm main \
        rover_simulation gazebo_sim_control pointcloud_to_laserscan \
        pathfinding \
        --symlink-install || return 1
    # shellcheck disable=SC1091
    source install/setup.bash
}

setup_gazebo() {
    cd "$ROS_WS" || return 1
    if [ ! -f install/setup.bash ]; then
        echo "[!] Workspace is not built: $ROS_WS/install/setup.bash" >&2
        return 1
    fi
    # shellcheck disable=SC1091
    source install/setup.bash || return 1

    # Always needed: gzclient loads the workspace GUI plugin, and the panel
    # uses ROVER_WS to find restart_gazebo.sh.
    export ROVER_WS="$ROS_WS"
    export GAZEBO_PLUGIN_PATH="$ROS_WS/install/gazebo_sim_control/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"

    # gazebo_ros2_control supplies libgazebo_ros2_control.so and is a system
    # dependency declared in rover_simulation/package.xml. Fail early with a
    # clear install instruction instead of letting Gazebo fail to load it.
    if ! ros2 pkg prefix gazebo_ros2_control >/dev/null 2>&1; then
        echo "[!] gazebo_ros2_control is not installed." >&2
        echo "    Install it with: sudo apt install ros-${ROS_DISTRO:-humble}-gazebo-ros2-control" >&2
        return 1
    fi
}

# ---------------------------------------------------------------------------
# Process cleanup helpers
# ---------------------------------------------------------------------------

# PIDs of this script and its ancestors. They must never be stopped, even if
# their command line happens to match one of the patterns.
protected_pids() {
    local pid="$$" parent out=" "
    while [ -n "$pid" ] && [ "$pid" -gt 0 ] 2>/dev/null; do
        out="$out$pid "
        parent="$(awk '{print $4}' "/proc/$pid/stat" 2>/dev/null)" || break
        [ "$parent" = "$pid" ] && break
        pid="$parent"
    done
    printf '%s' "$out"
}

PROTECTED_PIDS="$(protected_pids)"

matching_pids() {
    local pattern="$1" pid
    for pid in $(pgrep -f "$pattern" 2>/dev/null); do
        case "$PROTECTED_PIDS" in
            *" $pid "*) continue ;;
        esac
        printf '%s\n' "$pid"
    done
}

any_matching() {
    local pattern
    for pattern in "$@"; do
        if [ -n "$(matching_pids "$pattern")" ]; then
            return 0
        fi
    done
    return 1
}

stop_matching() {
    local signal="$1" pattern pid
    shift
    for pattern in "$@"; do
        for pid in $(matching_pids "$pattern"); do
            kill "-$signal" "$pid" 2>/dev/null || true
        done
    done
}

# Stop every process matching the supplied patterns. Try a clean interrupt
# first, then escalate. Returns as soon as nothing matches.
stop_matching_batch() {
    if ! any_matching "$@"; then
        return 0
    fi
    stop_matching INT "$@"
    for _ in $(seq 1 20); do
        any_matching "$@" || return 0
        sleep 0.25
    done
    stop_matching TERM "$@"
    sleep 2
    any_matching "$@" || return 0
    stop_matching KILL "$@"
    sleep 1
}
