#!/usr/bin/env bash
#
# Restart only Gazebo and the rover stack with a new robot config.
# Used by the in-Gazebo Simulation Control panel.
#
# The external basestation process is left running; this script stops the
# Gazebo GUI and the rover stack, rebuilds changed packages, and starts them
# again. Logs are written under $SIM_STATE_DIR/logs.
#
# Usage: ./restart_gazebo.sh [drivebase|arm_drivebase|life_drivebase]

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"

if [ "$#" -gt 0 ]; then
    ROBOT_CONFIG="$1"
fi
load_robot_config
CONFIG="$ROBOT_CONFIG"

LOG_DIR="$SIM_STATE_DIR/logs"
mkdir -p "$LOG_DIR"
BUILD_LOG="$LOG_DIR/build.log"
GAZEBO_LOG="$LOG_DIR/gazebo.log"
STACK_LOG="$LOG_DIR/stack.log"

# Rebuild before stopping anything, so a build failure leaves the current
# simulation running. The panel can then be used to fix the issue and retry.
if ! build_workspace >"$BUILD_LOG" 2>&1; then
    echo "Build failed; see $BUILD_LOG" >&2
    exit 1
fi
if ! setup_gazebo >>"$BUILD_LOG" 2>&1; then
    echo "Gazebo setup failed; see $BUILD_LOG" >&2
    exit 1
fi
cd "$ROS_WS" || exit 1

if any_matching "${SIM_STACK_PATTERNS[@]}"; then
    echo "[+] Stopping Gazebo and the rover stack..."
    stop_matching_batch "${SIM_STACK_PATTERNS[@]}"
    # Give the GUI a moment to release its rendering context. Relaunching in
    # the same instant can make the new gzclient trip a Gazebo camera
    # assertion (boost::shared_ptr<Camera> is null).
    sleep 1
fi

setsid ros2 launch rover_simulation launch_sim.launch.py \
    world:="$WORLD" robot_config:="$CONFIG" \
    >"$GAZEBO_LOG" 2>&1 &
setsid ros2 launch rover_simulation simulation_launch.xml \
    robot_config:="$CONFIG" \
    >"$STACK_LOG" 2>&1 &

echo "Restarted Gazebo and rover stack with $CONFIG"
echo "Logs: $GAZEBO_LOG and $STACK_LOG"
