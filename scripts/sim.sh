#!/usr/bin/env bash
#
# Rover simulation control.
#
#   ./scripts/sim.sh start [config]   full Gazebo + rover stack + basestation
#   ./scripts/sim.sh stop             stop everything
#   ./scripts/sim.sh restart [config] stop, then start
#   ./scripts/sim.sh reset [config]   same as restart
#   ./scripts/sim.sh stack [config]   rover ROS stack only (no Gazebo/GUI)
#   ./scripts/sim.sh view             open the RViz simulation view
#   ./scripts/sim.sh status           show running simulation processes
#
# Config values: drivebase, arm_drivebase, life_drivebase.
# The in-Gazebo Simulation Control panel can also change this and restart.

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"

usage() {
    cat <<'EOF'
Rover simulation control.

  ./scripts/sim.sh start [config]    full Gazebo + rover stack + basestation
  ./scripts/sim.sh stop              stop everything
  ./scripts/sim.sh restart [config]  stop, then start
  ./scripts/sim.sh reset [config]    same as restart
  ./scripts/sim.sh stack [config]    rover ROS stack only (no Gazebo/GUI)
  ./scripts/sim.sh view              open the RViz simulation view
  ./scripts/sim.sh status            show running simulation processes

Config values: drivebase, arm_drivebase, life_drivebase.
The in-Gazebo Simulation Control panel can also change this and restart.
EOF
}

open_tab() {
    local title="$1"
    local command="$2"
    local slug="${title//[^A-Za-z0-9_-]/_}"
    local pidfile="$TAB_DIR/$slug.pid"
    local runner="$TAB_DIR/$slug.sh"
    local kind

    # The first tab opens a window; later tabs join it. Using --window here
    # avoids attaching tabs to whatever terminal window happens to be last.
    if [ "${#TERM_ARGS[@]}" -eq 0 ]; then
        kind="--window"
    else
        kind="--tab"
    fi

    # Write the command to a runner script. gnome-terminal's -e expects one
    # self-contained argument, and -- cannot be used because it ends option
    # parsing for the whole command line (which drops the remaining tabs).
    cat > "$runner" <<EOF
#!/usr/bin/env bash
export ROVER_SIM_TAB='$slug'
echo \$\$ > '$pidfile'
trap 'rm -f "$pidfile"' EXIT
$command
exec bash
EOF
    chmod +x "$runner"

    TERM_ARGS+=("$kind" --title="$title" -e "bash $runner")
}

start_sim() {
    if [ "$#" -gt 0 ]; then
        ROBOT_CONFIG="$1"
    fi
    load_robot_config

    if ! command -v gnome-terminal >/dev/null 2>&1; then
        echo "gnome-terminal is required; install it first." >&2
        return 1
    fi
    if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
        echo "No graphical display found; run this from a desktop session." >&2
        return 1
    fi

    # Starting fresh: clear anything left over from a previous run so we do
    # not end up with two Gazebos and duplicate nodes.
    if any_matching "${SIM_PATTERNS[@]}"; then
        stop_sim
    fi

    build_workspace || return 1
    setup_gazebo || return 1

    local world_file="$WORLD"
    if [ ! -f "$world_file" ]; then
        world_file="$ROS_WS/install/rover_simulation/share/rover_simulation/worlds/$WORLD"
    fi
    if [ ! -f "$world_file" ]; then
        world_file="$ROS_WS/src/simulation/rover_simulation/worlds/$WORLD"
    fi
    if [ ! -f "$world_file" ]; then
        echo "[!] World '$WORLD' not found in rover_simulation/worlds." >&2
        echo "    Set WORLD to a file name in that directory." >&2
        return 1
    fi

    TAB_DIR="$SIM_STATE_DIR/tabs"
    mkdir -p "$TAB_DIR"
    # Drop state for tabs that are no longer alive; keep any tab still in use
    # (for example the terminal this command was started from).
    shopt -s nullglob
    for stale in "$TAB_DIR"/*.pid; do
        tab_pid "$stale" >/dev/null || rm -f "$stale" "${stale%.pid}.sh"
    done

    TERM_ARGS=()
    open_tab "Gazebo" \
        "cd '$ROS_WS' && source '$ROS_SETUP' && source install/setup.bash && \
source '$ROS_WS/scripts/common.sh' && setup_gazebo && \
ros2 launch rover_simulation launch_sim.launch.py \
world:='$WORLD' robot_config:='$ROBOT_CONFIG'"

    open_tab "Rover Stack" \
        "cd '$ROS_WS' && source '$ROS_SETUP' && source install/setup.bash && \
ros2 launch rover_simulation simulation_launch.xml robot_config:='$ROBOT_CONFIG'"

    if [ -d "$BASE_WS" ]; then
        open_tab "Base Station" \
            "cd '$BASE_WS' && \
if [ -x .venv/bin/python ]; then \
BASESTATION_SIMULATION=1 .venv/bin/python -m basestation \
--sim-inputs --real-spacemouse; \
else \
BASESTATION_SIMULATION=1 python3 -m basestation \
--sim-inputs --real-spacemouse; \
fi"
    else
        echo "[!] Basestation workspace not found at $BASE_WS; skipping its tab." >&2
    fi

    # One terminal window, one tab per launch.
    gnome-terminal "${TERM_ARGS[@]}"
}

# ---------------------------------------------------------------------------
# Tab helpers
# ---------------------------------------------------------------------------

tab_pid() {
    local pidfile="$1"
    local pid marker
    pid="$(cat "$pidfile" 2>/dev/null || true)"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    kill -0 "$pid" 2>/dev/null || return 1
    marker="$(basename "$pidfile" .pid)"
    [[ -r "/proc/$pid/environ" ]] || return 1
    tr '\0' '\n' < "/proc/$pid/environ" | grep -qx "ROVER_SIM_TAB=$marker" \
        || return 1
    printf '%s\n' "$pid"
}

close_tabs() {
    local pidfile slug pid kept=0
    TAB_DIR="${TAB_DIR:-$SIM_STATE_DIR/tabs}"
    shopt -s nullglob
    for pidfile in "$TAB_DIR"/*.pid; do
        slug="$(basename "$pidfile" .pid)"
        if pid="$(tab_pid "$pidfile")"; then
            # Never close the tab this script is running in.
            case "$PROTECTED_PIDS" in
                *" $pid "*)
                    echo "  keeping $slug tab (in use)"
                    kept=1
                    continue
                    ;;
            esac
            echo "  closing $slug tab (pid $pid)"
            kill -TERM "$pid" 2>/dev/null || true
        fi
        rm -f "$pidfile" "${pidfile%.pid}.sh"
    done
    [ "$kept" -eq 1 ] && return 0
    rm -rf "$TAB_DIR"
    rmdir "$SIM_STATE_DIR" 2>/dev/null || true
}

stop_sim() {
    echo "[+] Stopping simulation processes..."
    if ! any_matching "${SIM_PATTERNS[@]}"; then
        echo "[+] No simulation processes found."
        close_tabs
        return 0
    fi

    stop_matching_batch "${SIM_PATTERNS[@]}"
    close_tabs
    echo "[+] All simulations closed."
}

show_status() {
    local found=0 pattern pid
    local -A seen=()
    for pattern in "${SIM_PATTERNS[@]}"; do
        while read -r pid; do
            [ -n "$pid" ] || continue
            [ -n "${seen[$pid]:-}" ] && continue
            seen[$pid]=1
            found=1
            ps -o pid=,args= -p "$pid" | cut -c1-140
        done < <(matching_pids "$pattern")
    done
    [ "$found" -eq 1 ] || echo "No simulation processes running."
}

run_stack() {
    if [ "$#" -gt 0 ]; then
        ROBOT_CONFIG="$1"
    fi
    load_robot_config
    build_workspace || return 1
    exec ros2 launch rover_simulation simulation_launch.xml \
        robot_config:="$ROBOT_CONFIG"
}

run_view() {
    cd "$ROS_WS" || return 1
    if [ ! -f "$ROS_SETUP" ]; then
        echo "[!] ROS setup not found: $ROS_SETUP" >&2
        return 1
    fi
    # shellcheck disable=SC1090
    source "$ROS_SETUP" || return 1
    if [ ! -f install/setup.bash ]; then
        echo "[!] Workspace is not built: $ROS_WS/install/setup.bash" >&2
        return 1
    fi
    # shellcheck disable=SC1091
    source install/setup.bash || return 1

    local view_file
    view_file="$(ros2 pkg prefix rover_simulation)/share/rover_simulation/rviz/sim.rviz"
    if [ ! -f "$view_file" ]; then
        echo "[!] RViz view not found: $view_file" >&2
        echo "    Rebuild rover_simulation, then retry." >&2
        return 1
    fi
    exec rviz2 -d "$view_file" --ros-args -p use_sim_time:=true
}

command="${1:-start}"
[ "$#" -gt 0 ] && shift

case "$command" in
    start) start_sim "$@" ;;
    stop) stop_sim ;;
    restart|reset)
        stop_sim
        start_sim "$@"
        ;;
    stack) run_stack "$@" ;;
    view) run_view ;;
    status) show_status ;;
    help|-h|--help) usage ;;
    *)
        echo "Unknown command: $command" >&2
        usage
        exit 2
        ;;
esac
