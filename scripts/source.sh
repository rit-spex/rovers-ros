#!/usr/bin/env bash
# Source the workspace environment.
#
#   source scripts/source.sh
#
# Works from any directory. Sources ROS 2, the colcon overlay, and the
# optional Python virtual environment when present.
#
# The workspace is located from this file's own path, so the script does not
# have to be sourced from the workspace root. ROS_SETUP is honoured only when
# it points at a real file; a stale value falls back to a standard ROS 2
# underlay instead of breaking the overlay.

# ---------------------------------------------------------------------------
# Locate this script, wherever and however it was sourced.
# ---------------------------------------------------------------------------
if [ -z "${BASH_VERSION:-}" ] && [ -z "${ZSH_VERSION:-}" ]; then
    echo "[!] scripts/source.sh must be sourced from bash or zsh." >&2
    echo "    Use: source scripts/source.sh" >&2
    return 1 2>/dev/null || exit 1
fi

_shell_kind="sh"
[ -n "${BASH_VERSION:-}" ] && _shell_kind="bash"
[ -n "${ZSH_VERSION:-}" ] && _shell_kind="zsh"

# Resolve a setup script to the flavour for the shell running this script.
# ament and colcon generate .bash, .sh and .zsh variants; the .bash flavour
# locates itself through BASH_SOURCE, which is empty in zsh and silently
# resolves to the current directory (the usual "no such file: .../setup.sh"
# failure). Never hand a .bash script to zsh.
_resolve_setup() {
    _rs_dir="$(dirname "$1")"
    _rs_stem="$(basename "$1")"
    _rs_stem="${_rs_stem%%.*}"
    case "$_rs_stem" in
        setup | local_setup) ;;
        *)
            [ -f "$1" ] && { printf '%s' "$1"; return 0; }
            return 1
            ;;
    esac
    for _rs_ext in "$_shell_kind" sh; do
        if [ -f "$_rs_dir/$_rs_stem.$_rs_ext" ]; then
            printf '%s' "$_rs_dir/$_rs_stem.$_rs_ext"
            return 0
        fi
    done
    if [ "$_shell_kind" = "bash" ] && [ -f "$1" ]; then
        printf '%s' "$1"
        return 0
    fi
    return 1
}

if [ -n "${BASH_SOURCE[0]:-}" ]; then
    _ws_src="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
    # zsh exposes the sourced file's path through %N; bash never expands this
    # branch, so the zsh-only syntax is safe here.
    _ws_src="${(%):-%N}"
else
    _ws_src="${0:-}"
fi

if [ -n "$_ws_src" ] && [ -e "$_ws_src" ]; then
    _ws_dir="$(cd "$(dirname "$_ws_src")/.." >/dev/null 2>&1 && pwd -P)"
fi

# Fall back to the current directory when the script path cannot be resolved
# (piped source, deleted file, wrapper scripts). Only trust it if it actually
# looks like this workspace.
if [ -z "${_ws_dir:-}" ] && { [ -f "$PWD/src/main/package.xml" ] || [ -f "$PWD/install/setup.bash" ]; }; then
    _ws_dir="$(pwd -P)"
    echo "[!] Could not resolve the workspace from scripts/source.sh;" >&2
    echo "    assuming the current directory: $_ws_dir" >&2
fi

if [ -z "${_ws_dir:-}" ] || [ ! -d "$_ws_dir" ]; then
    echo "[!] Could not locate the rover workspace." >&2
    echo "    Run: source /path/to/rovers-ros/scripts/source.sh" >&2
    unset _ws_src _ws_dir
    return 1 2>/dev/null || exit 1
fi

# ---------------------------------------------------------------------------
# Pick the ROS 2 underlay. A ROS_SETUP that no longer exists is a common
# leftover after moving or rebuilding a ROS install, so treat it as a hint
# rather than a hard requirement.
# ---------------------------------------------------------------------------
_ros_setup=""
for _cand in "${ROS_SETUP:-}" "/opt/ros/${ROS_DISTRO:-}/setup.bash" /opt/ros/humble/setup.bash; do
    [ -n "$_cand" ] || continue
    if _resolved="$(_resolve_setup "$_cand")"; then
        _ros_setup="$_resolved"
        break
    fi
done

if [ -z "$_ros_setup" ]; then
    echo "[!] No ROS 2 setup file found for $_shell_kind." >&2
    if [ -n "${ROS_SETUP:-}" ]; then
        echo "    ROS_SETUP points at '${ROS_SETUP}', which does not exist." >&2
    fi
    echo "    Install ROS 2 or set ROS_SETUP to its setup.bash/setup.zsh." >&2
    unset _ws_src _ws_dir _cand _resolved _ros_setup _shell_kind
    unset -f _resolve_setup
    return 1 2>/dev/null || exit 1
fi

if [ -n "${ROS_SETUP:-}" ] && [ ! -f "$ROS_SETUP" ]; then
    echo "[!] ROS_SETUP='$ROS_SETUP' does not exist; using $_ros_setup instead." >&2
fi
# Keep the environment consistent for scripts/common.sh and scripts/sim.sh,
# which read ROS_SETUP directly.
export ROS_SETUP="$_ros_setup"

# shellcheck disable=SC1090
if ! source "$_ros_setup"; then
    echo "[!] Failed to source ROS 2 from $_ros_setup" >&2
    echo "    If this keeps happening, run the script from a bash shell:" >&2
    echo "    bash -c 'source scripts/source.sh; exec bash'" >&2
    unset _ws_src _ws_dir _ros_setup _cand _resolved _shell_kind
    unset -f _resolve_setup
    return 1 2>/dev/null || exit 1
fi

_loaded="ROS 2 (${ROS_DISTRO:-unknown})"
_build_info=""
_overlay=""

# ---------------------------------------------------------------------------
# Source the workspace overlay. Accept local_setup.bash as well, so a build
# made without a chained setup script is still usable.
# ---------------------------------------------------------------------------
for _cand in "$_ws_dir/install/setup.bash" "$_ws_dir/install/local_setup.bash"; do
    if _resolved="$(_resolve_setup "$_cand")"; then
        _overlay="$_resolved"
        break
    fi
done

if [ -n "$_overlay" ]; then
    # shellcheck disable=SC1090
    if source "$_overlay"; then
        _loaded="$_loaded + overlay"

        _build_ref="$_ws_dir/log/latest_build"
        [ -e "$_build_ref" ] || _build_ref="$_overlay"

        # Collect date information, so you are always aware of last build time
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
        _overlay=""
        echo "[!] Failed to source $_resolved" >&2
        echo "    Rebuild with: cd $_ws_dir && colcon build --symlink-install" >&2
    fi
fi

if [ -z "$_overlay" ]; then
    echo "[!] Workspace not built yet (missing $_ws_dir/install/setup.bash)"
    echo "    Build with: cd $_ws_dir && colcon build --symlink-install"
fi

if [ -f "$_ws_dir/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    if source "$_ws_dir/.venv/bin/activate"; then
        _loaded="$_loaded + .venv"
    else
        echo "[!] Could not activate $_ws_dir/.venv" >&2
    fi
fi

# Shorten long paths so the summary box keeps its shape.
_short() {
    if [ "${#1}" -le 46 ]; then
        printf '%s' "$1"
    else
        printf '...%s' "${1:$((${#1} - 43))}"
    fi
}

# Display message to user, with useful command reminders
echo ''
echo $'|\033[1;38;5;208m============ Rover ROS2 Sourcing Summary ============\033[0m|'
printf '|  %-46s [\033[1;32m✓\033[0m] |\n' "Sourced     |  $_loaded"
printf '|  %-46s [\033[1;32m✓\033[0m] |\n' "Workspace   |  $(_short "$_ws_dir")"
printf '|  %-46s [\033[1;32m✓\033[0m] |\n' "ROS 2       |  $(_short "$_ros_setup")"
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

unset _ws_src _ws_dir _ros_setup _cand _resolved _overlay _loaded _build_ref _build_epoch _build_time _diff _ago _build_info
unset _shell_kind
unset -f _short _resolve_setup 2>/dev/null || true
