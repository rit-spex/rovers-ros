#!/bin/bash
# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# Purpose  : Autorun the rover stack at boot.
#
# Installed as a systemd unit by scripts/autoboot/install.sh. It launches
# the real rover stack only (no Gazebo); use scripts/sim.sh for simulation.
#
# Usage:
#   autoboot.sh                 normal rover stack
#   autoboot.sh --maker-faire   rover stack plus arm_simulated_teensy_node
#
# --maker-faire is an autoboot-only demo profile; main_launch.xml is unchanged.
# ------------------------------------------------------------------
set -u

_usage() {
    echo "Usage: $0 [--maker-faire]"
}

_maker_faire=0
for _arg in "$@"; do
    case "$_arg" in
        --maker-faire) _maker_faire=1 ;;
        -h|--help) _usage; exit 0 ;;
        *)
            echo "Unknown option: $_arg" >&2
            _usage >&2
            exit 2
            ;;
    esac
done

_ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$_ws_dir" || exit 1

# shellcheck disable=SC1091
source scripts/source.sh || exit 1

if [ "$_maker_faire" -eq 1 ]; then
    exec ros2 launch "$_ws_dir/scripts/autoboot/maker_faire_launch.xml"
fi
exec ros2 launch main main_launch.xml
