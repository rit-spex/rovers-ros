#!/bin/bash
# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# Purpose  : Autorun the rover stack at boot.
#
# Installed as a systemd unit by scripts/autoboot/install.sh. It launches
# the real rover stack only (no Gazebo); use scripts/sim.sh for simulation.
# ------------------------------------------------------------------
set -u

_ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$_ws_dir" || exit 1

# shellcheck disable=SC1091
source scripts/source.sh || exit 1

exec ros2 launch main main_launch.xml
