#!/bin/sh
# Bring can0 back up for the rover bus.
#
# The bitrate is a property of the CAN interface, not a python-can Bus
# argument (the socketcan backend ignores bitrate= and only opens the
# device). It must be configured here with ip(8) before the node opens it.
#
# `sudo -n` is deliberate: autoboot has no terminal, so a password prompt
# would hang or fail the node. Install the narrow passwordless rule once:
#
#   ./scripts/can/install.sh

_run_ip() {
    if [ "$(id -u)" -eq 0 ]; then
        ip "$@"
    else
        sudo -n ip "$@"
    fi
}

_run_ip link set can0 down >/dev/null 2>&1 || true
if ! _run_ip link set can0 type can bitrate 1000000 berr-reporting on; then
    echo "reset_can: failed to configure can0 at 1 Mbit/s" >&2
    if [ "$(id -u)" -ne 0 ]; then
        echo "  If sudo asked for a password, install the one-time rule:" >&2
        echo "    ./scripts/can/install.sh" >&2
    fi
    exit 1
fi
_run_ip link set can0 up || exit 1
ip -details link show can0 | grep -o 'bitrate [0-9]*'
