#!/bin/sh
# Bring can0 back up for the rover bus.
#
# The bitrate is a property of the CAN interface, not a python-can Bus
# argument (the socketcan backend ignores bitrate= and only opens the
# device). It must be configured here with ip(8) before the node opens it.
sudo ip link set can0 down 2>/dev/null
if ! sudo ip link set can0 type can bitrate 1000000 berr-reporting on; then
    echo "reset_can: failed to configure can0 at 1 Mbit/s" >&2
    exit 1
fi
sudo ip link set can0 up
ip -details link show can0 | grep -o 'bitrate [0-9]*'
