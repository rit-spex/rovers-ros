# Rovers ROS

ROS 2 workspace for the RIT SPEX rover (Scorpio). Runs on the Nvidia Jetson Orin Nano.

## Shared Protocol Submodule

This repository depends on the shared protocol package (`rovers-protocol`) for encoding/decoding messages from the basestation. The same protocol repo is used by both this repo and `rovers-basestation`.

```bash
# Clone with submodule
git clone --recurse-submodules https://github.com/rit-spex/rovers-ros.git

# Or if already cloned
git submodule update --init --recursive
```

Expected submodule location: `lib/rovers-protocol/`

### How it works

The basestation sends controller data over XBee radio as compact bit-packed bytes. The `basestation_node` decodes them using `MessageEncoder` from the shared protocol and publishes the values as ROS topics. The `telemetry_uplink_node` collects rover telemetry from ROS topics, encodes them using the same protocol, and sends them back to the basestation over UDP.

```
                        XBee Radio
  ┌─────────────┐   ───────────────>   ┌─────────────────┐
  │ BASESTATION  │   controller data   │   ROVER (ROS 2) │
  │ (Raspberry Pi)│  <───────────────  │  (this repo)     │
  │              │   telemetry (UDP)    │                  │
  └─────────────┘                      └─────────────────┘
```

### ROS nodes that use the protocol

| Node | File | Purpose |
|------|------|---------|
| `basestation_node` | `src/communications/basestation/basestation/basestation_node.py` | Decodes XBee messages → ROS topics |
| `telemetry_uplink_node` | `src/communications/basestation/basestation/telemetry_uplink_node.py` | ROS topics → UDP telemetry packets |

Import wrappers (`encoding.py`, `command_codes.py`) in the basestation and constants packages add `lib/rovers-protocol` to `sys.path` so the protocol works without pip-installing.

## Building

Make sure you have ROS 2 installed, then:

```bash
colcon build
source source.sh
```

## Running

```bash
# Launch the full system
ros2 launch main main_launch.xml

# Launch the simulator
ros2 launch main simulation_launch.xml
```

## Connection

SSH to the rover:
```bash
ssh rovers@129.21.91.140
# Password: rovers
```

Then:
```bash
cd ~/ros/rovers-ros
source source.zsh
ros2 launch main main_launch.xml
```

After you see "starting xbee..." enter the password `rovers`.

## Proof of Protocol Establishment

```
ROVER_PROTOCOL_TRACE=1
```
^ in the shell (probably want to do this for both basestation and ros shells)