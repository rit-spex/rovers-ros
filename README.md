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
  ┌───────────────┐   ───────────────> ┌─────────────────┐
  │ BASESTATION   │   controller data  │  ROVER (ROS 2)  │
  │ (Raspberry Pi)│  <───────────────  │  (this repo)    │
  │               │   telemetry (UDP)  │                 │
  └───────────────┘                    └─────────────────┘
```

### ROS nodes that use the protocol

| Node | File | Purpose |
|------|------|---------|
| `basestation_node` | `src/communications/basestation/basestation/basestation_node.py` | Decodes XBee messages → ROS topics |
### Protocol Trace Debugging

You can enable real-time hex-level protocol tracing to verify that the rover is correctly encoding/decoding messages. This is useful for confirming bit-packed message IDs and field values match what the basestation expects.

To enable tracing, pass the `protocol_trace:=1` argument:

```bash
# In simulation
ros2 launch main simulation_launch.xml protocol_trace:=1

# On hardware (SSH in and run)
ros2 launch main main_launch.xml protocol_trace:=1
```

When enabled, the `basestation_node` and `telemetry_uplink_node` will print hex traces like:
- `[protocol rx] <ID:0x02> data:0200ff...` (Xbox control message)
- `[protocol tx] <ID:0xF1> data:f1045a...` (Telemetry message)

---

## Building

Make sure you have ROS 2 (Humble) installed, then:

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