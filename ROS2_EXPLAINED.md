# ROS 2 Explained

A practical tour of the ROS 2 concepts this workspace uses, in the order you
need them: the runtime, nodes, packages, topics, messages, subscribers,
publishers, timers, QoS, parameters, launch files, and process lifecycle.

## 1. Explanation: the big picture

ROS 2 is a set of libraries and conventions for building a distributed robot
application. It is not an operating system and it is not a central server.
Processes called **nodes** discover each other over DDS and exchange typed
messages directly.

A robot stack is therefore a graph:

```text
  inputs / transports              subsystem logic                 hardware
  ------------------              -----------------               --------
  xbee_node  ──/XBEE/MESSAGES──►  basestation_node
                                         │
                                         ├──/BASESTATION/*──►  chassis_node
                                         └──/ARM/COMMAND────►  arm_master_node
                                                                    │
                                                                    ├──► arm_route_planner
                                                                    └──► arm_end_effector

  chassis_node ──CAN/TX/CHASSIS/Drive_Power──► can_comms_node ──CAN──► Teensy
  Teensy ──CAN──► can_comms_node ──CAN/RX/*──► master / telemetry ──► telemetry_uplink
```

- **Transports** move bytes: `xbee_node` handles the radio, `can_comms_node`
  speaks SocketCAN to the Teensy, and `basestation_node` turns protocol frames
  into ordinary ROS topics.
- **Subsystem nodes** own behavior: `chassis_node` converts joystick axes to
  drive power, while the arm nodes split command arbitration, planning, and the
  end effector.
- **Composition** is declarative: `main/launch/main_launch.xml` starts the real
  rover, while `simulation/rover_simulation/launch/simulation_launch.xml`
  swaps the transports and hardware drivers for Gazebo equivalents.

This workspace uses topics for every interaction; there are no services or
actions in the runtime graph today.

## 2. Nodes

A **node** is the basic unit of computation. It is a process with a name,
publishers, subscriptions, timers, and callbacks.

In Python, a node subclasses `rclpy.node.Node`:

```python
from rclpy.node import Node


class Chassis(Node):
    def __init__(self):
        super().__init__("chassis_node")
```

The string passed to `super().__init__()` is the node name shown by
`ros2 node list`.

Each node should have one clear responsibility. In this repo:

| Node type | Examples | Responsibility |
|---|---|---|
| Transport | `xbee_node`, `can_comms_node` | Move bytes between ROS and hardware |
| Protocol bridge | `basestation_node`, `telemetry_uplink_node` | Convert packed protocol frames to/from ROS messages |
| Subsystem | `chassis_node`, `arm_route_planner_node` | Own robot behavior |
| Composition | `master_node`, launch files | Coordinate sources and start the stack |

A node's constructor wires the graph. It does **not** block or run the control
loop:

```python
self.create_subscription(Float32, topic, callback, qos)
self.create_publisher(MessageType, topic, qos)
self.create_timer(period_sec, callback)
```

Work happens in callbacks, driven by `rclpy.spin()`.

## 3. Packages and executables

A package is the unit of distribution and building. Python nodes use
`ament_python`; C++ and message-definition packages use `ament_cmake`.

From the directory that should own the node:

```bash
cd src/subsystems
ros2 pkg create \
  --build-type ament_python \
  --dependencies rclpy std_msgs \
  example
```

A minimal package looks like:

```text
example/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/example
├── example/
│   ├── __init__.py
│   └── example_node.py
├── launch/
│   └── example_launch.xml
└── test/
```

`package.xml` declares dependencies:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

`setup.py` turns a Python function into the executable name used by
`ros2 run` and `ros2 launch`:

```python
from glob import glob

from setuptools import find_packages, setup

setup(
    name="example",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/example"]),
        ("share/example", ["package.xml"]),
        ("share/example/launch", glob("launch/*.xml")),
    ],
    entry_points={
        "console_scripts": [
            "example_node = example.example_node:main",
        ],
    },
)
```

The part before `=` is the executable name used as `exec` in launch files.
The part after `:` is the function `ros2 run` calls.

Build one package with:

```bash
colcon build --packages-select example --symlink-install
source scripts/source.sh
```

## 4. Topics

A **topic** is a named, typed channel. Publishers write to it; subscribers read
from it. They do not know about each other.

Topic names in this repo encode their owner and direction:

```text
/ARM/CMD_VELOCITY
/ARM/STATE/ERROR
/BASESTATION/xbox/AXIS_LY
CAN/TX/CHASSIS/Drive_Power
CAN/RX/CHASSIS/Drive_Status
```

Useful CLI commands:

```bash
ros2 topic list
ros2 topic info /ARM/CMD_VELOCITY
ros2 topic echo /ARM/STATE/ERROR
ros2 topic hz /ARM/CMD_VELOCITY
```

A topic can have many publishers and many subscribers. The graph is
discovered dynamically, so starting a node late is normal. If late joiners
must immediately see the last value, use transient-local QoS (Section 9).

Topics are best for streams and state. This workspace does not currently use
services or actions; everything is publish/subscribe.

## 5. Messages

Every topic has one message type. The type defines the fields on the wire.

Standard messages come from packages such as `std_msgs`:

```python
from std_msgs.msg import Bool, Float32

Float32(data=0.5)
Bool(data=True)
```

Custom messages live in `src/custom_interfaces/msg/`. For example,
`ArmCommand` is used for operator actions and `ArmVelocity` for teleop
velocity. Import them like any other message:

```python
from custom_interfaces.msg import ArmCommand, ArmVelocity

command = ArmCommand(command="home")
velocity = ArmVelocity()
velocity.linear_x = 0.1
```

Inspect a definition with:

```bash
ros2 interface show custom_interfaces/msg/ArmCommand
ros2 interface show std_msgs/msg/Float32
```

Always publish the type the subscriber expects. A `Float32` published on a
topic that another node reads as `Bool` is a type mismatch and will not
connect.

## 6. Subscribers

A subscriber registers a callback for one topic and message type:

```python
self.create_subscription(
    msg_type,
    topic,
    callback,
    qos_profile,
)
```

Real example from `chassis_node`:

```python
self.create_subscription(
    Float32,
    "/BASESTATION/xbox/AXIS_LY",
    self.__LY_callback,
    TELEOP_QOS,
)


def __LY_callback(self, msg: Float32):
    if self.__estop:
        return
    self.__LY_value = -msg.data
    self.__drive_power_can_message.signals["left"].set_value(self.__LY_value)
    self.__publish_drive_power()
```

Rules for callbacks:

- Keep them short. Do not `sleep`, wait on hardware, or run a long loop.
- Latch input state, then let a timer or another callback act on it.
- The message object is only valid during the callback. Copy fields you need
  later (`self.__value = float(msg.data)`), rather than storing `msg` itself.
- Do not create publishers or subscriptions repeatedly inside a callback.

Subscriptions are registered once in the constructor. `rclpy.spin()` then
delivers messages whenever they arrive.

## 7. Publishers

A publisher is created once and reused:

```python
self.__publisher = self.create_publisher(
    msg_type,
    topic,
    qos_profile,
)
```

Publish with:

```python
self.__publisher.publish(msg)
```

Real example from `chassis_node`:

```python
def __publish_drive_power(self):
    can_packet = TeensyCommunication.encode_can_message(
        self.__drive_power_can_message)
    self.__publishers[CAN_MESSAGE_IDS.DRIVE_POWER].publish(can_packet)
```

A publish only puts the message into the local middleware; it does not wait
for a subscriber. If nothing is subscribed, the message is simply dropped
unless durability is configured.

Create the message once when it is a reusable command buffer, or construct a
fresh one per publish when the fields change shape. Do not call
`create_publisher()` inside a loop.

## 8. Timers and callbacks

`create_timer(period_sec, callback)` schedules periodic work:

```python
self.create_timer(0.05, self.__publish)
```

This is the ROS-native replacement for:

```python
while True:
    work()
    time.sleep(0.05)
```

The timer callback is dispatched by the executor, so it composes with
subscriptions instead of blocking them.

In this repo, control loops use the shared `DT = 0.05` period:

- `arm_route_planner_node` updates the kinematic model each `DT`.
- `arm_end_effector_node` advances a held gripper jog each `DT`.
- `arm_master_node` republishes control ownership and checks the planner
  watchdog each `DT`.

If a callback can run concurrently with another callback, protect shared
state or use a single-threaded executor's default behavior. This repo's nodes
are written for the default executor and keep callbacks small.

## 9. QoS

QoS controls how messages are delivered. The common settings in this
workspace:

| Setting | Meaning | Used for |
|---|---|---|
| Reliable, depth 10 | Retry delivery; keep a small queue | Commands and state |
| Best effort, depth 10 | Drop rather than retry | Teleop and radio streams |
| Transient local, depth 1 | Keep the last value for late joiners | `/ARM/STATE/CONTROL_SOURCE` |

Examples:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

TELEOP_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
```

```python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

CONTROL_SOURCE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
```

Use best effort for streams where the newest sample matters more than every
sample. Use reliable for commands where dropping a message changes behavior.
Use transient local for state that a late-starting node must see immediately.

## 10. Parameters versus constants

ROS parameters let a launch file change a value without editing code:

```python
self.declare_parameter("publish_rate_hz", 50.0)
rate = float(self.get_parameter("publish_rate_hz").value)
```

Use a parameter only when deployment actually varies it. Do not add
parameters just to expose every internal number.

This repo keeps shared tuning in Python constants modules, for example
`src/subsystems/arm/arm_constants.py`. `arm_teleop_node` declares no ROS
parameters because its sensitivity, deadzone, timeout, and publish rate are
defined once in `arm_constants` and no launch file overrides them.

Rule of thumb:

- Cross-node values and physics/kinematics tuning: constants module.
- Values that differ between real rover, simulation, or deployments: ROS
  parameter with a launch-file override.
- Per-callback scratch state: plain Python attributes on the node.

## 11. Launch files

A launch file starts a configured set of nodes. It contains no behavior:

```xml
<launch>
  <node pkg="example" exec="example_node" name="example_node"/>
</launch>
```

- `pkg` is the package name.
- `exec` is the `console_scripts` name from `setup.py`.
- `name` is the runtime node name.

Launch files can pass arguments and environment variables:

```xml
<launch>
  <arg name="protocol_trace" default="0"/>
  <set_env name="ROVER_PROTOCOL_TRACE" value="$(var protocol_trace)"/>
  <node pkg="basestation" exec="basestation_node" name="basestation"/>
</launch>
```

The two composition roots in this repo are:

- `src/main/launch/main_launch.xml` — real rover, real CAN and XBee.
- `src/simulation/rover_simulation/launch/simulation_launch.xml` — Gazebo and
  simulated transports.

Run either with:

```bash
ros2 launch main main_launch.xml
ros2 launch rover_simulation simulation_launch.xml
```

## 12. Lifecycle and shutdown

Every executable should use the same structure:

```python
def main():
    rclpy.init()
    node = None
    try:
        node = ExampleNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

Why this shape:

- `rclpy.init()` creates the ROS context.
- `rclpy.spin(node)` dispatches callbacks and blocks until shutdown.
- `KeyboardInterrupt` and `ExternalShutdownException` are the normal Ctrl-C
  paths; handle both.
- `spin()` must be inside the `try`. If it is outside, Ctrl-C escapes as an
  unhandled traceback.
- `destroy_node()` releases publishers, subscriptions, and timers.
- `rclpy.shutdown()` closes the context; only call it while `rclpy.ok()`.

Hardware nodes also release their device in cleanup. `xbee_node` removes its
data callback and closes the radio; `can_comms_node` sends a safe stop, stops
the CAN notifier, and shuts the bus down. Background driver threads should
check `rclpy.ok()` or a shutdown flag before publishing, so they do not log
into a dead context during Ctrl-C.

## 13. Creating a node, step by step

### 13.1 Create the package

```bash
cd src/subsystems
ros2 pkg create \
  --build-type ament_python \
  --dependencies rclpy std_msgs \
  example
```

Move it under the subsystem that owns it, for example
`src/subsystems/chassis/example`.

### 13.2 Add the executable

In `setup.py`:

```python
entry_points={
    "console_scripts": [
        "example_node = example.example_node:main",
    ],
},
```

### 13.3 Write the node

```python
"""Minimal timer-driven node with an E-stop latch."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class ExampleNode(Node):
    """Publish an input value unless E-stop is asserted."""

    def __init__(self):
        super().__init__("example_node")
        self.__estop = False
        self.__value = 0.0

        self.__publisher = self.create_publisher(
            Float32, "/EXAMPLE/OUTPUT", 10)
        self.create_subscription(
            Float32, "/EXAMPLE/INPUT", self.__on_input, 10)
        self.create_subscription(
            Bool, "/ESTOP", self.__on_estop, 10)
        self.create_timer(0.05, self.__publish)

    def __on_input(self, msg: Float32):
        if not self.__estop:
            self.__value = float(msg.data)

    def __on_estop(self, msg: Bool):
        self.__estop = bool(msg.data)
        if self.__estop:
            self.__value = 0.0

    def __publish(self):
        self.__publisher.publish(Float32(data=self.__value))


def main():
    rclpy.init()
    node = None
    try:
        node = ExampleNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

What to notice:

- The constructor creates the graph once.
- Inputs latch state; the timer decides when output is published.
- E-stop zeroes the output but leaves the node alive, so it can recover.
- Cleanup is in `finally`.

### 13.4 Add the launch entry

```xml
<launch>
  <node pkg="example" exec="example_node" name="example_node"/>
</launch>
```

### 13.5 Build and run

```bash
cd /path/to/rovers-ros
colcon build --packages-select example --symlink-install
source scripts/source.sh

ros2 run example example_node
ros2 launch example example_launch.xml
```

### 13.6 Test the logic

Keep pure logic in functions or classes that do not import `rclpy`, then test
them with pytest. The arm package is the reference: `AxisShaper`, the control
source policy, and the velocity gate are all tested without a ROS runtime.

```bash
colcon test --packages-select example
```

## 14. Build, run, and inspect

Build and source:

```bash
colcon build --symlink-install
source scripts/source.sh
```

Useful inspection commands:

```bash
ros2 node list
ros2 node info /chassis_node

ros2 topic list
ros2 topic info /ARM/CMD_VELOCITY
ros2 topic echo /ARM/STATE/ERROR
ros2 topic hz /ARM/CMD_VELOCITY

ros2 interface show custom_interfaces/msg/ArmCommand
ros2 param list /arm_teleop_node
```

`--symlink-install` keeps Python package files linked to the source tree, so
editing a node does not require a rebuild. New executables and message
definitions still require a build.

## 15. Repository conventions

- Name nodes and executables `snake_case`; node names match the process role
  (`chassis_node`, `arm_master_node`).
- Prefix internal attributes with `__` and keep callbacks small.
- Use timers for periodic work; do not block inside a callback.
- Create publishers, subscriptions, and timers once in the constructor.
- Put topic strings and cross-node tuning in a shared constants module rather
  than repeating literals.
- Use `BEST_EFFORT` for teleop/radio streams and reliable QoS for commands.
- Keep `main()` and shutdown handling identical across nodes so Ctrl-C behaves
  the same everywhere.
- Clean up hardware in `destroy_node()` or an explicit shutdown method.
- Add a launch file only for composition; behavior belongs in the node.
- Test pure logic without a ROS runtime wherever possible.
