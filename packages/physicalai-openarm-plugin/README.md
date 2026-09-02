# PhysicalAI OpenArm Plugin

Direct [SocketCAN](https://docs.kernel.org/networking/can.html)/Damiao control for
[OpenArm](https://github.com/enactic/openarm) followers and hand-guided leaders
in [PhysicalAI](https://github.com/openvinotoolkit/physicalai). The plugin also
registers single-arm and bimanual OpenArm entries in Physical AI Studio.

## Safety and support

OpenArm moves powerful physical hardware. Secure each arm, provide clearance,
keep an emergency stop accessible, and follow OpenArm's official safety guide.
Only Linux SocketCAN is supported. Exactly one process may control a CAN
interface at a time: do not concurrently run this plugin with LeRobot, ROS 2,

This first release assumes hardware setup has already been completed. It does
not set motor IDs, write persistent parameters, calibrate, or zero encoders.
Before connecting, configure the documented motor IDs, provision CAN/CAN-FD,
arm and closed-gripper procedure.

## Installation

```bash
uv add physicalai-openarm-plugin
```

The package requires `python-can` and Linux SocketCAN. OpenArm's default
configuration uses CAN-FD at 1 Mbps arbitration and 5 Mbps data rate.

## Runtime contract

Positions are **degrees**, in this fixed order:

```text
joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, gripper
```

Followers require `side: left` or `side: right`; the plugin applies the
side-specific conservative limits used by LeRobot's OpenArm support. Leaders
are read-only and default to torque-disabled manual control. Native bilateral
force feedback is not implemented.

## Run

Update the `can0` and `can1` placeholders, then run unilateral teleoperation:

```bash
uv run physicalai run --config packages/physicalai-openarm-plugin/examples/runtime/teleop.yaml
```

For two leaders and two followers on independent CAN interfaces:

```bash
uv run physicalai run --config packages/physicalai-openarm-plugin/examples/runtime/bimanual-teleop.yaml
```

## Studio assets

The plugin includes lightweight parallel-link gripper URDF visualization models
for Studio. They are plugin-authored kinematic visualizations, not redistributed
OpenArm CAD or mesh assets. OpenArm software is Apache-2.0; hardware design data
has separate CERN-OHL-S-2.0 licensing.

## Development

```bash
uv sync
uv run pytest packages/physicalai-openarm-plugin/tests/
```
