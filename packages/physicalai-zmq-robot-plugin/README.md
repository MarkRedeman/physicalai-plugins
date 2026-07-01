# physicalai-zmq-robot-plugin

A ZMQ-based remote robot client implementing the [PhysicalAI](https://github.com/openvinotoolkit/physicalai) `Robot` protocol.

## Installation

```bash
uv add physicalai-zmq-robot-plugin
```

## Usage

```python
from physicalai.robot import connect
from physicalai_zmq_robot_plugin import ZMQRobot

robot = ZMQRobot("tcp://192.168.1.100:5555")

with connect(robot) as r:
    obs = r.get_observation()
    print(f"Joint positions: {obs.joint_positions}")
    r.send_action(obs.joint_positions)
```

## Protocol

Expects a ZMQ REP server that accepts JSON REQ messages with:

- `{"command": "read_state", "payload": {}}` → returns current joint state
- `{"command": "set_joints_state", "payload": {"joints": {...}, "goal_time": ...}}` → sets joint targets
- `{"command": "features", "payload": {}}` → returns available joint names
- `{"command": "enable_torque" | "disable_torque", "payload": {}}` → torque control
- `{"command": "ping", "payload": {}}` → health check
