# physicalai-websocket-robot-plugin

A WebSocket-based remote robot client implementing the [PhysicalAI](https://github.com/openvinotoolkit/physicalai) `Robot` protocol.

## Installation

```bash
uv add physicalai-websocket-robot-plugin
```

## Usage

```python
from physicalai.robot import connect
from physicalai_websocket_robot_plugin import WebSocketRobot

robot = WebSocketRobot("ws://192.168.1.100:8765")

with connect(robot) as r:
    obs = r.get_observation()
    print(f"Joint positions: {obs.joint_positions}")
    r.send_action(obs.joint_positions)
```

## Protocol

Expects a WebSocket server that accepts JSON messages with:

- `{"event": "read_state", "payload": {}}` → returns current joint state
- `{"event": "set_joints_state", "payload": {"joints": {...}, "goal_time": ...}}` → sets joint targets
- `{"event": "features", "payload": {}}` → returns available joint names
- `{"event": "enable_torque" | "disable_torque", "payload": {}}` → torque control
- `{"event": "ping", "payload": {}}` → health check

The server may push `joints_state_was_updated` events at any time.
