# physicalai-lerobot-plugin

An adapter plugin that wraps [LeRobot](https://github.com/huggingface/lerobot)-compatible robots into the
[PhysicalAI](https://github.com/openvinotoolkit/physicalai) Robot protocol so they can be driven from
**Physical AI Studio**.

## Installation

```bash
uv add physicalai-lerobot-plugin
```

## Usage

The plugin registers `LeRobot_Follower` and `LeRobot_Leader` catalog entries with Studio.
Configure a robot by providing its type, serial port, and joint order in the Studio UI payload.

### Supported robot types

- `so100_follower` — SO-100 arm (6-DOF shoulder+elbow+wrist+gripper)
- `so101_follower` — SO-101 arm (6-DOF shoulder+elbow+wrist+gripper)

### Payload fields

| Field                          | Type        | Required | Default                     | Description                                  |
| ------------------------------ | ----------- | -------- | --------------------------- | -------------------------------------------- |
| `robot_type`                   | `str`       | Yes      | —                           | LeRobot robot name (e.g. `"so100_follower"`) |
| `port`                         | `str`       | Yes      | —                           | Serial port path (e.g. `"/dev/ttyACM0"`)     |
| `joint_order`                  | `list[str]` | Yes      | —                           | Joint names in PhysicalAI order              |
| `obs_position_keys`            | `list[str]` | No       | `["{name}.pos", ...]`       | Keys in the LeRobot observation dict         |
| `act_position_keys`            | `list[str]` | No       | same as `obs_position_keys` | Keys in the LeRobot action dict              |
| `disable_torque_on_disconnect` | `bool`      | No       | `True`                      | Whether to disable torque on disconnect      |
| `serial_number`                | `str`       | No       | `""`                        | Serial number for port discovery             |

## Development

```bash
uv sync
uv run pytest
```

See `docs/creating-a-studio-plugin.md` for the full plugin development guide.
