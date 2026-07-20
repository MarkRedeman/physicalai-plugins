# physicalai-rebot-b601-plugin

Third-party Seeed reBot B601 robot arm plugin for [PhysicalAI](https://github.com/openvinotoolkit/physicalai).

Provides concrete implementations of the `Robot` protocol for:

| Class               | Arm              | Motors                                           | Protocol                     |
| ------------------- | ---------------- | ------------------------------------------------ | ---------------------------- |
| `ReBotB601DM`       | B601-DM follower | Damiao (via `motorbridge`)                       | POS_VEL / FORCE_POS          |
| `ReBotB601RS`       | B601-RS follower | RobStride (via `motorbridge`)                    | MIT mode + gripper impedance |
| `ReBotArm102Leader` | Arm 102 leader   | FashionStar UART (via `motorbridge-smart-servo`) | Read-only                    |

## Installation

```bash
uv add physicalai-rebot-b601-plugin
```

`motorbridge` and `motorbridge-smart-servo` are included as core dependencies.

## Usage

```python
import numpy as np
from physicalai.robot import Robot, connect
from physicalai_rebot_b601_plugin import ReBotB601DM

robot = ReBotB601DM(port="/dev/ttyACM0", can_adapter="damiao")

with connect(robot) as arm:
    obs = arm.get_observation()
    action = obs.joint_positions.copy()
    arm.send_action(action)
```

All classes satisfy `isinstance(robot, Robot)` — no inheritance or registration
required. Use with `physicalai.robot.connect` and `physicalai.robot.verify_robot`.

## URDF Models

Bundled URDF descriptions for gravity compensation and kinematics:

```python
from physicalai_rebot_b601_plugin import get_urdf_path

urdf_dir = get_urdf_path()

# B601-DM / fixend arm (for gravity compensation)
dm_urdf = urdf_dir / "rebot-b601-dm" / "urdf" / "reBot-DevArm_fixend.urdf"

# B601-RS arm
rs_urdf = urdf_dir / "rebot-b601-rs" / "urdf" / "00-arm-rs_asm-v3.urdf"

# Star Arm 102 (leader)
star_urdf = urdf_dir / "stararm102" / "urdf" / "stararm102_description.urdf"
```

| URDF            | Model            | Use                                    |
| --------------- | ---------------- | -------------------------------------- |
| `rebot-b601-dm` | B601-DM (fixend) | Gravity compensation for `ReBotB601DM` |
| `rebot-b601-rs` | B601-RS v3       | Kinematics for `ReBotB601RS`           |
| `stararm102`    | Star Arm 102     | Kinematics for `ReBotArm102Leader`     |

## Acknowledgments

URDF models for the reBot Arm B601 are from the
[reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) project,
released under the MIT License by vectorBH6.

The Star Arm 102 URDF is from the
[Star-Arm-102](https://github.com/servodevelop/Star-Arm-102) project.
