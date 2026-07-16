# PhysicalAI Bimanual SO-101 Plugin

Third-party bimanual SO-101 robot arm plugin for [PhysicalAI](https://github.com/openvinotoolkit/physicalai).

Composes two SO-101 arms (left + right) behind the `Robot` protocol.

| Class           | Robot                   | Motors          | Protocol |
| --------------- | ----------------------- | --------------- | -------- |
| `BimanualSO101` | Dual 6-DOF STS3215 arms | Feetech STS3215 | POSITION |

## Installation

```bash
uv add physicalai-bimanual-so101-plugin
```

`feetech-servo-sdk` is included as a core dependency.

## Usage

### Calibrated follower

```python
import numpy as np
from physicalai.robot import connect
from physicalai_bimanual_so101_plugin import BimanualSO101

# Build left and right arms manually
from physicalai.robot.so101 import SO101, SO101Calibration

left_cal = SO101Calibration.from_path("left_calibration.json")
right_cal = SO101Calibration.from_path("right_calibration.json")

left = SO101(port="/dev/ttyACM0", calibration=left_cal, role="follower")
right = SO101(port="/dev/ttyACM1", calibration=right_cal, role="follower")

robot = BimanualSO101(left=left, right=right)

with connect(robot) as arm:
    obs = arm.get_observation()     # shape (12,)
    action = obs.joint_positions.copy()
    arm.send_action(action)
```

### Uncalibrated bringup mode

```python
from physicalai.robot import connect
from physicalai_bimanual_so101_plugin import BimanualSO101
from physicalai.robot.so101 import SO101

left = SO101.uncalibrated(port="/dev/ttyACM0", role="follower")
right = SO101.uncalibrated(port="/dev/ttyACM1", role="follower")

robot = BimanualSO101(left=left, right=right)
```

### Leader (read-only teleoperation)

```python
left = SO101.uncalibrated(port="/dev/ttyACM0", role="leader")
right = SO101.uncalibrated(port="/dev/ttyACM1", role="leader")

robot = BimanualSO101(left=left, right=right)

with connect(robot) as arm:
    while True:
        obs = arm.get_observation()
        print(obs.joint_positions)
```

## URDF Model

```python
from physicalai_bimanual_so101_plugin import get_urdf_path

urdf_path = get_urdf_path()
```

| URDF                         | Model           | Use                        |
| ---------------------------- | --------------- | -------------------------- |
| `so101_dual/so101_dual.urdf` | Bimanual SO-101 | Kinematics & visualization |

Two arms mounted 0.4 m apart: left at +0.2 m (Y), right at -0.2 m (Y).

## Joint Order

Left arm (indices 0-5): `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`

Right arm (indices 6-11): same as above, prefixed with `right_`.

## Acknowledgments

URDF model generated from Onshape via onshape-to-robot.
SO-101 arm by [LeRobot](https://github.com/huggingface/lerobot).
