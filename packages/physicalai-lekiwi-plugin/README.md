# PhysicalAI LeKiwi Plugin

Third-party LeKiwi mobile manipulator plugin for [PhysicalAI](https://github.com/openvinotoolkit/physicalai).

Provides a concrete implementation of the `Robot` protocol for:

| Class    | Robot                                        | Motors                              | Protocol            |
| -------- | -------------------------------------------- | ----------------------------------- | ------------------- |
| `LeKiwi` | 6-DOF SO-ARM100 arm + 3-wheel holonomic base | Feetech STS3215 (via `scservo_sdk`) | POSITION / VELOCITY |

## Installation

```bash
uv add physicalai-lekiwi-plugin
```

`scservo_sdk` (Feetech servo serial SDK) is included as a core dependency.

> No calibration JSON is bundled. You must provide your own calibration file (LeRobot format) or use uncalibrated/ticks mode.

## Usage

### Basic follower (with calibration file)

```python
import numpy as np
from physicalai.robot import connect
from physicalai_lekiwi_plugin import LeKiwiCalibration, LeKiwi

robot = LeKiwi(
    port="/dev/ttyACM0",
    role="follower",
    calibration=LeKiwiCalibration.from_path("calibration.json"),
)

with connect(robot) as robot:
    obs = robot.get_observation()
    action = obs.joint_positions.copy()
    robot.send_action(action)
```

### Calibration from a dict

```python
cal = LeKiwiCalibration.from_dict({
    "arm_shoulder_pan":  {"id": 1, "drive_mode": 0, "homing_offset": 0,    "range_min": 0,    "range_max": 4095},
    "arm_shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": -512, "range_min": 512,  "range_max": 3583},
    "arm_elbow_flex":    {"id": 3, "drive_mode": 0, "homing_offset": 0,    "range_min": 0,    "range_max": 4095},
    "arm_wrist_flex":    {"id": 4, "drive_mode": 1, "homing_offset": 0,    "range_min": 0,    "range_max": 4095},
    "arm_wrist_roll":    {"id": 5, "drive_mode": 0, "homing_offset": 0,    "range_min": 0,    "range_max": 4095},
    "arm_gripper":       {"id": 6, "drive_mode": 0, "homing_offset": 0,    "range_min": 2048, "range_max": 3072},
    "base_left_wheel":   {"id": 7, "drive_mode": 0, "homing_offset": 0,    "range_min": 0,    "range_max": 4095},
    "base_back_wheel":   {"id": 8, "drive_mode": 0, "homing_offset": 0,    "range_min": 0,    "range_max": 4095},
    "base_right_wheel":  {"id": 9, "drive_mode": 0, "homing_offset": 0,    "range_min": 0,    "range_max": 4095},
})

robot = LeKiwi(port="/dev/ttyACM0", role="follower", calibration=cal)
```

### Uncalibrated bringup mode

```python
robot = LeKiwi.uncalibrated(port="/dev/ttyACM0", role="follower")
```

Observations and actions use raw servo ticks (0–4095). No calibration file needed.

### Leader (read-only teleoperation)

```python
robot = LeKiwi.uncalibrated(port="/dev/ttyACM0", role="leader")

with connect(robot) as robot:
    while True:
        obs = robot.get_observation()
        print(obs.joint_positions)
```

Leader mode disables torque on all motors so the arm can be moved manually.

### Keyboard WASD base control

```python
import numpy as np
from physicalai.robot import connect
from physicalai_lekiwi_plugin import LeKiwi

robot = LeKiwi.uncalibrated(port="/dev/ttyACM0", role="follower")

with connect(robot) as robot:
    print("WASD drive: W=forward, S=backward, A=rotate left, D=rotate right, Q=strafe left, E=strafe right")
    try:
        while True:
            key = input("cmd> ").strip().lower()
            vx = vy = vtheta = 0.0
            if key == "w":
                vx = 0.15
            elif key == "s":
                vx = -0.15
            elif key == "a":
                vtheta = 0.5
            elif key == "d":
                vtheta = -0.5
            elif key == "q":
                vy = 0.15
            elif key == "e":
                vy = -0.15
            elif key in ("", "x"):
                break

            action = np.zeros(9, dtype=np.float32)
            action[6:] = [vx, vy, vtheta]
            robot.send_action(action)
    except KeyboardInterrupt:
        pass

    robot.send_action(np.zeros(9, dtype=np.float32))
```

## URDF Model

```python
from physicalai_lekiwi_plugin import get_urdf_path

urdf_path = get_urdf_path()
```

| URDF                 | Model                     | Use                               |
| -------------------- | ------------------------- | --------------------------------- |
| `lekiwi/LeKiwi.urdf` | LeKiwi mobile manipulator | Kinematics & gravity compensation |

The URDF references original STL mesh files from the [SIGRobotics-UIUC/LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) project.

## Kinematics Model

The base uses a three-wheel holonomic drive with wheels at 240°, 0°, and 120°:

```python
# Body-frame velocities -> individual wheel raw speeds
wheel_raw = robot._body_to_wheel_raw(vx, vy, vtheta)

# Individual wheel raw speeds -> body-frame velocities
body_vel = robot._wheel_raw_to_body(left_raw, back_raw, right_raw)
```

| Parameter     | Value          |
| ------------- | -------------- |
| Wheel angles  | 240°, 0°, 120° |
| Wheel radius  | 0.05 m         |
| Base radius   | 0.125 m        |
| Max raw wheel | 3000           |

## Acknowledgments

The URDF model for the LeKiwi is from the [SIGRobotics-UIUC/LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) project.
