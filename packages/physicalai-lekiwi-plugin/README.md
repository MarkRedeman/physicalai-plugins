# PhysicalAI LeKiwi Plugin

Third-party LeKiwi mobile manipulator plugin for [PhysicalAI](https://github.com/openvinotoolkit/physicalai), the Python library and runtime for robot control, transport, and CLI workflows. It registers with [Physical AI Studio](https://github.com/open-edge-platform/physical-ai-studio), the application that discovers catalog plugins and provides robot setup, teleoperation, and workflow experiences. Part of the [physicalai-plugins](https://github.com/MarkRedeman/physicalai-plugins) monorepo.

[![PyPI version](https://img.shields.io/pypi/v/physicalai-lekiwi-plugin.svg)](https://pypi.org/project/physicalai-lekiwi-plugin/)
[![Python versions](https://img.shields.io/pypi/pyversions/physicalai-lekiwi-plugin.svg)](https://pypi.org/project/physicalai-lekiwi-plugin/)

## Features

- Concrete implementation of the `Robot` protocol — no inheritance or registration required
- 6-DOF SO-ARM100 arm + 3-wheel holonomic base, in normalized or raw-ticks units
- Follower and leader (read-only) roles for teleoperation
- Bundled URDF for kinematics and gravity compensation
- `KeyboardTeleop` and `CompositeSource` action sources for `physicalai run`

## Hardware

| Class    | Robot                                        | Motors                              | Protocol            |
| -------- | -------------------------------------------- | ----------------------------------- | ------------------- |
| `LeKiwi` | 6-DOF SO-ARM100 arm + 3-wheel holonomic base | Feetech STS3215 (via `scservo_sdk`) | POSITION / VELOCITY |

## Screenshots

_Placeholder images — replace them with real screenshots._

![LeKiwi in the PhysicalAI Studio robot catalog](https://raw.githubusercontent.com/MarkRedeman/physicalai-plugins/main/screenshots/studio-catalog.png)

![Connecting to a LeKiwi in PhysicalAI Studio](https://raw.githubusercontent.com/MarkRedeman/physicalai-plugins/main/packages/physicalai-lekiwi-plugin/screenshots/studio.png)

![LeKiwi composite teleop in the PhysicalAI CLI](https://raw.githubusercontent.com/MarkRedeman/physicalai-plugins/main/packages/physicalai-lekiwi-plugin/screenshots/cli-teleop.png)

## Installation

```bash
uv add physicalai-lekiwi-plugin
```

`scservo_sdk` (Feetech servo serial SDK) is included as a core dependency.

> No calibration JSON is bundled. You must provide your own calibration file (LeRobot format) or use uncalibrated/ticks mode.

## Quick start

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

In Studio catalog payloads, `calibration_id` is optional. When provided, the
plugin loads that calibration and runs in normalized units. When omitted, the
driver falls back to explicit uncalibrated `ticks` mode.

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

## Run with the PhysicalAI CLI

The [PhysicalAI CLI](https://github.com/openvinotoolkit/physicalai) `run`
subcommand executes a `RobotRuntime` from a YAML config. Run the bundled
configs from the repo root:

```bash
# Keyboard drive of the base (arm holds its position)
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/drive-keyboard.yaml

# Composite teleop: leader arm positions the arm, keyboard drives the base
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/teleop.yaml
```

Press `Ctrl+C` to stop. Optionally cap the run with `--run.duration_s=60`.

### Keyboard controls

| Key       | Action               |
| --------- | -------------------- |
| `w` / `s` | forward / backward   |
| `a` / `d` | rotate left / right  |
| `q` / `e` | strafe left / right  |
| `space`   | stop (zero the base) |

### Action sources

The runtime configs above use action sources from the
[`physicalai-common-extras`](../physicalai-common-extras/README.md) package
(installed from git, not PyPI):

```python
from physicalai_common_extras import CompositeSource, KeyboardTeleop
```

| Source            | Purpose                                                |
| ----------------- | ------------------------------------------------------ |
| `KeyboardTeleop`  | WASD/QE base velocities; arm held at its observed pose |
| `CompositeSource` | Combine any number of sources (e.g. leader + keyboard) |

### Sinusoidal motion and joint reading

```bash
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/move-joints.yaml
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/read-joints.yaml
```

## URDF Model

```python
from physicalai_lekiwi_plugin import get_urdf_path

urdf_path = get_urdf_path()
```

| URDF                      | Model                     | Use                               |
| ------------------------- | ------------------------- | --------------------------------- |
| `lekiwi/urdf/LeKiwi.urdf` | LeKiwi mobile manipulator | Kinematics & gravity compensation |

The URDF references original STL mesh files from the [SIGRobotics-UIUC/LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) project.

## Kinematics Model

The base uses a three-wheel holonomic drive with wheels at 240°, 0°, and 120°:

```python
# Body-frame velocities -> individual wheel raw speeds
wheel_raw = robot._body_to_wheel_raw(vx, vy, vtheta)

# Individual wheel raw speeds -> body-frame velocities
body_vel = robot._wheel_raw_to_body(left_raw, back_raw, right_raw)
```

The helper methods above are internal; prefer the `Robot` protocol
(`get_observation` / `send_action`) in application code.

| Parameter     | Value          |
| ------------- | -------------- |
| Wheel angles  | 240°, 0°, 120° |
| Wheel radius  | 0.05 m         |
| Base radius   | 0.125 m        |
| Max raw wheel | 3000           |

## Development

```bash
uv sync
uv run pytest packages/physicalai-lekiwi-plugin/tests/
```

## Acknowledgments

The URDF model for the LeKiwi is from the [SIGRobotics-UIUC/LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) project.
