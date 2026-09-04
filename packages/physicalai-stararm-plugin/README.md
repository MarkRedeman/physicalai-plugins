# PhysicalAI Star Arm Plugin

Fashion Star Star Arm 102 plugin for [PhysicalAI](https://github.com/openvinotoolkit/physicalai). This package provides Star Arm 102-LD/102-HD leaders and a Star Arm 102-FL follower, and registers all variants with [Physical AI Studio](https://github.com/open-edge-platform/physical-ai-studio).

[![PyPI version](https://img.shields.io/pypi/v/physicalai-stararm-plugin.svg)](https://pypi.org/project/physicalai-stararm-plugin/)
[![Python versions](https://img.shields.io/pypi/pyversions/physicalai-stararm-plugin.svg)](https://pypi.org/project/physicalai-stararm-plugin/)

## Features

- Star Arm 102-LD and 102-HD leader drivers (`StarArm102LDLeader`, `StarArm102HDLeader`) for read-only teleoperation input
- Star Arm 102-FL follower driver (`StarArm102FLFollower`) for position control
- Studio catalog plugin entries for both HD leader and FL follower
- Bundled `stararm102` URDF package

## Installation

```bash
uv add physicalai-stararm-plugin
```

## Quick start

```python
from physicalai.robot import connect
from physicalai_stararm_plugin import StarArm102FLFollower

robot = StarArm102FLFollower(port="/dev/ttyUSB1", baudrate=1_000_000)

with connect(robot) as arm:
    obs = arm.get_observation()
    arm.send_action(obs.joint_positions)
```

## Run with the PhysicalAI CLI

```bash
uv run physicalai run --config packages/physicalai-stararm-plugin/examples/runtime/teleop-hd-to-fl.yaml
uv run physicalai run --config packages/physicalai-stararm-plugin/examples/runtime/teleop-ld-to-fl.yaml
uv run physicalai run --config packages/physicalai-stararm-plugin/examples/runtime/read-joints-hd.yaml
uv run physicalai run --config packages/physicalai-stararm-plugin/examples/runtime/read-joints-ld.yaml
uv run physicalai run --config packages/physicalai-stararm-plugin/examples/runtime/read-joints-fl.yaml
uv run physicalai run --config packages/physicalai-stararm-plugin/examples/runtime/move-joints-fl.yaml
```

## URDF

```python
from physicalai_stararm_plugin import get_urdf_path

urdf_dir = get_urdf_path()
star_urdf = urdf_dir / "stararm102" / "urdf" / "stararm102_description.urdf"
```

## Development

```bash
uv sync
uv run pytest packages/physicalai-stararm-plugin/tests/
```
