# physicalai-plugins

A monorepo of third-party robot plugins for [PhysicalAI](https://github.com/openvinotoolkit/physicalai).

Every package provides concrete implementations of the `Robot` protocol
(no inheritance or registration required) and registers a robot catalog with
[PhysicalAI Studio](https://github.com/open-edge-platform/physical-ai-studio)
via an entry point. Packages are built and released independently with
[release-please](RELEASE.md).

## Packages

| Package                                                                                   | Description                                                    | Released          |
| ----------------------------------------------------------------------------------------- | -------------------------------------------------------------- | ----------------- |
| [`physicalai-common-extras`](packages/physicalai-common-extras/README.md)                 | Reusable action sources / callbacks (Composite, keyboard, ...) | not yet published |
| [`physicalai-lekiwi-plugin`](packages/physicalai-lekiwi-plugin/README.md)                 | LeKiwi mobile manipulator (6-DOF arm + 3-wheel holonomic base) | yes               |
| [`physicalai-rebot-b601-plugin`](packages/physicalai-rebot-b601-plugin/README.md)         | Seeed reBot B601 arm (B601-DM / B601-RS) + Star Arm 102 leader | yes               |
| [`physicalai-bimanual-so101-plugin`](packages/physicalai-bimanual-so101-plugin/README.md) | Bimanual SO-101 (twin 6-DOF STS3215 arms)                      | yes               |
| [`physicalai-lerobot-plugin`](packages/physicalai-lerobot-plugin/README.md)               | LeRobot robot/teleoperator adapter for the Studio catalog      | yes               |
| [`physicalai-mujoco-so101-plugin`](packages/physicalai-mujoco-so101-plugin/README.md)     | MuJoCo SO-101 simulation plugin for PhysicalAI Studio          | not yet published |

## Requirements

- Python 3.12+
- [uv](https://docs.astral.sh/uv/) (used for the workspace, locking, and tooling)
- Physical hardware per package (serial ports / CAN adapters), or the MuJoCo simulator

## Repository layout

```text
packages/
  physicalai-common-extras/     # shared action sources / callbacks
  physicalai-bimanual-so101-plugin/
  physicalai-lekiwi-plugin/
  physicalai-lerobot-plugin/
  physicalai-mujoco-so101-plugin/
  physicalai-rebot-b601-plugin/
docs/                          # guides (e.g. creating a plugin)
scripts/smoke.py               # import + version smoke test used by CI releases
.github/                       # CI, release-please config
```

## Installation

From the repo root, install the whole workspace (all plugins, dev tooling):

```bash
uv sync
```

To depend on a single package from another project:

```bash
uv add physicalai-lekiwi-plugin
```

## Running examples with the PhysicalAI CLI

The [PhysicalAI CLI](https://github.com/openvinotoolkit/physicalai) ships a
`run` subcommand that executes a `RobotRuntime` (read observation → ask an
action source → send action → tick) from a YAML config. This replaces the
hand-written control loops that used to live in `examples/`.

Run any of the bundled runtime configs from the repo root with:

```bash
uv run physicalai run --config <path-to-yaml>
```

Press `Ctrl+C` to stop. Optionally cap the run with
`--run.duration_s=60`.

### LeKiwi

Keyboard drive of the base (arm holds its position):

```bash
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/drive-keyboard.yaml
```

Composite teleoperation — leader arm positions the arm, keyboard drives the
base:

```bash
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/teleop.yaml
```

### reBot B601

Leader → follower teleoperation (Star Arm 102 leader to a B601-DM or B601-RS):

```bash
uv run physicalai run --config packages/physicalai-rebot-b601-plugin/examples/runtime/teleop-dm.yaml
uv run physicalai run --config packages/physicalai-rebot-b601-plugin/examples/runtime/teleop-rs.yaml
```

### Bimanual SO-101

Bimanual teleoperation with a leader BimanualSO101 (both arms):

```bash
uv run physicalai run --config packages/physicalai-bimanual-so101-plugin/examples/runtime/teleop.yaml
```

### LeRobot

Follower → leader teleoperation for any bundled LeRobot robot:

```bash
uv run physicalai run --config packages/physicalai-lerobot-plugin/examples/runtime/teleop.yaml
```

### MuJoCo SO-101

Self-relay of a running MuJoCo owner over Zenoh:

```bash
uv run python packages/physicalai-mujoco-so101-plugin/examples/run_mujoco_owner.py
uv run physicalai run --config packages/physicalai-mujoco-so101-plugin/examples/runtime/teleop.yaml
```

### Sinusoidal motion and joint reading

```bash
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/move-joints.yaml
uv run physicalai run --config packages/physicalai-lekiwi-plugin/examples/runtime/read-joints.yaml
uv run physicalai run --config packages/physicalai-rebot-b601-plugin/examples/runtime/move-joints-dm.yaml
uv run physicalai run --config packages/physicalai-rebot-b601-plugin/examples/runtime/read-joints-leader.yaml
```

The only remaining script is the MuJoCo **owner** process (`run_mujoco_owner.py`),
which starts the simulation server rather than a control loop.

## Action sources

The reusable
[action sources](https://github.com/openvinotoolkit/physicalai) shipped in the
[`physicalai-common-extras`](packages/physicalai-common-extras/README.md)
package can be wired into any `physicalai run` config (or used directly from
Python):

```python
from physicalai_common_extras import CompositeSource, KeyboardTeleop
```

| Source            | Purpose                                                |
| ----------------- | ------------------------------------------------------ |
| `KeyboardTeleop`  | WASD/QE base velocities; arm held at its observed pose |
| `CompositeSource` | Combine any number of sources (e.g. leader + keyboard) |

Keyboard controls (single characters, case-insensitive):

| Key       | Action               |
| --------- | -------------------- |
| `w` / `s` | forward / backward   |
| `a` / `d` | rotate left / right  |
| `q` / `e` | strafe left / right  |
| `space`   | stop (zero the base) |

## Development

```bash
uv sync                          # install workspace + dev tooling
uv run prek                      # lint, format, type check (ruff + pyrefly + hooks)
uv run pytest packages/*/tests/  # run all package test suites
```

## Release process

See [RELEASE.md](RELEASE.md) — packages are versioned from git tags via
`hatch-vcs` and published to PyPI by release-please.

## License

Apache-2.0 — see [LICENSE](LICENSE).
