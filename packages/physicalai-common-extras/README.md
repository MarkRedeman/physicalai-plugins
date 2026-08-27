# PhysicalAI Common Extras

Reusable [action sources](https://github.com/openvinotoolkit/physicalai) and
runtime callbacks for PhysicalAI plugins. Part of the
[physicalai-plugins](https://github.com/MarkRedeman/physicalai-plugins) monorepo.

These classes implement the `physicalai.runtime.ActionSource` protocol and can
be wired into any `physicalai run --config` runtime YAML (or used directly in
Python).

## Sources

| Source                                 | Purpose                                                                                    |
| -------------------------------------- | ------------------------------------------------------------------------------------------ |
| `CompositeSource` / `CompositeChannel` | Combine any number of action sources, each filling a subset of the action by joint indices |
| `KeyboardTeleop`                       | WASD/QE base velocities; arm held at its observed pose                                     |
| `SineWaveSource`                       | Sinusoidal joint targets (per-joint amplitudes/phases)                                     |
| `HoldPoseSource`                       | Echo the current observation (hold / read-only)                                            |
| `JointLogger`                          | Print observed joint positions each tick (runtime callback)                                |

## CompositeSource

Drive one robot from multiple action sources — e.g. a leader arm plus a
keyboard base, or a policy for one arm and a teleop leader for the other.
Each channel's `source` fills the action entries given by its `indices`; the
channels must cover every joint exactly once.

```python
from physicalai.runtime import TeleopSource
from physicalai_common_extras import CompositeChannel, CompositeSource, KeyboardTeleop

composite = CompositeSource(
    channels=[
        CompositeChannel(source=TeleopSource(leader=leader_arm), indices=[0, 1, 2, 3, 4, 5]),
        CompositeChannel(source=KeyboardTeleop(), indices=[6, 7, 8]),
    ],
)
```

### Runtime config

```yaml
runtime:
  robot: { class_path: ..., init_args: { ... } }
  action_source:
    class_path: physicalai_common_extras.CompositeSource
    init_args:
      channels:
        - class_path: physicalai_common_extras.CompositeChannel
          init_args:
            source:
              class_path: physicalai.runtime.TeleopSource
              init_args:
                leader: { class_path: ..., init_args: { ... } }
            indices: [0, 1, 2, 3, 4, 5]
        - class_path: physicalai_common_extras.CompositeChannel
          init_args:
            source:
              class_path: physicalai_common_extras.KeyboardTeleop
            indices: [6, 7, 8]
  fps: 30.0
```

## Keyboard controls

| Key       | Action               |
| --------- | -------------------- |
| `w` / `s` | forward / backward   |
| `a` / `d` | rotate left / right  |
| `q` / `e` | strafe left / right  |
| `space`   | stop (zero the base) |

## Development

```bash
uv sync
uv run pytest packages/physicalai-common-extras/tests/
```
