# physicalai-lerobot-plugin

`physicalai-lerobot-plugin` bridges
[LeRobot](https://github.com/huggingface/lerobot) robot and teleoperator
configs into the
[PhysicalAI](https://github.com/openvinotoolkit/physicalai) Studio catalog.

The intent of this package is to let Studio users select supported LeRobot
hardware from a schema-driven UI, auto-resolve serial devices, and run them
through a PhysicalAI-compatible adapter without writing robot-specific glue
code.

## Installation

```bash
uv add physicalai-lerobot-plugin
```

## Usage

The plugin registers one catalog entry for every installed LeRobot follower
robot and teleoperator. This includes first-party LeRobot types and compatible
third-party extensions.

Each entry is exposed as:

- follower type: `LeRobot_<follower_type>_Follower`
- leader type: `LeRobot_<teleoperator_type>_Leader`

### Third-party LeRobot extensions

Install this package first, then install a LeRobot extension into the same
Python environment. On Studio startup, the catalog invokes LeRobot's native
third-party discovery and imports installed extension packages with names that
start with `lerobot_robot_` or `lerobot_teleoperator_`.

For example, after installing a compatible LeSlider package, its registered
follower and leader types appear automatically in the Studio catalog. An
extension must import its config registration code from its package root and
use LeRobot's `register_subclass(...)` mechanism. Hardware SDK dependencies
remain the responsibility of the extension package.

### Bundled LeRobot followers

- `bi_so_follower`
- `bi_rebot_b601_follower`
- `bi_openarm_follower`
- `so100_follower`
- `so101_follower`
- `koch_follower`
- `omx_follower`
- `hope_jr_hand`
- `hope_jr_arm`
- `openarm_follower`
- `rebot_b601_follower`
- `reachy2`
- `earthrover_mini_plus`
- `unitree_g1`
- `lekiwi`
- `lekiwi_client`

### Bundled LeRobot leader teleoperators

- `so100_leader` (for `so100_follower`)
- `so101_leader` (for `so101_follower`)
- `koch_leader` (for `koch_follower`)
- `omx_leader` (for `omx_follower`)
- `openarm_leader` (for `openarm_follower`)
- `rebot_102_leader` (for `rebot_b601_follower`)
- `reachy2_teleoperator` (for `reachy2`)

### Notes

- Test-only robots are intentionally not registered by this plugin.
- Payload models are generated from LeRobot config dataclasses, so fields can
  differ by robot type.

### How payload fields are built

Payload classes are generated dynamically from each LeRobot config dataclass.

1. Fields are derived directly from the selected LeRobot config dataclass.
2. Complex fields are supported recursively:
   - nested dataclasses, `dict[...]`, `list[...]`, tuples, optional/union types
3. Required/default behavior:
   - fields without dataclass defaults are required
   - fields with defaults/factories are optional in payload
4. Runtime endpoint resolution:
   - `port` fields are resolved at build time through the Studio factory when
     possible (including nested bimanual arm configs)
   - payload schema itself does not add Studio-specific connection fields

### Common payload fields

There is no fixed cross-robot payload contract anymore. Fields come from the
selected LeRobot config type.

### Payload examples

#### 1) `LeRobot_so100_follower_Follower`

```json
{
  "port": "/dev/ttyACM0",
  "disable_torque_on_disconnect": true,
  "use_degrees": true,
  "id": "so100-main"
}
```

#### 2) `LeRobot_hope_jr_hand_Follower`

```json
{
  "port": "/dev/ttyACM1",
  "side": "left",
  "disable_torque_on_disconnect": true,
  "id": "hope-left-hand"
}
```

#### 3) `LeRobot_bi_so_follower_Follower` (nested bimanual config)

```json
{
  "left_arm_config": {
    "port": "/dev/ttyACM0",
    "disable_torque_on_disconnect": true
  },
  "right_arm_config": {
    "port": "/dev/ttyACM1",
    "disable_torque_on_disconnect": true
  },
  "id": "bi-so-main"
}
```

## Development

```bash
uv sync
uv run pytest
```

See [`docs/creating-a-studio-plugin.md`](../../docs/creating-a-studio-plugin.md) for the full plugin development guide.
