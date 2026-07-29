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

The plugin registers one catalog entry per supported LeRobot follower robot,
plus leader entries for follower types that have a matching LeRobot
teleoperator.

Each entry is exposed as:

- follower type: `LeRobot_<follower_type>`
- leader type: `LeRobot_<leader_teleoperator_type>`

### Supported followers

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

### Supported leader teleoperators

- `so100_leader` (for `so100_follower`)
- `so101_leader` (for `so101_follower`)
- `koch_leader` (for `koch_follower`)
- `omx_leader` (for `omx_follower`)
- `openarm_leader` (for `openarm_follower`)
- `rebot_102_leader` (for `rebot_b601_follower`)
- `reachy2_teleoperator` (for `reachy2`)

### Notes

- Bimanual, test-only, and separately maintained robots are intentionally not
  registered by this plugin.
- Payload models are generated from LeRobot config dataclasses, so fields can
  differ by robot type.

### Payload fields

| Field                        | Type   | Required | Default | Description                                            |
| ---------------------------- | ------ | -------- | ------- | ------------------------------------------------------ |
| `connection_string`          | `str`  | No       | `""`    | Serial device path (for example, `"/dev/ttyACM0"`)     |
| `serial_number`              | `str`  | No       | `""`    | USB serial number for device discovery                 |
| Robot-specific config fields | varies | varies   | varies  | Scalar fields derived from the selected LeRobot config |

Either `serial_number` or `connection_string` must be provided.

## Development

```bash
uv sync
uv run pytest
```

See `docs/creating-a-studio-plugin.md` for the full plugin development guide.
