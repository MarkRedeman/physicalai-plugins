# PhysicalAI MuJoCo SO-101 Plugin

MuJoCo SO-101 simulation plugin for PhysicalAI Studio.

This plugin lets you run a virtual SO-101 robot as a PhysicalAI transport owner, then connect to it from Studio exactly like real hardware.

## What this is for

- Teleoperating a virtual SO-101 from PhysicalAI Studio.
- Testing robot setup, task logic, and control flows without physical hardware.
- Playing back policy/inference outputs in a simulated scene while monitoring robot + camera streams.
- Developing workflows where Studio, transport, and robot APIs stay identical between sim and real deployments.

## Quick start

From the repo root:

```bash
uv run --no-sync physicalai-mujoco-so101 start
```

By default this starts:

- A MuJoCo SO-101 owner named `mujoco-so101`
- Viewer window enabled
- Virtual cameras enabled (`/dev/video60` and `/dev/video61`)
- Control rate `50 Hz`
- Substeps `10`

Then open PhysicalAI Studio and connect to the robot type `MuJoCo SO-101 Follower` with name `mujoco-so101`.

## CLI options

```bash
uv run --no-sync physicalai-mujoco-so101 start --help
```

Common options:

- `--name <robot-name>`: transport name (must match Studio payload)
- `--model <path>`: custom XML/URDF path
- `--no-gui`: disable MuJoCo interactive viewer
- `--no-cameras`: disable v4l2loopback camera output
- `--wrist-video-id <int>`: video ID for wrist stream (default `60`)
- `--overview-video-id <int>`: video ID for overview stream (default `61`)
- `--rate-hz <float>`: owner loop frequency
- `--substeps <int>`: MuJoCo steps per control cycle
- `--allow-remote`: allow non-loopback zenoh connections

## Cameras and v4l2loopback

The plugin publishes two virtual camera feeds:

- `wrist` -> `/dev/video<wrist-video-id>` (default `/dev/video60`)
- `overview` -> `/dev/video<overview-video-id>` (default `/dev/video61`)

Example with custom IDs:

```bash
uv run --no-sync physicalai-mujoco-so101 start --wrist-video-id 70 --overview-video-id 71
```

### One-time setup (Linux)

Install and load v4l2loopback:

```bash
sudo modprobe v4l2loopback exclusive_caps=1 video_nr=60,61
```

If you use custom camera IDs, use matching `video_nr` values. Example:

```bash
sudo modprobe v4l2loopback exclusive_caps=1 video_nr=70,71
```

If the module is already loaded with different params, unload and reload:

```bash
sudo rmmod v4l2loopback
sudo modprobe v4l2loopback exclusive_caps=1 video_nr=60,61
```

### Verify devices

```bash
ls /dev/video60 /dev/video61
```

For custom IDs, verify those device nodes instead.

Optional sanity checks:

```bash
v4l2-ctl --all -d /dev/video60
v4l2-ctl --all -d /dev/video61
```

## Troubleshooting

### `Camera '<name>' unavailable` or invalid argument on `/dev/video*`

- Ensure v4l2loopback is loaded with `exclusive_caps=1`
- Ensure the configured video devices exist (default `/dev/video60`, `/dev/video61`)
- Check permissions on device nodes

### Viewer opens but has Wayland warnings (`libdecor`, window position)

These warnings are typically non-fatal on Wayland and can be ignored if simulation continues.

## Using with Studio teleop and inference playback

Typical workflow:

1. Start simulation owner: `physicalai-mujoco-so101 start`
2. Connect from PhysicalAI Studio (`MuJoCo SO-101 Follower`)
3. Teleoperate in Studio and observe state/cameras
4. Run policy inference and play action outputs into the same simulated robot

Because this uses PhysicalAI transport + Studio catalog integration, you can iterate on control and inference loops in simulation before moving to hardware.

## Current status and future improvements

Planned/desired improvements:

- More configurable camera presets (pose/FOV/device/fps via CLI or payload)
- Scene randomization presets (object layouts, target marker variants, textures)
- Better turnkey persistence for v4l2loopback module options across reboots
- Optional richer sensor streams (depth/segmentation style outputs)
- Additional sample tasks and policy playback recipes in this package
