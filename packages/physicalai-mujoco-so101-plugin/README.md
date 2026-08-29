# PhysicalAI MuJoCo SO-101 Plugin

MuJoCo SO-101 simulation plugin for [PhysicalAI](https://github.com/openvinotoolkit/physicalai), the Python library and runtime for robot control, transport, and CLI workflows. It registers with [Physical AI Studio](https://github.com/open-edge-platform/physical-ai-studio), the application that discovers catalog plugins and provides robot setup, teleoperation, and workflow experiences. This plugin lets you run a virtual SO-101 robot as a PhysicalAI transport owner, then connect to it from Studio exactly like real hardware. Part of the [physicalai-plugins](https://github.com/MarkRedeman/physicalai-plugins) monorepo.

## Features

- Run a virtual SO-101 as a PhysicalAI transport owner
- MuJoCo interactive viewer
- Camera streams over HTTP (MJPEG) and optional v4l2loopback
- REST control server (scenes, reset, shutdown)
- Built-in task scenes

## Screenshots

_Placeholder images — replace them with real screenshots._

![MuJoCo SO-101 in the PhysicalAI Studio robot catalog](https://raw.githubusercontent.com/MarkRedeman/physicalai-plugins/main/screenshots/studio-catalog.png)

![Connecting to the MuJoCo SO-101 in PhysicalAI Studio](https://raw.githubusercontent.com/MarkRedeman/physicalai-plugins/main/packages/physicalai-mujoco-so101-plugin/screenshots/studio.png)

![MuJoCo viewer with a pick-and-place scene](https://raw.githubusercontent.com/MarkRedeman/physicalai-plugins/main/packages/physicalai-mujoco-so101-plugin/screenshots/viewer.png)

![MJPEG camera stream in a browser](https://raw.githubusercontent.com/MarkRedeman/physicalai-plugins/main/packages/physicalai-mujoco-so101-plugin/screenshots/mjpeg.png)

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
- Camera streams served over HTTP (MJPEG + REST control server on port `8080`)
- Control rate `50 Hz`
- Substeps `10`

Then open PhysicalAI Studio and connect to the robot type `MuJoCo SO-101 Follower` with name `mujoco-so101`.

### CLI teleoperation (self-relay)

With the owner running, you can also relay the simulation back to itself using
the [PhysicalAI CLI](https://github.com/openvinotoolkit/physicalai):

```bash
uv run physicalai run --config packages/physicalai-mujoco-so101-plugin/examples/runtime/teleop.yaml
```

Press `Ctrl+C` to stop.

View the camera streams in a browser or with `curl`:

```bash
curl http://127.0.0.1:8080/health
```

MJPEG stream URLs are `http://127.0.0.1:8080/cameras/<name>/mjpeg` (e.g. open
`http://127.0.0.1:8080/cameras/overview/mjpeg` in a browser, or play it in VLC).

## CLI options

```bash
uv run --no-sync physicalai-mujoco-so101 start --help
```

Common options:

- `--name <robot-name>`: transport name (must match Studio payload)
- `--model <path>`: custom XML/URDF path (bypasses scene resolution)
- `--scene <name>`: scene name (`single_pick_place`, `pick_lift`, `pick_place`, or `yahtzee`, default `single_pick_place`)
- `--no-gui`: disable MuJoCo interactive viewer
- `--no-cameras`: disable camera rendering entirely (HTTP streams and v4l2loopback)
- `--http-host <host>`: host for the camera/control HTTP server (default `127.0.0.1`)
- `--http-port <port>`: port for the camera/control HTTP server (default `8080`)
- `--no-http`: disable the camera/control HTTP server
- `--v4l2`: also publish cameras to v4l2loopback devices (requires `modprobe v4l2loopback`)
- `--rate-hz <float>`: owner loop frequency
- `--substeps <int>`: MuJoCo steps per control cycle
- `--idle-timeout <seconds>`: seconds with zero subscribers before self-exit
  (default `10` without HTTP, disabled when HTTP is enabled so stream viewers keep the sim alive)
- `--allow-remote`: allow non-loopback zenoh connections

## Cameras over HTTP (default)

The plugin renders two camera feeds and serves them over HTTP:

- `wrist` -> `http://127.0.0.1:8080/cameras/wrist/mjpeg`
- `overview` -> `http://127.0.0.1:8080/cameras/overview/mjpeg`

Each camera is also available as a single JPEG snapshot at
`http://127.0.0.1:8080/cameras/<name>/frame.jpg`.

### REST control API

The HTTP server exposes control endpoints for resetting and switching scenes:

```bash
# List available scenes and the current one
curl http://127.0.0.1:8080/scenes

# Switch to another scene
curl -X POST http://127.0.0.1:8080/scenes/pick_place

# Reset/randomize the current scene
curl -X POST http://127.0.0.1:8080/reset

# Stop the simulation owner
curl -X POST http://127.0.0.1:8080/shutdown
```

| Endpoint                    | Method | Description                                   |
| --------------------------- | ------ | --------------------------------------------- |
| `/`                         | GET    | Service info, endpoint index                  |
| `/health`                   | GET    | Sim status: connected, current scene, cameras |
| `/cameras`                  | GET    | Camera list with stream/snapshot URLs         |
| `/cameras/{name}/mjpeg`     | GET    | MJPEG stream (`multipart/x-mixed-replace`)    |
| `/cameras/{name}/frame.jpg` | GET    | Latest frame as a JPEG snapshot               |
| `/scenes`                   | GET    | Current scene and available scene IDs         |
| `/scenes/{scene_id}`        | POST   | Switch to another registered scene            |
| `/reset`                    | POST   | Reset/randomize the current scene             |
| `/shutdown`                 | POST   | Gracefully stop the simulation owner          |

Because the owner process is detached, stopping the CLI with `Ctrl+C` does not
stop the simulation. Use `POST /shutdown`, `--idle-timeout`, or kill the owner
process directly.

## Cameras and v4l2loopback (opt-in)

For workflows that need a webcam-visible device, pass `--v4l2` to publish the
same camera feeds to v4l2loopback devices:

- `wrist` -> `/dev/video<wrist-video-id>` (default `/dev/video60`)
- `overview` -> `/dev/video<overview-video-id>` (default `/dev/video62`)

Example with custom IDs:

```bash
uv run --no-sync physicalai-mujoco-so101 start --v4l2 --wrist-video-id 70 --overview-video-id 71
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

### HTTP server unavailable or port already in use

- The camera/control server binds to `--http-host`/`--http-port` (default `127.0.0.1:8080`).
- If the port is taken the simulation continues without HTTP and logs a warning — pass a different `--http-port`.
- When running multiple simulations, give each a distinct `--http-port`.

### `Camera '<name>' unavailable` or invalid argument on `/dev/video*` (with `--v4l2`)

- Ensure v4l2loopback is loaded with `exclusive_caps=1`
- Ensure the configured video devices exist (default `/dev/video60`, `/dev/video62`)
- Check permissions on device nodes

### Viewer opens but has Wayland warnings (`libdecor`, window position)

These warnings are typically non-fatal on Wayland and can be ignored if simulation continues.

### Camera/control server is not started

- Confirm the sim is running and the port is free: `curl http://127.0.0.1:8080/health`
- `--no-http` or `--http-port 0` disables the server; `--no-cameras` disables all camera rendering

## Using with Studio teleop and inference playback

Typical workflow:

1. Start simulation owner: `physicalai-mujoco-so101 start`
2. Connect from PhysicalAI Studio (`MuJoCo SO-101 Follower`)
3. Teleoperate in Studio and observe state/cameras
4. Run policy inference and play action outputs into the same simulated robot

Because this uses PhysicalAI transport + Studio catalog integration, you can iterate on control and inference loops in simulation before moving to hardware.

## Scenes

The plugin ships with built-in scenes that provide different environments for the robot:

| Scene ID                      | Description                           | Free objects    | Target      |
| ----------------------------- | ------------------------------------- | --------------- | ----------- |
| `single_pick_place` (default) | One block and a target disc           | 1 cube          | target disc |
| `pick_lift`                   | Three colored cubes and a target disc | 3 cubes         | target disc |
| `pick_place`                  | A cube, a cylinder, and a target zone | cube + cylinder | target zone |
| `yahtzee`                     | Six dice and a cup                    | 6 dice          | cup         |

Start with a specific scene:

```bash
uv run --no-sync physicalai-mujoco-so101 start --scene pick_place
```

### Keyboard shortcut: cycle scenes

When the MuJoCo viewer is open, press **`n`** (next scene) to cycle through available scenes. The scene switches at the next control cycle:

- Loads the new scene XML
- Updates the existing viewer with the new environment
- Resets block joints, target bodies, and spawn parameters

`--model` bypasses scene resolution entirely; only the exported scene XML path is loaded, and scene cycling is unavailable.

## Current status and future improvements

Planned/desired improvements:

- More configurable camera presets (pose/FOV/fps via CLI or payload)
- Scene randomization presets (object layouts, target marker variants, textures)
- RTSP streaming sink (the frame-buffer plumbing is sink-agnostic; an RTSP
  server such as `aiortsp` or MediaMTX can consume the same buffers later)
- Optional richer sensor streams (depth/segmentation style outputs)
- Additional sample tasks and policy playback recipes in this package
