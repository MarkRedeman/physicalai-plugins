# Design Gaps — `physicalai-lerobot-plugin`

This document captures known mismatches between the PhysicalAI Studio plugin model
and the LeRobot robot abstraction. These are issues that a single-robot-type plugin
(e.g. `physicalai-lekiwi-plugin`) does not face.

---

## Resolved

### Joint Mapping & Key Auto-Detection (formerly gaps 2 & 3)

**Problem:** `joint_map` and observation/action key mappings were robot-specific
(SO100 has 6 joints with `"{motor}.pos"` keys, Aloha has 14, Koch has 7, etc.).
The PhysicalAI protocol uses fixed `np.ndarray` vectors indexed by `joint_order`,
requiring a configurable per-type mapping.

**Resolution:** `LeRobotAdapter._ensure_joint_order()` (and its teleoperator
counterpart) auto-discovers joint order at connect time from the lerobot
observation/action dict. Keys ending in `.pos` are sorted alphabetically and
stripped of the suffix to derive joint names. The position keys are stored as
`_obs_position_keys` / `_act_position_keys` for use by `get_observation()` and
`send_action()`. No per-type configuration is needed.

Additionally, both adapters perform eager joint-order discovery from
`observation_features` / `action_features` at construction time when a live
robot instance is available (guarded via `isinstance(features, dict)`).

### Robot Type vs Catalog Entry Cardinality (formerly gap 8)

**Problem:** It was unclear whether to register one catalog entry per LeRobot
type or a single generic entry with a type discriminator.

**Resolution:** The plugin now registers one `RobotCatalogDefinition` per
LeRobot type (currently 17 entries: 10 followers + 7 leaders). Follower types
use `LeRobot_{follower_name}` naming; leaders use `LeRobot_{teleop_name}`.
The Studio's `RolloutRobotDefinition` selects the appropriate entry at
configuration time.

### Multiprocessing / Pickling Safety

**Problem:** LeRobot's `FeetechMotorsBus` monkey-patches `setPacketTimeout` on
`PortHandler` instances. When multiprocessing spawns a child process, the
`PortHandler` cannot be pickled, causing a crash.

**Resolution:** Both `LeRobotAdapter` and `LeRobotTeleoperatorAdapter` now
store only the config class and keyword arguments at construction time. The
actual lerobot robot/teleoperator is created lazily inside `connect()`. The
`__getstate__` / `__setstate__` protocol strips the live device for
serialization. A pre-built device can be passed via the private `_robot` /
`_teleoperator` parameter for the eager-builder case in the main process.

### `id` Field in Payload Schema

**Problem:** The `id` field from lerobot's `RobotConfig` (`str | None`) was
flattened to `str` with `default=""` for JSON Schema cleanliness, but the UI
failed to render it because it was not marked as required.

**Resolution:** `_make_payload_model` now consistently marks `id` as a
required field in the Pydantic model (`Field(...)`), overriding the resolved
default. This ensures the Studio UI always shows an `id` input.

---

## Remaining Gaps

## 1. URDF per Robot Type

**Problem:** PhysicalAI's `RobotCatalogDefinition` embeds a single
`urdf_relative_path`, `asset_root_resolver`, and `package_map`. Each catalog
entry is tied to one specific robot with one specific URDF. LeRobot supports
many robot types each with a different kinematic tree and mesh files.

**Current workaround:** All 17 catalog entries pass `asset=None`. The Studio
receives no URDF data, so visualisation and kinematic computations are
unavailable.

**Desired:** A mechanism to select/resolve the correct URDF at configuration
time based on the robot type (or some other payload field).

---

## 2. Calibration Is Handled by LeRobot

**Problem:** LeRobot has a built-in calibration system (homing offsets, range
of motion recording, calibration file persistence via `MotorCalibration`).
PhysicalAI plugins (like lekiwi) implement calibration as a separate module.

**Current workaround:** The adapter delegates calibration to the LeRobot
robot's own `connect(calibrate=True)` and `calibrate()` methods. Studio has no
visibility into the calibration state.

**Desired:** Either expose calibration status/information through the PhysicalAI
observation, or provide an adapter-level calibration wrapper.

---

## 3. `is_connected` — Property vs Method

**Problem:** LeRobot's `Robot` declares `is_connected` as an `@property`.
PhysicalAI's `Robot` protocol expects it as a method (`is_connected()`). Both
work at runtime (Python properties are accessed like attributes), but static
type checkers may flag the mismatch.

**Current workaround:** The adapter calls `self._robot.is_connected` which
works whether it's a property or a method.

**Desired:** Agreement on a single convention, or a more relaxed protocol
definition.

---

## 4. `send_action` Return Value

**Problem:** LeRobot's `send_action` returns the action _actually sent_
(potentially clipped by safety limits). PhysicalAI's `send_action` returns
`None`.

**Current workaround:** The adapter discards the return value of
`lerobot_robot.send_action(...)`. Clipped actions are invisible to Studio.

**Desired:** Either include the actually-sent action in the next observation,
or add an optional callback/event for clipped actions.

---

## 5. `connect(calibrate=…)` Signature

**Problem:** LeRobot's `connect()` accepts a `calibrate: bool = True`
parameter. PhysicalAI's `connect()` takes no arguments.

**Current workaround:** The adapter hard-codes `calibrate=True`. Users who want
custom calibration behavior must modify the adapter.

**Desired:** Either add an optional `calibrate` parameter to PhysicalAI's
`connect()`, or expose it through the payload configuration.

---

## 6. Velocity Data

**Problem:** PhysicalAI Studio's `PhysicalAIRobotAdapter._observation_to_state`
optionally extracts velocities from `observation.sensor_data["velocities"]`.
When `include_velocities=True` (as configured by some robot catalog entries)
and the observation lacks this key, `read_state()` raises an error.

**Current workaround:** All LeRobot catalog entries set
`adapter_options=RobotAdapterOptions(include_velocities=False)` — the Studio
simply reads positions and ignores velocity keys in the lerobot observation
dict.

**Desired:** Extract `.vel` keys from the lerobot observation dict and pack
them into `sensor_data["velocities"]` as an array ordered by joint name.
