# Design Gaps — `physicalai-lerobot-plugin`

This document captures known mismatches between the PhysicalAI Studio plugin model
and the LeRobot robot abstraction. These are issues that a single-robot-type plugin
(e.g. `physicalai-lekiwi-plugin`) does not face.

---

## 1. URDF per Robot Type

**Problem:** PhysicalAI's `_CatalogDefinition` embeds a single `urdf_relative_path`,
`asset_root_resolver`, and `package_map`. Each catalog entry is tied to one specific
robot with one specific URDF. LeRobot, however, supports many robot types (SO100,
SO101, Aloha, Koch, Reachy2, …) each with a different kinematic tree and mesh files.

**Current workaround:** The catalog entries `LeRobot_Follower` / `LeRobot_Leader` use
a single placeholder URDF. Users must swap the URDF for their specific robot.

**Desired:** A mechanism to select/resolve the correct URDF at configuration time
based on `robot_type` (or some other payload field).

---

## 2. Joint Mapping Is Robot-Specific

**Problem:** The `joint_map` in `_CatalogDefinition` maps observation keys (e.g.
`"shoulder_pan.pos"`) to URDF joint names (e.g. `["joint1"]`). This mapping differs
per robot type — SO100 has 6 joints, Aloha has 14, Koch has 7, etc. The current
schema fixes this at catalog-registration time.

**Current workaround:** We hard-code an SO100 joint map in the definition. Using a
different LeRobot type would produce an incorrect mapping.

**Desired:** A way to provide per-robot-type joint maps, possibly via the payload
or a registry.

---

## 3. Observation/Action Dict Keys Are Robot-Specific

**Problem:** LeRobot robots use `dict[str, Any]` for observations and actions. The
key names vary by robot:

- SO100/SO101: `"{motor}.pos"` keys
- Koch: different naming convention
- Aloha: different keys for the two arms

The PhysicalAI protocol uses fixed `np.ndarray` vectors indexed by `joint_order`.
The adapter needs configurable key-mapping between the two representations.

**Current workaround:** The `LeRobotAdapter` accepts `obs_position_keys` and
`act_position_keys` parameters. These must be explicitly provided per robot type
in the payload.

**Desired:** An automatic or convention-based key resolver that derives the correct
keys from `Robot.observation_features` / `Robot.action_features`.

---

## 4. Calibration Is Handled by LeRobot

**Problem:** LeRobot has a built-in calibration system (homing offsets, range of
motion recording, calibration file persistence via `MotorCalibration`). PhysicalAI
plugins (like lekiwi) implement calibration as a separate module.

**Current workaround:** The adapter delegates calibration to the LeRobot robot's
own `connect(calibrate=True)` and `calibrate()` methods. Studio has no visibility
into the calibration state.

**Desired:** Either expose calibration status/information through the PhysicalAI
observation, or provide an adapter-level calibration wrapper.

---

## 5. `is_connected` — Property vs Method

**Problem:** LeRobot's `Robot` declares `is_connected` as an `@property`. PhysicalAI's
`Robot` protocol expects it as a method (`is_connected()`). Both work at runtime
(Python properties are accessed like attributes), but static type checkers may
flag the mismatch.

**Current workaround:** The adapter calls `self._robot.is_connected` which works
whether it's a property or a method.

**Desired:** Agreement on a single convention, or a more relaxed protocol
definition.

---

## 6. `send_action` Return Value

**Problem:** LeRobot's `send_action` returns the action _actually sent_ (potentially
clipped by safety limits). PhysicalAI's `send_action` returns `None`.

**Current workaround:** The adapter discards the return value of
`lerobot_robot.send_action(...)`. Clipped actions are invisible to Studio.

**Desired:** Either include the actually-sent action in the next observation, or
add an optional callback/event for clipped actions.

---

## 7. `connect(calibrate=…)` Signature

**Problem:** LeRobot's `connect()` accepts a `calibrate: bool = True` parameter.
PhysicalAI's `connect()` takes no arguments.

**Current workaround:** The adapter hard-codes `calibrate=True`. Users who want
custom calibration behavior must modify the adapter.

**Desired:** Either add an optional `calibrate` parameter to PhysicalAI's `connect()`,
or expose it through the payload configuration.

---

## 8. Robot Type vs Catalog Entry Cardinality

**Problem:** Currently, PhysicalAI registers one catalog entry per robot _instance_
(e.g., a specific `LeRobot_Follower`). With LeRobot, one adapter serves many robot
_types_. It is unclear whether we should:

1. Register one catalog entry per LeRobot type (proliferates entries).
2. Register a single generic entry with a `robot_type` discriminator in the payload
   (current approach — requires per-type runtime branching).

**Desired:** A design pattern for polymorphic catalog entries that select the
correct URDF, joint map, and key mappings based on a type discriminator.
