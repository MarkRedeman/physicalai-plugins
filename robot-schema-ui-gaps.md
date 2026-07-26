# Robot Schema UI Gaps

## LeKiwi — Calibration Not Configurable

**Status: Unresolved**

`LeKiwiPayload.calibration` is typed as `dict[str, LeKiwiJointCalibrationPayload] | None`. This is a nested object containing per-joint calibration data (servo IDs, drive modes, homing offsets, tick range limits). The schema-driven form does not support nested objects or arrays, so this field defaults to `None` and is hidden from the UI.

**Impact:** A user configuring a LeKiwi follower through the Studio UI always starts in uncalibrated raw-ticks mode (`LeKiwi.uncalibrated(...)`). Calibrated operation with normalized units requires either a custom Studio workflow or programmatic payload construction.

**Possible approaches (in order of effort):**

| Approach | Effort | Upside | Downside |
|----------|--------|--------|----------|
| **1. File picker widget** — Add a `widget: "file-picker"` to the schema form that lets users select a calibration JSON file from disk. The form reads the file and populates the dict field. | Medium | User can use any calibration file. | Requires new widget support in Studio schema renderer; file must be accessible server-side. |
| **2. Pre-calibrated robot types** — Register multiple catalog entries per calibration profile (e.g. `LeKiwi_Follower_Calibrated`, `LeKiwi_Follower_Uncalibrated`) with different defaults. | Low | No schema changes needed. | Proliferates catalog entries; calibration becomes a deployment-time rather than runtime choice. |
| **3. Calibration as a reference string** — Store a calibration file path or name in the payload instead of inline dict data. The builder loads from disk. | Low | Simple string field is renderable. | Calibration files must be pre-installed; indirection makes the payload non-self-describing. |
| **4. Custom Studio workflow** — Add a named Studio workflow (`calibrate-lekiwi`) that provides a full calibration UI outside the schema form. | High | Full flexibility; calibration could include guided servo sweeps. | Requires React development in Studio; adds workflow infrastructure. |

## LeRobot — Dynamic Registration Resolved Gaps

The following gaps from the original implementation were resolved by rewriting the lerobot plugin:

| Gap | Resolution |
|-----|-----------|
| **List fields not renderable** (`joint_order`, `obs_position_keys`, `act_position_keys`) | `LeRobotAdapter` now auto-detects joint order from the lerobot observation dict keys ending in `.pos`. These fields were removed from the payload entirely. |
| **Connection model ambiguity** | The dynamically generated payload models now use the same `connection_string`/`serial_number` device-selector pattern as the rebot plugin, with `device_discovery=True`. |

## LeRobot — Bimanual Robots Skipped

**Status: Deferred**

Three lerobot bimanual robots are excluded from dynamic registration:
- `bi_so_follower`
- `bi_rebot_b601_follower`
- `bi_openarm_follower`

These require nested `left_arm_config`/`right_arm_config` dataclass fields (typed as other `RobotConfig` subclasses) which the schema form cannot render. A custom Studio workflow (`configure-bimanual-lerobot`) would be needed to provide a UI that composes two sub-arm configs.

## LeRobot — unitree_g1 (Humanoid) Deferred

**Status: Deferred (planned for future inclusion)**

`unitree_g1` is a full-body humanoid with 29 joints, simulation mode, and complex configuration (list-typed `kp`, `kd`, `default_positions`). Dedicated catalog entries for this robot should be added once the plugin and/or Studio supports its specific workflow requirements (IP configuration, simulation toggle, joint limit setup).

## LeRobot — No URDF Assets

**Status: Accepted limitation**

Dynamically registered lerobot robots use `asset=None`. Most lerobot robots don't ship URDF files, and those that do have no standard discovery path. Over time, known robots could be mapped to bundled URDFs in the plugin or discovered from the lerobot package at registration time.

## LeRobot — Network Robots with Serial Connection UI

**Status: Minor UX friction**

`reachy2` (IP-based) and `earthrover_mini_plus` (HTTP-based) are registered with the same serial device-selector connection fields. The device-selector won't find anything useful for these robots, but the fields are harmless — the builder simply doesn't override non-string `port` fields and the robots work with their lerobot defaults.

## LeRobot — lerobot Config Fields Hidden by Default

**Status: By design**

All lerobot config fields with defaults (e.g. `disable_torque_on_disconnect: bool = True`, `use_degrees: bool = True`, `sdk_url: str = "http://localhost:8000"`) are initialized and submitted but not shown in the form. This follows the schema form convention in the handoff doc. A user who needs to change hidden fields requires programmatic payload construction or a custom workflow.

## General — No Identify Support

**Status: Unresolved**

All three plugin probes (ReBot, LeKiwi, LeRobot) have no-op `identify()` methods. The connection group metadata does not set `identify: True`.

**Impact:** The Studio UI never shows the "Identify" action button next to the connection selector.

**Fix:** Implement `RobotProbe.identify()` for each probe that actually sends a visual/audible signal, then set `identify: True` in the respective `robot_payload_ui` payload group metadata.
