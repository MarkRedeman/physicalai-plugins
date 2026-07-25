# Robot Schema UI Gaps

## LeKiwi — Calibration Not Configurable

**Problem:** `LeKiwiPayload.calibration` is typed as `dict[str, LeKiwiJointCalibrationPayload] | None`. This is a nested object containing per-joint calibration data (servo IDs, drive modes, homing offsets, tick range limits). The schema-driven form does not support nested objects or arrays, so this field defaults to `None` and is hidden from the UI.

**Impact:** A user configuring a LeKiwi follower through the Studio UI always starts in uncalibrated raw-ticks mode (`LeKiwi.uncalibrated(...)`). Calibrated operation with normalized units requires either a custom Studio workflow or programmatic payload construction.

**Possible approaches (in order of effort):**

| Approach | Effort | Upside | Downside |
|----------|--------|--------|----------|
| **1. File picker widget** — Add a `widget: "file-picker"` to the schema form that lets users select a calibration JSON file from disk. The form reads the file and populates the dict field. | Medium | User can use any calibration file. | Requires new widget support in Studio schema renderer; file must be accessible server-side. |
| **2. Pre-calibrated robot types** — Register multiple catalog entries per calibration profile (e.g. `LeKiwi_Follower_Calibrated`, `LeKiwi_Follower_Uncalibrated`) with different defaults. | Low | No schema changes needed. | Proliferates catalog entries; calibration becomes a deployment-time rather than runtime choice. |
| **3. Calibration as a reference string** — Store a calibration file path or name in the payload instead of inline dict data. The builder loads from disk. | Low | Simple string field is renderable. | Calibration files must be pre-installed; indirection makes the payload non-self-describing. |
| **4. Custom Studio workflow** — Add a named Studio workflow (`calibrate-lekiwi`) that provides a full calibration UI outside the schema form. | High | Full flexibility; calibration could include guided servo sweeps. | Requires React development in Studio; adds workflow infrastructure. |

## LeRobot — List Fields Not Renderable

**Problem:** `LeRobotPayload` has required list fields (`joint_order: list[str]`) and optional list fields (`obs_position_keys`, `act_position_keys`) that the schema-driven form cannot render.

**Impact:** The `joint_order` field is required but cannot be set through the generic form UI. A user submitting the form will fail validation because `joint_order` is missing.

**Possible approaches:**

| Approach | Effort | Upside | Downside |
|----------|--------|--------|----------|
| **1. Custom Studio workflow** — Add a workflow (`configure-lerobot`) that provides a full LeRobot configuration UI, including joint order selection and key mapping. | High | Full flexibility. | Requires React development; the workflow must exist before LeRobot can be configured. |
| **2. Per-robot-type payloads** — Split into `SO100Payload` and `SO101Payload` with hardcoded `joint_order` and key defaults. The schema form only shows `robot_type`, `port`, and `serial_number`. | Medium | Clean separation; `joint_order` is never a user concern. | Requires separate builder functions for each type; payload model change. |
| **3. String serialization** — Replace `joint_order` with a comma-separated `str` field, then parse in the builder. | Low | Simple string fields render fine. | Fragile format; no schema-level validation of joint names. |

## LeRobot — Connection Model Ambiguity

**Problem:** LeRobot adapters use a `port` field (serial path) rather than the dual `connection_string`/`serial_number` pattern used by serial robots. The current probe checks both `serial_number` and `port`, but the builder does not use `serial_number` for device discovery.

**Impact:** The connection group is marked without `device_discovery`, so users must manually enter a port. The `serial_number` field is mostly decorative.

## General — No Identify Support

**Problem:** All three plugin probes (ReBot, LeKiwi, LeRobot) have no-op `identify()` methods. The connection group metadata does not set `identify: True`.

**Impact:** The Studio UI never shows the "Identify" action button next to the connection selector.

**Fix:** Implement `RobotProbe.identify()` for each probe that actually sends a visual/audible signal, then set `identify: True` in the respective `robot_payload_ui` payload group metadata.
