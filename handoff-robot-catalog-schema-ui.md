# Robot Catalog Schema UI Handoff

## Purpose

Physical AI Studio now renders standard robot configuration forms from each catalog payload model's Pydantic JSON Schema. A plugin can add a conventional robot configuration UI without changing the Studio React application.

The UI obtains the model schema from:

```
GET /api/robots/catalog/{robot_type}/schema
```

The plugin SDK provides typed helpers for Studio-specific JSON Schema metadata:

```python
from physicalai_studio_plugin import robot_field_ui, robot_payload_ui
```

Those helpers produce the namespaced `x-physicalai-ui` fields consumed by the Studio UI.

## What Changed

- Added typed UI-schema options in `application/plugin/src/physicalai_studio_plugin/ui_schema.py`.
- Annotated built-in SO101 and Trossen payload models with connection metadata.
- Annotated ReBot B601 and Arm102 payload models with the same contract.
- Replaced per-robot React form implementations with a schema-driven form in:
  - `application/ui/src/features/robots/robot-form/schema-form.tsx`
  - `application/ui/src/features/robots/robot-form/schema-fields.tsx`
- The renderer currently supports strings, integers, numbers, booleans, and enums.
- Fields that have a JSON Schema default and are not part of the connection group are initialized and submitted, but are not shown. This keeps normal setup minimal while retaining reproducible configuration defaults.
- Bimanual SO101 remains a custom form because it composes already-configured project robots. It is not a payload-only configuration problem.

## Plugin Author Guide

### Basic Payload

Define every user-configurable field with a Pydantic `Field`. Supply a useful description and either make it required or provide a default.

```python
from pydantic import BaseModel, Field


class ExamplePayload(BaseModel):
    host: str = Field(..., description="Robot IP address")
    port: int = Field(default=5000, description="Robot control port")
```

The resulting UI renders `host` because it is required. `port` is initialized to `5000` and submitted, but is hidden because it has a default.

### Enum Fields

Use `Literal` for an enum. Studio renders it as a picker when the field is visible.

```python
from typing import Literal

adapter: Literal["damiao", "socketcan"] = Field(
    default="damiao",
    description="CAN adapter implementation",
)
```

### Boolean Fields

Use `bool`. Studio renders visible boolean fields as switches.

```python
enable_safety_check: bool = Field(..., description="Verify the controller safety state before connecting")
```

### Connection Selector

Serial robots should use one connection group with both a stable serial identifier and a port-path fallback. Studio renders these paired fields as a single editable connection selector.

```python
from pydantic import BaseModel, ConfigDict, Field, model_validator
from physicalai_studio_plugin import robot_field_ui, robot_payload_ui


class ExampleSerialPayload(BaseModel):
    connection_string: str = Field(
        default="",
        description="Serial port path",
        json_schema_extra=robot_field_ui(
            {
                "group": "connection",
                "widget": "device-selector",
                "device_value": "connection_string",
                "manual_entry": True,
            }
        ),
    )
    serial_number: str = Field(
        default="",
        description="USB serial number",
        json_schema_extra=robot_field_ui(
            {
                "group": "connection",
                "widget": "device-selector",
                "device_value": "serial_number",
                "manual_entry": True,
            }
        ),
    )

    model_config = ConfigDict(
        json_schema_extra=robot_payload_ui(
            {
                "groups": {
                    "connection": {
                        "title": "Connection",
                        "device_discovery": True,
                        "stable_key": "serial_number",
                        "fallback_key": "connection_string",
                    }
                }
            }
        )
    )

    @model_validator(mode="after")
    def validate_identifier(self) -> "ExampleSerialPayload":
        if not self.serial_number and not self.connection_string:
            raise ValueError("Either serial_number or connection_string is required")
        return self
```

The selector behavior is:

- Selecting a discovered device writes both fields to the payload.
- A manually entered `/dev/...` path, or a value containing `:`, is stored as `connection_string`.
- Other manually entered values are stored as `serial_number`.
- The serial number is preferred as the selected value because it is stable across port changes.

Set `identify: True` only if the registered probe provides meaningful visual identification. Studio renders the Identify action beside the selector.

### Network Connections

Network robots normally do not need discovery metadata. Put required IP or hostname fields in a regular `connection` group if a shared group title is helpful, or use ordinary required fields without metadata.

```python
connection_string: str = Field(
    ...,
    description="IP address of the robot",
    json_schema_extra=robot_field_ui({"group": "connection"}),
)
```

Do not declare `device_discovery: True` unless the plugin registers a `RobotProbe` whose `discover()` implementation can return candidate devices.

## Metadata Reference

### Field Metadata

Pass this through `Field(json_schema_extra=robot_field_ui(...))`.

| Key | Meaning |
| --- | --- |
| `group` | Name of a model-level group, such as `connection`. |
| `widget` | `device-selector` for a field managed by the shared connection selector. |
| `device_value` | Field on `SerialPortInfo` to copy from a selected device: `serial_number` or `connection_string`. |
| `manual_entry` | Declares that manual input is valid for this selector field. |

### Model Metadata

Pass this through `ConfigDict(json_schema_extra=robot_payload_ui(...))`.

| Key | Meaning |
| --- | --- |
| `groups.<name>.title` | Display label for the group. |
| `groups.<name>.device_discovery` | Enables the shared discovered-device selector. |
| `groups.<name>.identify` | Shows the Identify action for this selector. |
| `groups.<name>.stable_key` | Payload field used as the preferred device identity. |
| `groups.<name>.fallback_key` | Payload field used if the stable identity is unavailable. |

## Registration Requirements

The payload model must be assigned to `RobotCatalogDefinition.robot_payload` and a probe must be assigned when discovery or identification is requested:

```python
RobotCatalogDefinition(
    type="Example_Follower",
    display_name="Example Follower",
    role="follower",
    robot_payload=ExampleSerialPayload,
    probe=ExampleProbe(),
    robot_builder=build_example_robot,
)
```

`RobotProbe.discover()` returns `SerialPortInfo` values. `RobotProbe.identify()` receives the fully validated payload submitted by the UI.

## Current Limits

- Supported generic field types: string, integer, number, boolean, and `Literal` enums.
- Nested objects, arrays, and fields requiring project data are not generically rendered.
- A plugin that needs a workflow beyond payload configuration should add a named Studio workflow rather than attempting to encode executable behavior in schema metadata.
- The backend remains authoritative. Plugin payload validators must validate all semantic constraints, including "one of these fields is required" rules.

## Verification Checklist

When adding or changing a plugin:

1. Confirm `payload_model.model_json_schema()` includes `x-physicalai-ui` metadata where needed.
2. Confirm `GET /api/robots/catalog/{robot_type}/schema` returns the expected schema.
3. Test the form with an installed plugin, including default-only configuration and manual connection entry.
4. Test discovery selection maps every `device-selector` field correctly.
5. Test backend validation for invalid payload combinations.
6. Run the plugin's lint and tests, then UI `npm run lint` and `npm run type-check` when modifying Studio.
