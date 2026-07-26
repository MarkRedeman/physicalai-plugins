# Building a PhysicalAI Studio Plugin

This guide documents the patterns, protocols, and conventions used across existing PhysicalAI Studio robot plugins. Follow it when creating a new plugin.

---

## Overview

A PhysicalAI Studio plugin makes a robot type available in the Studio UI by registering one or more `RobotCatalogDefinition` entries. Each entry describes a robot model, its configuration payload, its builder function, and its hardware probe.

### Existing plugins (reference implementations)

| Plugin | Robot(s) | Type count | Entry point |
|---|---|---|---|
| `physicalai-rebot-b601-plugin` | ReBot B601 (Damiao + RobStride motors), Arm102 leader | 3 | `rebot-b601` |
| `physicalai-lekiwi-plugin` | LeKiwi mobile manipulator | 1 | `lekiwi` |
| `physicalai-lerobot-plugin` | 10+ LeRobot types (SO100, Koch, Reachy2…) | 17 | `lerobot` |
| `physicalai-bimanual-so101-plugin` | Twin SO101 arms (left + right) | 1 | `bimanual-so101` |
| `physicalai-websocket-robot-plugin` | Remote robot via WebSocket | 1 | `websocket-robot` |
| `physicalai-zmq-robot-plugin` | Remote robot via ZMQ | 1 | `zmq-robot` |

---

## Package Structure

```
packages/<plugin-name>/
    pyproject.toml
    README.md
    src/<import_name>/
        __init__.py
        studio_catalog.py       # REQUIRED — registration entry point
        _urdf.py                # OPTIONAL — URDF path helpers
        <robot_driver>.py       # REQUIRED — Robot protocol implementation
    tests/
        test_studio_catalog.py
        test_<driver>.py
    urdf/                       # OPTIONAL — bundled visual models
```

---

## Step 1: `pyproject.toml`

Use `hatchling` + `hatch-vcs` for versioning. Declare the entry point under `[project.entry-points."physicalai.studio.catalog_plugins"]`.

```toml
[build-system]
requires = ["hatchling", "hatch-vcs"]
build-backend = "hatchling.build"

[project]
name = "physicalai-<robot>-plugin"
dynamic = ["version"]
description = "<Robot name> plugin for PhysicalAI"
readme = "README.md"
requires-python = ">=3.12"
license = "Apache-2.0"
dependencies = [
    "physicalai>=0.1.1",
    "physicalai-studio-plugin",
    "numpy>=1.24",
    "loguru>=0.7",
    "pydantic>=2.0",
    # robot-specific SDK here
]

[project.entry-points."physicalai.studio.catalog_plugins"]
<unique-name> = "<import_name>.studio_catalog:register_physicalai_studio_plugin"

[tool.hatch.version]
source = "vcs"
raw-options = { search_parent_directories = true }

[tool.hatch.build.targets.wheel]
packages = ["src/<import_name>"]

# Include URDFs if bundled
[tool.hatch.build.targets.wheel.force-include]
"urdf" = "urdf"

[tool.hatch.build.targets.sdist]
include = [
    "src/<import_name>/**",
    "urdf/**",
    "README.md",
    "pyproject.toml",
    "LICENSE",
]
```

### Entry point naming

The entry-point name must be unique across all installed plugins. Convention: use the robot family name (e.g. `rebot-b601`, `lekiwi`, `lerobot`, `bimanual-so101`, `websocket-robot`, `zmq-robot`).

---

## Step 2: Implement the Robot Protocol

The core interface is defined in `physicalai.robot.interface`:

```python
@runtime_checkable
class Robot(Protocol):
    joint_names: list[str]

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def get_observation(self) -> RobotObservation: ...
    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None: ...
    def is_connected(self) -> bool: ...

@runtime_checkable
class RobotObservation(Protocol):
    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None
    images: dict[str, Frame] | None

    @property
    def state(self) -> np.ndarray: ...
```

No inheritance required — structural duck typing. Implement the attributes and methods and `isinstance(my_robot, Robot)` will return `True`.

### Observation pattern (dataclass)

Create a concrete observation dataclass:

```python
@dataclass
class MyRobotObservation:
    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        return self.joint_positions
```

### Key implementation notes

- **`joint_names`** — Must match the length and order of `joint_positions` and the `action` array. Return a fixed list or auto-discover from hardware.
- **`connect()`** — Must be idempotent. Some robots support `calibrate=True` parameter; the PhysicalAI protocol has no such parameter, so hard-code what makes sense.
- **`disconnect()`** — Must leave motors in a safe stationary state.
- **`send_action()`** — Receives `np.ndarray` of shape `(N,)`. The `goal_time` kwarg is advisory.
- **`is_connected()`** — PhysicalAI protocol expects a method call (`is_connected()`), but some SDKs expose it as a `@property`. The adapter can handle either at runtime.
- **`device_ids`** — Return a tuple of canonical device identifiers derived purely from constructor parameters (no I/O). Used by the transport layer for host-local exclusivity locking.

### Reference implementations

| Plugin | Driver class | File |
|---|---|---|
| rebot-b601 | `ReBotB601DM`, `ReBotB601RS`, `ReBotArm102Leader` | `dm.py`, `rs.py`, `leader.py` |
| lekiwi | `LeKiwi` | `lekiwi.py` |
| lerobot | `LeRobotAdapter`, `LeRobotTeleoperatorAdapter` | `lerobot_adapter.py` |
| websocket | `WebSocketRobot` | `websocket_robot.py` |
| zmq | `ZMQRobot` | `zmq_robot.py` |
| bimanual-so101 | `BimanualSO101` | `bimanual.py` |

---

## Step 3: Build the `studio_catalog.py`

This is the core registration file. It must expose a single public function:

```python
def register_physicalai_studio_plugin(registry: RobotCatalogRegistry) -> None: ...
```

### 3a. Payload model

Define a Pydantic `BaseModel` for user-configurable fields (serial port, baud rate, calibration options, etc.):

```python
from pydantic import BaseModel, Field
from physicalai_studio_plugin import robot_field_ui, robot_payload_ui, RobotFieldUiOptions, RobotPayloadUiOptions

class MyRobotPayload(BaseModel):
    model_config = ConfigDict(
        json_schema_extra=robot_payload_ui(...)
    )

    serial_number: str = Field(default="", description="Robot serial number")
    connection_string: str = Field(default="", description="Port or connection endpoint")
    baudrate: int = Field(default=1_000_000, description="Serial baud rate")
    # ... robot-specific fields
```

**UI metadata** — Use `robot_field_ui()` and `robot_payload_ui()` to annotate fields for the Studio form renderer:

```python
class MyRobotPayload(BaseModel):
    port: str = Field(
        default="",
        description="Serial port",
        json_schema_extra=robot_field_ui({
            "group": "connection",
            "widget": "device-selector",
            "device_value": "serial_number",
            "manual_entry": True,
        }),
    )
```

See the lekiwi or rebot-b601 payload models for complete examples of UI grouping and device-selector widgets.

**`id` field** — If your config model has an `id` field that may be `str | None`, mark it required in the Pydantic model so the Studio UI renders it even when the underlying hardware allows a null default (`Field(...)` instead of `Field(default="")`). See `_make_payload_model` in the lerobot plugin for the pattern.

### 3b. Probe class

Implement `RobotProbe[PayloadT]` for hardware discovery:

```python
from physicalai_studio_plugin import RobotProbe, PortScanner, SerialPortInfo

class MyRobotProbe(RobotProbe[MyRobotPayload]):
    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        await manager.find_robots()
        return manager.robots

    async def identify(self, payload: MyRobotPayload, manager: PortScanner | None, joint: str | None = None) -> None:
        # blink a light, move a joint, etc.
        ...

    async def is_online(self, payload: MyRobotPayload, manager: PortScanner | None = None) -> bool:
        # check if the serial port or network endpoint is reachable
        ...
```

### 3c. Builder function

The builder is an `async` callable that receives a `PayloadContainer[PayloadT]` and a `CatalogRobotFactory`, and returns a `PhysicalAIRobot`:

```python
from collections.abc import Awaitable, Callable
from physicalai_studio_plugin import CatalogRobotFactory, PayloadContainer
from physicalai.robot.interface import Robot as PhysicalAIRobot

BuildRobot = Callable[[PayloadContainer[MyRobotPayload], CatalogRobotFactory], Awaitable[PhysicalAIRobot]]

async def _build_my_robot(
    robot: PayloadContainer[MyRobotPayload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, BaseModel) and type(raw) is not MyRobotPayload:
        raw = raw.model_dump()
    validated = raw if isinstance(raw, MyRobotPayload) else MyRobotPayload.model_validate(raw)

    port = await factory.find_port(
        SerialPortInfo(connection_string=validated.connection_string, serial_number=validated.serial_number),
    )
    if port is None:
        msg = f"Robot not found: {validated.serial_number}"
        raise RuntimeError(msg)

    # Create and return your Robot-protocol implementation
    driver = MyRobotDriver(port=port, baudrate=validated.baudrate)
    return driver
```

**Builder patterns to follow:**

1. **Cross-identity validation** — The payload may arrive as a different Pydantic model class or a plain dict. Always check `isinstance(raw, BaseModel) and type(raw) is not payload_cls` and call `.model_dump()` before `model_validate()`.
2. **Port resolution** — Use `factory.find_port()` with `SerialPortInfo`. Raise `RuntimeError` if the port isn't found.
3. **Config passthrough** — Map validated payload fields to your driver constructor.
4. **Multiprocessing safety** — See step 4 below.

### 3d. RobotAdapterOptions

Each definition includes `adapter_options` that controls how the Studio wraps your robot:

```python
@dataclass(frozen=True)
class RobotAdapterOptions:
    include_velocities: bool = False    # read .vel keys from sensor_data
    goal_time_scale: float = 1.0        # multiplier for goal_time
    external_effort_gain: float | None = 0.1  # force feedback gain (None = disabled)
```

Set `include_velocities=False` if your observation does not provide `sensor_data["velocities"]` as a single array.

### 3e. Definitions and registration

```python
from physicalai_studio_plugin import RobotCatalogDefinition

def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="MyRobot_Follower",
            display_name="My Robot Follower",
            role="follower",
            robot_builder=_build_my_robot,
            robot_payload=MyRobotPayload,
            asset=None,  # or RobotAsset(...) if URDF is available
            adapter_options=RobotAdapterOptions(include_velocities=False),
            probe=MyRobotProbe(),
        ),
    ]

def register_physicalai_studio_plugin(registry: RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register_robot(definition)
```

**Type naming** — Each robot type string must be unique across all installed plugins. Convention: `{RobotFamily}_{OptionalVariant}_{Role}` (e.g. `ReBot_B601_DM_Follower`, `LeRobot_so100`, `LeRobot_so100_leader`).

**Role** — `"follower"` for full control, `"leader"` for read-only teleoperation input.

---

## Step 4: Multiprocessing Safety

The Studio spawns worker processes. Any object passed across the process boundary must be picklable.

### Problem

SDKs often create objects that are not picklable (e.g. `dynamixel-sdk` `PortHandler` patched by `FeetechMotorsBus`, serial port handles, network sockets).

### Solution pattern

Store constructor *parameters* instead of live SDK objects. Create the live device lazily in `connect()`. Implement `__getstate__` / `__setstate__` to strip non-picklable fields:

```python
class MyAdapter:
    def __init__(self, config_cls: type, config_kwargs: dict, *, _device=None):
        self._config_cls = config_cls
        self._config_kwargs = config_kwargs
        self._device = _device  # may be None in child process

    def __getstate__(self):
        return {"_config_cls": self._config_cls, "_config_kwargs": self._config_kwargs}

    def __setstate__(self, state):
        self._config_cls = state["_config_cls"]
        self._config_kwargs = state["_config_kwargs"]
        self._device = None

    def connect(self):
        if self._device is None:
            config = self._config_cls(**self._config_kwargs)
            self._device = create_device(config)
        self._device.connect()
```

The builder in `studio_catalog.py` can eagerly create the device (before pickling) and pass it as a private parameter for the main-process case, while the lazy fallback handles the child process.

**Reference:** `LeRobotAdapter` and `LeRobotTeleoperatorAdapter` in the lerobot plugin.

---

## Step 5: URDF Assets (optional)

If your robot has a URDF model, bundle it in a `urdf/` directory and create an asset resolver:

```python
# _urdf.py
from pathlib import Path

def _get_urdf_root() -> Path:
    return Path(__file__).resolve().parent.parent.parent / "urdf"

def get_my_robot_urdf_path() -> Path:
    return _get_urdf_root() / "my-robot" / "urdf" / "model.urdf"
```

In the catalog definition, set `asset=RobotAsset(...)`:

```python
from physicalai_studio_plugin import RobotAsset

RobotAsset(
    urdf_relative_path=Path("my-robot/urdf/model.urdf"),
    packages={"my-robot": Path("my-robot")},
    joint_map={
        "joint_1": ["shoulder_pan.pos"],
        "joint_2": ["shoulder_lift.pos"],
    },
    root_resolver=_get_urdf_root,
)
```

**Reference:** `_urdf.py` in rebot-b601, lekiwi, and lerobot plugins.

---

## Step 6: `__init__.py` Public API

Use lazy `__getattr__` to avoid importing heavy SDKs at package import time:

```python
from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .my_robot import MyRobot as MyRobot
    from .my_robot import MyRobotObservation as MyRobotObservation

__all__ = ["MyRobot", "MyRobotObservation"]

def __getattr__(name: str) -> object:
    if name == "MyRobot":
        from .my_robot import MyRobot
        return MyRobot
    if name == "MyRobotObservation":
        from .my_robot import MyRobotObservation
        return MyRobotObservation
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
```

---

## Step 7: Testing

### Studio catalog tests

Test that definitions are correct, payload models validate, and builders produce valid drivers:

- Use stub/mock factory and robot objects (see `test_studio_catalog.py` in any existing plugin).
- Test builder with both `BaseModel` and `dict` payloads (cross-identity validation).
- Test builder raises `RuntimeError` when port not found.
- Verify `adapter_options` values.
- Verify `json_schema_extra` metadata on payload models.

### Driver tests

- Unit test `connect`/`disconnect` idempotency.
- Unit test `get_observation` returns correct structure.
- Unit test `send_action` rejects wrong-shaped arrays.
- Unit test `send_action` raises in leader role (if applicable).
- Mock the hardware SDK to avoid needing physical hardware.

### Test configuration

Add to repo root `pyproject.toml`:

```toml
[tool.pytest.ini_options]
testpaths = "packages/"
addopts = "--import-mode=importlib"
```

---

## Common Pitfalls

| Issue | Cause | Fix |
|---|---|---|
| `id` not rendering in UI | `str \| None` field with default produces `anyOf: [string, null]` in JSON Schema | Mark as required with `Field(...)` even if default is empty string |
| Pickling crash in multiprocessing | SDK objects (PortHandler, serial) not picklable | Store config params, create device lazily in `connect()`, implement `__getstate__`/`__setstate__` |
| `is_connected` type error | LeRobot exposes it as `@property`, PAI expects method | Access as attribute (`self._robot.is_connected`) — works for both |
| Velocity error | Studio expects `sensor_data["velocities"]` array | Set `include_velocities=False` in adapter options, or extract `.vel` keys and pack into array |
| Cross-identity payload validation | Studio may pass dict or different BaseModel subclass | Always check `type(raw) is not payload_cls` and call `model_dump()` before `model_validate()` |
| Entry point not discovered | Wrong group name | Must be `physicalai.studio.catalog_plugins` |
| URDF not packaged | `hatchling` excludes files outside `src/` by default | Add `[tool.hatch.build.targets.wheel.force-include] "urdf" = "urdf"` |

---

## API Reference

Public types from `physicalai-studio-plugin`:

```python
from physicalai_studio_plugin import (
    RobotCatalogDefinition,     # @dataclass — one catalog entry
    RobotAdapterOptions,        # @dataclass — velocity/timing/effort config
    RobotCatalogRegistry,       # Protocol — register_robot(definition)
    CatalogRobotFactory,        # Protocol — find_port(port_info)
    PayloadContainer,           # Protocol — .payload: PayloadT
    RobotProbe,                 # Protocol — discover/identify/is_online
    PortScanner,                # Protocol — find_robots/robots
    SerialPortInfo,             # Pydantic model — connection_string, serial_number
    RobotAsset,                 # @dataclass — URDF path, packages, joint map
    robot_field_ui,             # helper — json_schema_extra for field
    robot_payload_ui,           # helper — json_schema_extra for model
    RobotFieldUiOptions,        # TypedDict — group, widget, device_value
    RobotPayloadUiOptions,      # TypedDict — groups
)
```

Public types from `physicalai`:

```python
from physicalai.robot import (
    Robot,              # Protocol — connect/disconnect/get_observation/send_action/is_connected
    RobotObservation,   # Protocol — joint_positions, timestamp, sensor_data, images
)
```
