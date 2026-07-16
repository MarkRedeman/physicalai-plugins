# Creating a Physical AI Studio Plugin

This guide explains how to create a new robot plugin for **Physical AI Studio** by
walking through the patterns used by existing plugins:

- [`physicalai-rebot-b601-plugin`](../packages/physicalai-rebot-b601-plugin/) — three
  robot drivers (Damiao, RobStride, leader arm) with URDF models
- [`physicalai-lekiwi-plugin`](../packages/physicalai-lekiwi-plugin/) — mobile
  manipulator (6-DOF arm + 3-wheel holonomic base) with calibration support

Both are third-party plugins that Studio discovers at runtime via Python entry
points — no changes to Studio's source code are required.

---

## 1. Overview

At startup, Studio's `RobotCatalogRegistry` loads external plugins by scanning
the `physicalai.studio.catalog_plugins` entry point group:

```python
from importlib.metadata import entry_points

for ep in entry_points(group="physicalai.studio.catalog_plugins"):
    register = ep.load()
    register(registry)
```

Your plugin must:

1. **Export a callable** named `register_physicalai_studio_plugin(registry)` that
   registers one or more `_CatalogDefinition` entries.
2. **Declare an entry point** in `pyproject.toml` pointing to that callable.
3. **Implement at least one robot driver** that satisfies the
   `physicalai.robot.Robot` protocol (duck-typed, no base class required).

---

## 2. Package Layout

A typical plugin package follows the `src/` layout. Here is the full-featured
layout (rebot-b601 with URDF):

```
physicalai-rebot-b601-plugin/
├── pyproject.toml
├── README.md
├── src/
│   └── physicalai_rebot_b601_plugin/
│       ├── __init__.py          # Public API, lazy imports via __getattr__
│       ├── _urdf.py             # URDF path resolution
│       ├── constants.py         # Joint orders, motor IDs, limits, gains
│       ├── dm.py                # Robot driver: ReBotB601DM
│       ├── rs.py                # Robot driver: ReBotB601RS
│       ├── leader.py            # Robot driver: ReBotArm102Leader
│       └── studio_catalog.py    # Studio plugin registration
├── tests/
│   ├── test_dm.py
│   ├── test_rs.py
│   ├── test_leader.py
│   └── test_studio_catalog.py
├── urdf/
│   ├── rebot-b601-dm/
│   │   ├── meshes/              # STL files
│   │   └── urdf/                # URDF files
│   ├── rebot-b601-rs/
│   └── stararm102/
└── examples/
    ├── read_joints.py
    ├── move_joints.py
    └── teleoperation.py
```

And a minimal layout without URDF (lekiwi):

```
physicalai-lekiwi-plugin/
├── pyproject.toml
├── README.md
├── src/
│   └── physicalai_lekiwi_plugin/
│       ├── __init__.py
│       ├── _urdf.py
│       ├── calibration.py
│       ├── constants.py
│       ├── lekiwi.py            # Single robot driver
│       └── studio_catalog.py
├── tests/
│   ├── test_lekiwi.py
│   └── test_studio_catalog.py
└── urdf/
    └── lekiwi/
        ├── meshes/
        └── urdf/
            └── LeKiwi.urdf
```

### 2.1 `__init__.py` — Public API with Lazy Imports

Use `TYPE_CHECKING` guards for type annotations and `__getattr__` for lazy
loading. This avoids importing heavy hardware SDKs at package import time.

```python
"""My robot plugin for PhysicalAI."""

from __future__ import annotations

from typing import TYPE_CHECKING

from my_plugin._urdf import get_urdf_path as get_urdf_path

if TYPE_CHECKING:
    from my_plugin.my_robot import MyRobot as MyRobot
    from my_plugin.my_robot import MyRobotObservation as MyRobotObservation

__all__ = [
    "MyRobot",
    "MyRobotObservation",
    "get_urdf_path",
]


def __getattr__(name: str) -> object:
    if name == "MyRobot":
        from my_plugin.my_robot import MyRobot
        return MyRobot
    if name == "MyRobotObservation":
        from my_plugin.my_robot import MyRobotObservation
        return MyRobotObservation
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
```

---

## 3. Implementing the Robot Driver

Your driver class must duck-type the `physicalai.robot.Robot` protocol. No base
class or metaclass is needed — just implement the right methods.

### 3.1 Required Protocol

```python
from physicalai.robot.interface import RobotObservation, Robot


class MyRobot:
    """Duck-typed implementation of the Robot protocol."""

    JOINT_ORDER: ClassVar[list[str]] = [...]   # ordered joint names
    NUM_JOINTS: ClassVar[int] = ...

    def __init__(self, ...) -> None:
        ...

    def connect(self) -> None:
        """Open the connection. Raise ConnectionError on failure."""

    def disconnect(self) -> None:
        """Close the connection. Safe to call multiple times."""

    def is_connected(self) -> bool:
        ...

    def get_observation(self) -> RobotObservation:
        """Read current joint positions and return an observation."""

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Send a control command. Raise RuntimeError for leader/read-only."""

    @property
    def joint_names(self) -> list[str]:
        return self.JOINT_ORDER
```

### 3.2 Observation Dataclass

Return a dataclass from `get_observation()` with at minimum `joint_positions`,
`timestamp`, and optionally `sensor_data` and `images`. Include a `state`
property that returns the primary state vector.

```python
@dataclass
class MyRobotObservation:
    joint_positions: np.ndarray      # shape (N,)
    timestamp: float                  # time.monotonic()
    sensor_data: dict | None = None
    images: dict | None = None

    @property
    def state(self) -> np.ndarray:
        return self.joint_positions
```

### 3.3 Constants Module

Centralize all hardware constants — joint orders, motor IDs, limits, directions,
gains, and address maps — in a single file.

```python
# constants.py
from __future__ import annotations

from typing import Final

MY_ROBOT_JOINT_ORDER: Final = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

MY_ROBOT_MOTOR_IDS: Final = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    ...
}

MY_ROBOT_JOINT_LIMITS_DEG: Final = {
    "shoulder_pan": (-170.0, 170.0),
    ...
}

VALID_ROLES: Final = frozenset({"leader", "follower"})
```

### 3.4 Key Patterns from Existing Drivers

| Pattern                           | Example                                                                                                                                    |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| **Connection management**         | Store connection state in a private `_connection` field. Use a dataclass to hold port handler, packet handler, and sync read/write groups. |
| **Idempotent connect/disconnect** | `connect()` returns early if already connected. `disconnect()` uses `contextlib.suppress` for cleanup.                                     |
| **Validation in `__init__`**      | Validate all configurable parameters immediately (role, baudrate, adapter type). Raise `ValueError`.                                       |
| **Role-based behavior**           | If `role == "leader"`, `send_action()` raises `RuntimeError`. Torque is disabled on connect.                                               |
| **Observation normalization**     | Convert raw sensor readings (ticks, encoder counts) to degrees or normalized units in `get_observation()`.                                 |
| **Safe disconnect**               | On disconnect, stop all motion first, then optionally hold position with torque, then close the port.                                      |

---

## 4. The `studio_catalog.py` — Plugin Registration

This is the central file that Studio loads. It defines the robot type metadata,
how to build a driver instance, and how to discover/identify devices.

### 4.1 Protocol Helpers

Define these at the top of `studio_catalog.py`. They mirror Studio's internal
protocols and keep your plugin decoupled from Studio internals.

```python
from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol

from physicalai.robot.interface import Robot as PhysicalAIRobot
from pydantic import BaseModel, Field

import my_plugin
from my_plugin import MyRobot, get_urdf_path


class _PortFinder(Protocol):
    """Factory provided by Studio to resolve serial ports."""
    async def find_so101_port(self, robot: Any) -> str: ...
    async def find_port_by_serial(self, serial_number: str) -> str | None: ...
    async def get_calibration_by_id(self, calibration_id: Any) -> Any: ...


class _SerialPortInfo(Protocol):
    connection_string: str | None
    serial_number: str | None


class _PortScanner(Protocol):
    async def find_robots(self) -> None: ...
    @property
    def robots(self) -> list[_SerialPortInfo]: ...
```

### 4.2 `_RobotAdapterOptions`

Controls how Studio wraps the driver in a `PhysicalAIRobotAdapter`.

```python
@dataclass(frozen=True)
class _RobotAdapterOptions:
    include_velocities: bool = False
    goal_time_scale: float = 1.0
    external_effort_gain: float | None = 0.1
```

| Field                  | Meaning                                                                 |
| ---------------------- | ----------------------------------------------------------------------- |
| `include_velocities`   | Whether the observation includes velocity data (for residual learning). |
| `goal_time_scale`      | Multiplier for the `goal_time` argument in `send_action()`.             |
| `external_effort_gain` | Gain for external effort integration (set to `None` to disable).        |

### 4.3 `_CatalogDefinition`

The flat schema (current) places all fields at the top level. This is the schema
used by both the rebot-b601 and lekiwi plugins.

```python
@dataclass
class _CatalogDefinition:
    type: str                                                 # Unique ID, e.g. "MyRobot_Follower"
    display_name: str                                         # Human-readable name
    role: str                                                 # "follower" | "leader"
    urdf_path: str                                            # API path for URDF serving
    urdf_relative_path: str                                   # Relative path within package
    asset_root_resolver: Callable[[], Path] | None            # Resolves URDF root on disk
    robot_builder: Callable[..., Awaitable[PhysicalAIRobot]] | None = None
    robot_model: type | None = None                           # Pydantic model class
    package_map: dict[str, str] = field(default_factory=dict)
    joint_map: dict[str, list[str]] = field(default_factory=dict)
    adapter_options: _RobotAdapterOptions = field(default_factory=_RobotAdapterOptions)
    probe: Any = None

    @property
    def robot_type(self) -> str:
        return self.type
```

| Field                 | Description                                                                                                                           |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `type`                | Unique string identifier used in API routes and database records. Convention: `"Vendor_Model_Role"` (e.g., `"LeKiwi_Follower"`).      |
| `display_name`        | Human-readable label shown in the Studio UI.                                                                                          |
| `role`                | `"follower"` (torque-enabled, full control) or `"leader"` (read-only teleoperation).                                                  |
| `urdf_path`           | The URL path where Studio serves the URDF file. Convention: `/api/robots/catalog/{type}/urdf`.                                        |
| `urdf_relative_path`  | Path relative to the URDF root directory. Example: `"rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf"`.                                   |
| `asset_root_resolver` | Callable that returns the absolute `Path` to the URDF root directory.                                                                 |
| `robot_builder`       | Async callable `(robot, factory) -> PhysicalAIRobot`. Constructs the driver from a validated payload.                                 |
| `robot_model`         | A Pydantic `BaseModel` subclass with `type` (Literal) and `payload` fields. Enables Studio's dynamic discriminated union.             |
| `package_map`         | Maps URDF package names to URL roots for mesh resolution. Example: `{"rebot-b601-dm": "/api/robots/catalog/ReBot_B601_DM_Follower"}`. |
| `joint_map`           | Maps observation joint names to URDF joint names. Example: `{"shoulder_pan.pos": ["joint1"]}`.                                        |
| `adapter_options`     | Controls adapter behavior (velocity inclusion, goal time scaling, external effort).                                                   |
| `probe`               | Object with `discover()`, `identify()`, and `is_online()` methods for device discovery.                                               |

### 4.4 Pydantic Payload Model

Define a Pydantic model for validated per-robot configuration. Use `Field(...)`
to mark required fields.

```python
class MyRobotPayload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)           # Required (no default)
    baudrate: int = 1_000_000
    disable_torque_on_disconnect: bool = True
```

### 4.5 Pydantic Robot Model

A thin wrapper that enables Studio's dynamic discriminated union:

```python
class MyRobot(BaseModel):
    type: Literal["MyRobot_Follower"] = "MyRobot_Follower"
    payload: MyRobotPayload
```

The `type` field uses `Literal` so Pydantic can discriminate between robot types
without an enum. The value must match the `type` in your `_CatalogDefinition`.

### 4.6 Probe Class

The probe handles device discovery and identification. It is optional but
recommended for serial-based robots.

```python
class MyRobotProbe:
    async def discover(self, manager: _PortScanner) -> list[_SerialPortInfo]:
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: dict[str, Any],
        manager: Any = None,
        joint: str | None = None,
    ) -> None:
        # Optional: wiggle a joint or flash an LED for identification
        pass

    async def is_online(self, payload: dict[str, Any], manager: Any = None) -> bool:
        # Optional: check if the robot is currently reachable
        return True


_MY_ROBOT_PROBE = MyRobotProbe()
```

For a more detailed implementation, see the lekiwi plugin's probe which
implements `is_online()` by checking serial port availability.

### 4.7 URDF Root Resolver

The resolver locates the bundled URDF directory regardless of how the package
was installed (editable vs. wheel):

```python
def _get_my_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root
    plugin_package_root = Path(my_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        return site_packages_urdf_root
    return configured_root
```

### 4.8 Joint Map

Maps physical joint keys in the observation to their corresponding URDF joint
names. Values are lists to support URDFs with multiple physical joints per
logical joint (e.g., a gripper driven by two mirrored joints).

```python
_MY_JOINT_MAP: dict[str, list[str]] = {
    "shoulder_pan.pos": ["joint1"],
    "shoulder_lift.pos": ["joint2"],
    "elbow_flex.pos": ["joint3"],
    "wrist_flex.pos": ["joint4"],
    "wrist_roll.pos": ["joint5"],
    "gripper.pos": ["joint6_left", "joint6_right"],
}
```

### 4.9 Robot Builder

An async factory that receives the stored robot config and a `_PortFinder`,
validates the payload, resolves the port, and constructs the driver.

```python
async def _build_my_robot_driver(robot: Any, factory: _PortFinder) -> PhysicalAIRobot:
    # Handle payload in any format (Pydantic, dict, or unknown model)
    raw = robot.payload
    if isinstance(raw, MyRobotPayload):
        validated = raw
    elif isinstance(raw, dict):
        validated = MyRobotPayload.model_validate(raw)
    else:
        validated = MyRobotPayload.model_validate(raw.model_dump(mode="json"))

    serial_number = validated.serial_number
    port = await factory.find_port_by_serial(serial_number)
    if port is None:
        msg = f"Robot not found: {serial_number}"
        raise RuntimeError(msg)

    return MyRobot(port=port, baudrate=validated.baudrate, role="follower")
```

### 4.10 Definitions and Registration

Wire everything together:

```python
def _definitions() -> list[_CatalogDefinition]:
    return [
        _CatalogDefinition(
            type="MyRobot_Follower",
            display_name="My Robot Follower",
            role="follower",
            urdf_path="/api/robots/catalog/MyRobot_Follower/urdf",
            urdf_relative_path="myrobot/urdf/myrobot.urdf",
            asset_root_resolver=_get_my_urdf_root,
            robot_builder=_build_my_robot_driver,
            robot_model=MyRobot,
            package_map={"myrobot": "/api/robots/catalog/MyRobot_Follower"},
            joint_map=_MY_JOINT_MAP,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_MY_ROBOT_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
```

---

## 5. `pyproject.toml` Configuration

### 5.1 Build System and Dependencies

```toml
[build-system]
requires = ["hatchling", "hatch-vcs"]
build-backend = "hatchling.build"

[project]
name = "physicalai-my-plugin"
dynamic = ["version"]
description = "My robot plugin for PhysicalAI"
readme = "README.md"
requires-python = ">=3.12"
license = "Apache-2.0"
dependencies = [
    "physicalai>=0.1.1",
    "numpy>=1.24",
    "loguru>=0.7",
    "pydantic>=2.0",
]

[project.optional-dependencies]
tests = ["pytest"]

[project.urls]
Homepage = "https://github.com/example/physicalai-my-plugin"
Repository = "https://github.com/example/physicalai-my-plugin"
Issues = "https://github.com/example/physicalai-my-plugin/issues"
```

### 5.2 Entry Point Declaration

This is how Studio discovers your plugin:

```toml
[project.entry-points."physicalai.studio.catalog_plugins"]
my-plugin = "my_plugin.studio_catalog:register_physicalai_studio_plugin"
```

The left side (`my-plugin`) is an arbitrary unique name within the entry point
group. The right side is the module path and function name.

### 5.3 Wheel Packaging

```toml
[tool.hatch.version]
source = "vcs"
raw-options = { search_parent_directories = true }

[tool.hatch.build.targets.wheel]
packages = ["src/my_plugin"]

[tool.hatch.build.targets.wheel.force-include]
"urdf" = "urdf"

[tool.hatch.build.targets.sdist]
include = [
    "src/my_plugin/**",
    "urdf/**",
    "README.md",
    "pyproject.toml",
    "LICENSE",
]
```

The `force-include` directive bundles the `urdf/` directory into the wheel so
that URDF and mesh files are available at runtime. Without this, only Python
files under `packages` would be included.

---

## 6. Bundling URDF Assets

The `_urdf.py` module provides a reliable way to locate the bundled URDF
directory regardless of installation mode.

```python
"""URDF path utility for bundled robot description packages."""

from __future__ import annotations

import importlib.resources as ir
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


def get_urdf_path() -> Path:
    """Return the path to the bundled URDF directory."""
    traversal = ir.files("my_plugin")
    with ir.as_file(traversal) as p:
        return p.parent.parent.joinpath("urdf")
```

The fallback logic in the catalog's `asset_root_resolver` (shown in section 4.7)
handles cases where `importlib.resources` returns a different path than expected,
particularly in editable installs vs. installed wheels.

### 6.1 URDF Package References

Your URDF files should reference meshes using `package://` URIs that match
your `package_map` entries. In the catalog definition, `package_map` maps URDF
package names to API URL roots so Studio can serve the mesh files.

```
<!-- Embedded in URDF: -->
<mesh filename="package://rebot-b601-dm/meshes/base_link.STL"/>
```

```python
# In catalog definition:
package_map={"rebot-b601-dm": "/api/robots/catalog/ReBot_B601_DM_Follower"},
```

---

## 7. Testing

### 7.1 Test the Catalog Registration

```python
from __future__ import annotations

from pathlib import Path

import pytest


class _FakeRegistry:
    def __init__(self) -> None:
        self.definitions: list = []

    def register(self, definition: object) -> None:
        self.definitions.append(definition)

    def register_many(self, definitions: list[object]) -> None:
        self.definitions.extend(definitions)


def test_register_plugin() -> None:
    from my_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert len(registry.definitions) == 1
    assert registry.definitions[0].type == "MyRobot_Follower"
    assert registry.definitions[0].role == "follower"
```

### 7.2 Test Payload Models

```python
def test_payload_defaults() -> None:
    from my_plugin.studio_catalog import MyRobotPayload

    payload = MyRobotPayload(serial_number="SN-001")
    assert payload.serial_number == "SN-001"
    assert payload.baudrate == 1_000_000
    assert payload.disable_torque_on_disconnect is True


def test_payload_requires_serial() -> None:
    from my_plugin.studio_catalog import MyRobotPayload

    with pytest.raises(Exception):
        MyRobotPayload()
```

### 7.3 Test Robot Builder

```python
class _StubFactory:
    def __init__(self, port: str | None = "/dev/ttyACM0") -> None:
        self._port = port

    async def find_port_by_serial(self, serial_number: str) -> str | None:
        return self._port


class _StubRobot:
    def __init__(self, payload: object) -> None:
        self.payload = payload


@pytest.mark.anyio
async def test_build_robot_from_pydantic_payload() -> None:
    from my_plugin.studio_catalog import MyRobotPayload, _build_my_robot_driver

    payload = MyRobotPayload(serial_number="SN-001")
    robot = _StubRobot(payload)
    factory = _StubFactory(port="/dev/ttyACM0")
    driver = await _build_my_robot_driver(robot, factory)
    assert driver is not None


@pytest.mark.anyio
async def test_build_robot_from_dict() -> None:
    from my_plugin.studio_catalog import _build_my_robot_driver

    payload = {"serial_number": "SN-002", "baudrate": 115200}
    robot = _StubRobot(payload)
    factory = _StubFactory(port="/dev/ttyACM1")
    driver = await _build_my_robot_driver(robot, factory)
    assert driver is not None


@pytest.mark.anyio
async def test_build_robot_port_not_found() -> None:
    from my_plugin.studio_catalog import MyRobotPayload, _build_my_robot_driver

    payload = MyRobotPayload(serial_number="SN-MISSING")
    robot = _StubRobot(payload)
    factory = _StubFactory(port=None)
    with pytest.raises(RuntimeError, match="Robot not found"):
        await _build_my_robot_driver(robot, factory)
```

### 7.4 Test URDF Path

```python
def test_urdf_path_exists() -> None:
    from my_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "myrobot" / "urdf" / "myrobot.urdf"
    assert urdf_file.exists()
```

### 7.5 Test Robot Driver

Mock the hardware SDK entirely using `sys.modules` patching or `unittest.mock`.
Focus on:

- Construction validation (invalid parameters raise `ValueError`)
- Lifecycle (connect / disconnect are idempotent)
- Observation (returns expected structure, converts units)
- Action (maps/clips/clamps correctly)
- Error handling (not connected, wrong shape, read-only role)

---

## 8. Complete Walkthrough: `physicalai-widget-plugin`

This section builds a minimal but complete plugin step by step.

### 8.1 Project Scaffolding

```
physicalai-widget-plugin/
├── pyproject.toml
├── README.md
├── src/
│   └── physicalai_widget_plugin/
│       ├── __init__.py
│       ├── _urdf.py
│       ├── constants.py
│       ├── widget.py
│       └── studio_catalog.py
├── tests/
│   └── test_studio_catalog.py
└── urdf/
    └── widget/
        ├── meshes/
        │   └── base_link.STL
        └── urdf/
            └── widget.urdf
```

### 8.2 `src/physicalai_widget_plugin/constants.py`

```python
from __future__ import annotations

from typing import Final

WIDGET_JOINT_ORDER: Final = (
    "joint_0",
    "joint_1",
    "joint_2",
)

WIDGET_MOTOR_IDS: Final = {name: i for i, name in enumerate(WIDGET_JOINT_ORDER)}
WIDGET_JOINT_LIMITS_DEG: Final = {name: (-90.0, 90.0) for name in WIDGET_JOINT_ORDER}

VALID_ROLES: Final = frozenset({"leader", "follower"})
```

### 8.3 `src/physicalai_widget_plugin/widget.py`

```python
"""Widget robot driver."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar, Literal

import numpy as np

from physicalai_widget_plugin.constants import VALID_ROLES, WIDGET_JOINT_ORDER

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation


@dataclass
class WidgetObservation:
    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        return self.joint_positions


class Widget:
    JOINT_ORDER: ClassVar[list[str]] = list(WIDGET_JOINT_ORDER)
    NUM_JOINTS: ClassVar[int] = 3

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 1_000_000,
        role: Literal["leader", "follower"] = "follower",
    ) -> None:
        if role not in VALID_ROLES:
            msg = f"Invalid role {role!r}."
            raise ValueError(msg)
        self._port = port
        self._baudrate = baudrate
        self._role = role
        self._connection: object | None = None

    @property
    def joint_names(self) -> list[str]:
        return self.JOINT_ORDER

    def connect(self) -> None:
        if self.is_connected():
            return
        # Open serial port, ping motors, configure torque
        self._connection = object()
        if self._role == "leader":
            self._set_torque(False)

    def disconnect(self) -> None:
        if not self.is_connected():
            return
        # Stop motion, optionally hold, close port
        self._connection = None

    def is_connected(self) -> bool:
        return self._connection is not None

    def get_observation(self) -> RobotObservation:
        # Read joint positions from hardware, convert to degrees
        positions = np.zeros(self.NUM_JOINTS, dtype=np.float32)
        return WidgetObservation(
            joint_positions=positions,
            timestamp=time.monotonic(),
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        if self._role == "leader":
            msg = "Cannot send actions to a leader arm."
            raise RuntimeError(msg)
        if action.shape != (self.NUM_JOINTS,):
            msg = f"Expected action shape ({self.NUM_JOINTS},), got {action.shape}"
            raise ValueError(msg)
        # Clip to limits, convert to raw units, write to motors

    def _set_torque(self, enabled: bool) -> None:
        pass
```

### 8.4 `src/physicalai_widget_plugin/_urdf.py`

```python
from __future__ import annotations

import importlib.resources as ir
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


def get_urdf_path() -> Path:
    traversal = ir.files("physicalai_widget_plugin")
    with ir.as_file(traversal) as p:
        return p.parent.parent.joinpath("urdf")
```

### 8.5 `src/physicalai_widget_plugin/__init__.py`

```python
"""Widget robot plugin for PhysicalAI."""

from __future__ import annotations

from typing import TYPE_CHECKING

from physicalai_widget_plugin._urdf import get_urdf_path as get_urdf_path

if TYPE_CHECKING:
    from physicalai_widget_plugin.widget import Widget as Widget
    from physicalai_widget_plugin.widget import WidgetObservation as WidgetObservation

__all__ = [
    "Widget",
    "WidgetObservation",
    "get_urdf_path",
]


def __getattr__(name: str) -> object:
    if name == "Widget":
        from physicalai_widget_plugin.widget import Widget
        return Widget
    if name == "WidgetObservation":
        from physicalai_widget_plugin.widget import WidgetObservation
        return WidgetObservation
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
```

### 8.6 `src/physicalai_widget_plugin/studio_catalog.py`

```python
"""Studio catalog plugin for Physical AI Studio."""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol

from physicalai.robot.interface import Robot as PhysicalAIRobot
from pydantic import BaseModel, Field

import physicalai_widget_plugin
from physicalai_widget_plugin import Widget, get_urdf_path


class _PortFinder(Protocol):
    async def find_so101_port(self, robot: Any) -> str: ...
    async def find_port_by_serial(self, serial_number: str) -> str | None: ...
    async def get_calibration_by_id(self, calibration_id: Any) -> Any: ...


class _SerialPortInfo(Protocol):
    connection_string: str | None
    serial_number: str | None


class _PortScanner(Protocol):
    async def find_robots(self) -> None: ...
    @property
    def robots(self) -> list[_SerialPortInfo]: ...


@dataclass(frozen=True)
class _RobotAdapterOptions:
    include_velocities: bool = False
    goal_time_scale: float = 1.0
    external_effort_gain: float | None = 0.1


@dataclass
class _CatalogDefinition:
    type: str
    display_name: str
    role: str
    urdf_path: str
    urdf_relative_path: str
    asset_root_resolver: Callable[[], Path] | None
    robot_builder: Callable[..., Awaitable[PhysicalAIRobot]] | None = None
    robot_model: type | None = None
    package_map: dict[str, str] = field(default_factory=dict)
    joint_map: dict[str, list[str]] = field(default_factory=dict)
    adapter_options: _RobotAdapterOptions = field(default_factory=_RobotAdapterOptions)
    probe: Any = None

    @property
    def robot_type(self) -> str:
        return self.type


if TYPE_CHECKING:
    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: _CatalogDefinition) -> None: ...


_WIDGET_TO_URDF: dict[str, list[str]] = {
    "joint_0.pos": ["joint1"],
    "joint_1.pos": ["joint2"],
    "joint_2.pos": ["joint3"],
}


def _get_widget_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root
    plugin_package_root = Path(physicalai_widget_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        return site_packages_urdf_root
    return configured_root


class WidgetPayload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)
    baudrate: int = 1_000_000
    disable_torque_on_disconnect: bool = True


class WidgetRobot(BaseModel):
    type: Literal["Widget_Follower"] = "Widget_Follower"
    payload: WidgetPayload


class WidgetProbe:
    async def discover(self, manager: _PortScanner) -> list[_SerialPortInfo]:
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: dict[str, Any],
        manager: Any = None,
        joint: str | None = None,
    ) -> None:
        pass

    async def is_online(self, payload: dict[str, Any], manager: Any = None) -> bool:
        return True


_WIDGET_PROBE = WidgetProbe()


async def _build_widget_driver(robot: Any, factory: _PortFinder) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, WidgetPayload):
        validated = raw
    elif isinstance(raw, dict):
        validated = WidgetPayload.model_validate(raw)
    else:
        validated = WidgetPayload.model_validate(raw.model_dump(mode="json"))
    serial_number = validated.serial_number
    port = await factory.find_port_by_serial(serial_number)
    if port is None:
        msg = f"Robot not found: {serial_number}"
        raise RuntimeError(msg)
    return Widget(port=port, baudrate=validated.baudrate, role="follower")


def _definitions() -> list[_CatalogDefinition]:
    return [
        _CatalogDefinition(
            type="Widget_Follower",
            display_name="Widget Follower",
            role="follower",
            urdf_path="/api/robots/catalog/Widget_Follower/urdf",
            urdf_relative_path="widget/urdf/widget.urdf",
            asset_root_resolver=_get_widget_urdf_root,
            robot_builder=_build_widget_driver,
            robot_model=WidgetRobot,
            package_map={"widget": "/api/robots/catalog/Widget_Follower"},
            joint_map=_WIDGET_TO_URDF,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_WIDGET_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
```

### 8.7 `pyproject.toml`

```toml
[build-system]
requires = ["hatchling", "hatch-vcs"]
build-backend = "hatchling.build"

[project]
name = "physicalai-widget-plugin"
dynamic = ["version"]
description = "Widget robot plugin for PhysicalAI"
readme = "README.md"
requires-python = ">=3.12"
license = "Apache-2.0"
dependencies = [
    "physicalai>=0.1.1",
    "numpy>=1.24",
    "loguru>=0.7",
    "pydantic>=2.0",
]

[project.optional-dependencies]
tests = ["pytest"]

[project.urls]
Homepage = "https://github.com/example/physicalai-widget-plugin"
Repository = "https://github.com/example/physicalai-widget-plugin"
Issues = "https://github.com/example/physicalai-widget-plugin/issues"

[project.entry-points."physicalai.studio.catalog_plugins"]
widget = "physicalai_widget_plugin.studio_catalog:register_physicalai_studio_plugin"

[tool.hatch.version]
source = "vcs"
raw-options = { search_parent_directories = true }

[tool.hatch.build.targets.wheel]
packages = ["src/physicalai_widget_plugin"]

[tool.hatch.build.targets.wheel.force-include]
"urdf" = "urdf"

[tool.hatch.build.targets.sdist]
include = [
    "src/physicalai_widget_plugin/**",
    "urdf/**",
    "README.md",
    "pyproject.toml",
    "LICENSE",
]
```

### 8.8 `tests/test_studio_catalog.py`

```python
from __future__ import annotations

from pathlib import Path

import pytest


class _FakeRegistry:
    def __init__(self) -> None:
        self.definitions: list = []

    def register(self, definition: object) -> None:
        self.definitions.append(definition)

    def register_many(self, definitions: list[object]) -> None:
        self.definitions.extend(definitions)


class _StubFactory:
    def __init__(self, port: str | None = "/dev/ttyACM0") -> None:
        self._port = port

    async def find_port_by_serial(self, serial_number: str) -> str | None:
        return self._port


class _StubRobot:
    def __init__(self, payload: object) -> None:
        self.payload = payload


def test_register_plugin() -> None:
    from physicalai_widget_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)
    assert len(registry.definitions) == 1
    assert registry.definitions[0].type == "Widget_Follower"
    assert registry.definitions[0].display_name == "Widget Follower"
    assert registry.definitions[0].role == "follower"


def test_payload_defaults() -> None:
    from physicalai_widget_plugin.studio_catalog import WidgetPayload

    payload = WidgetPayload(serial_number="SN-001")
    assert payload.serial_number == "SN-001"
    assert payload.baudrate == 1_000_000
    assert payload.disable_torque_on_disconnect is True


def test_payload_requires_serial() -> None:
    from physicalai_widget_plugin.studio_catalog import WidgetPayload

    with pytest.raises(Exception):
        WidgetPayload()


@pytest.mark.anyio
async def test_build_robot() -> None:
    from physicalai_widget_plugin.studio_catalog import (
        WidgetPayload,
        _build_widget_driver,
    )

    payload = WidgetPayload(serial_number="SN-001")
    robot = _StubRobot(payload)
    factory = _StubFactory(port="/dev/ttyACM0")
    driver = await _build_widget_driver(robot, factory)
    assert driver is not None


@pytest.mark.anyio
async def test_build_robot_port_not_found() -> None:
    from physicalai_widget_plugin.studio_catalog import (
        WidgetPayload,
        _build_widget_driver,
    )

    payload = WidgetPayload(serial_number="SN-MISSING")
    robot = _StubRobot(payload)
    factory = _StubFactory(port=None)
    with pytest.raises(RuntimeError, match="Robot not found"):
        await _build_widget_driver(robot, factory)


def test_urdf_path_exists() -> None:
    from physicalai_widget_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
```

---

## 9. Quick-Reference Checklist

| Item                                       | File                           | Done |
| ------------------------------------------ | ------------------------------ | ---- |
| Create package directory                   | `src/<plugin_package>/`        | ☐    |
| Define constants                           | `constants.py`                 | ☐    |
| Implement robot driver                     | `widget.py` (or similar)       | ☐    |
| Create observation dataclass               | Same file as driver            | ☐    |
| Lazy-load exports via `__getattr__`        | `__init__.py`                  | ☐    |
| Implement `get_urdf_path()`                | `_urdf.py`                     | ☐    |
| Create `_CatalogDefinition`                | `studio_catalog.py`            | ☐    |
| Create Pydantic payload model              | `studio_catalog.py`            | ☐    |
| Create Pydantic robot model                | `studio_catalog.py`            | ☐    |
| Create Probe class                         | `studio_catalog.py`            | ☐    |
| Create async robot builder                 | `studio_catalog.py`            | ☐    |
| Create `register_physicalai_studio_plugin` | `studio_catalog.py`            | ☐    |
| Declare entry point in `pyproject.toml`    | `pyproject.toml`               | ☐    |
| Configure hatchling build                  | `pyproject.toml`               | ☐    |
| `force-include` URDF assets                | `pyproject.toml`               | ☐    |
| Write catalog registration tests           | `tests/test_studio_catalog.py` | ☐    |
| Write robot driver tests                   | `tests/test_widget.py`         | ☐    |

---

## 10. Older Schema Notes

Some plugins (such as `physicalai-zmq-robot-plugin` and
`physicalai-websocket-robot-plugin`) use an earlier version of the catalog
schema. The main differences from the flat schema documented here are:

- **Nested `entry` field**: Instead of flat fields, the definition had a
  `_CatalogEntry` nested dataclass containing `type`, `display_name`, `role`,
  `urdf_path`, `package_map`, and `joint_map`.
- **Separate metadata fields**: `urdf_relative_path`, `package_root`,
  `asset_source`, `asset_root_resolver`, `discover_devices`, and `payload_model`
  lived alongside `entry` rather than being merged into it.
- **`register_many()`**: Older plugins called `registry.register_many(definitions)`
  instead of iterating and calling `registry.register(d)`.
- **No `probe`**: Device discovery was handled via `discover_devices` callable;
  there was no `probe` field for `identify` or `is_online`.
- **No `robot_model`**: The payload model was set on `payload_model`, but there
  was no `robot_model` for Studio's discriminated union.

If you are maintaining an older plugin, the current pattern preferred by Studio
is the flat schema shown in this guide (no `_CatalogEntry` wrapper, no
`register_many`, fields directly on `_CatalogDefinition`, plus `probe` and
`robot_model`).
