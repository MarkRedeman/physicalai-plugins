# Handoff: External Robot Catalog Plugins (Standalone Guide)

## Objective

Implement robot catalog plugins so Studio discovers robot integrations from installed Python packages.

Primary target: move ReBot registration out of Studio core into `physicalai-rebot-b601-plugin`.

This document is self-contained and includes the interfaces, expected behavior, and example code.

---

## Desired End State

- Studio discovers plugins through Python entry points (`physicalai.studio.catalog_plugins`).
- Each plugin exposes one callable: `register_physicalai_studio_plugin(registry)`.
- ReBot robot types are registered by the external ReBot package, not by Studio source code.
- Plugin definitions include:
  - `robot_builder` (returns a `physicalai.robot.interface.Robot` driver)
  - `payload_model` (Pydantic payload validation model)
  - `adapter_options` (runtime adapter behavior)
- Studio catalog API behavior remains unchanged for UI consumers.

---

## Core Protocol and Data Model to Implement

The plugin callable and catalog definition shape should look like this.

```python
# plugin_api.py (or equivalent stable API surface)
from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Protocol

from pydantic import BaseModel
from physicalai.robot.interface import Robot as PhysicalAIRobot


class SerialPortInfo(Protocol):
    connection_string: str
    serial_number: str
    robot_type: str


class RobotCatalogEntry(Protocol):
    # Keep fields aligned with Studio's catalog response model
    type: str
    display_name: str
    role: Literal["follower", "leader"]
    urdf_path: str | None
    package_map: dict[str, str]
    joint_map: dict[str, list[str]]


AssetSource = Literal["builtin", "plugin"]
DiscoverDevicesCallable = Callable[[list[SerialPortInfo]], Awaitable[list[SerialPortInfo]]]
AssetRootResolver = Callable[[], Path]
BuildRobotCallable = Callable[[object, object], Awaitable[PhysicalAIRobot]]
PayloadModelType = type[BaseModel]


@dataclass(frozen=True)
class RobotAdapterOptions:
    include_velocities: bool = False
    goal_time_scale: float = 1.0
    external_effort_gain: float | None = 0.1


@dataclass(frozen=True)
class RobotCatalogDefinition:
    entry: RobotCatalogEntry
    urdf_relative_path: Path | None
    package_root: Path | None
    asset_source: AssetSource
    asset_root_resolver: AssetRootResolver | None
    discover_devices: DiscoverDevicesCallable
    robot_builder: BuildRobotCallable | None = None
    payload_model: PayloadModelType | None = None
    adapter_options: RobotAdapterOptions = RobotAdapterOptions()


class RobotCatalogRegistry(Protocol):
    def register(self, definition: RobotCatalogDefinition) -> None: ...
    def register_many(self, definitions: list[RobotCatalogDefinition]) -> None: ...


class RegisterPhysicalAIStudioPlugin(Protocol):
    def __call__(self, registry: RobotCatalogRegistry) -> None: ...
```

Plugin modules must export:

```python
def register_physicalai_studio_plugin(registry: RobotCatalogRegistry) -> None:
    ...
```

---

## Studio-Side Loader (Entry Point Discovery)

Studio should discover plugins using `importlib.metadata.entry_points`.

```python
from importlib.metadata import entry_points

CATALOG_PLUGIN_ENTRYPOINT_GROUP = "physicalai.studio.catalog_plugins"


def load_catalog_plugins(registry) -> None:
    for ep in entry_points(group=CATALOG_PLUGIN_ENTRYPOINT_GROUP):
        register = ep.load()
        if not callable(register):
            raise ValueError(
                f"Catalog plugin entry point '{ep.name}' must load a callable "
                "register_physicalai_studio_plugin(registry)"
            )
        register(registry)
```

Important:

- Keep duplicate robot type protection in `registry.register(...)`.
- If plugin loading failure policy is non-fatal, catch exceptions, log clearly, continue.

---

## ReBot Plugin Implementation (External Package)

In `physicalai-rebot-b601-plugin`, add a module such as:

- `physicalai_rebot_b601_plugin/studio_catalog.py`

Example structure:

```python
from __future__ import annotations

from pathlib import Path

import physicalai_rebot_b601_plugin
from loguru import logger
from pydantic import BaseModel, Field
from physicalai_rebot_b601_plugin import ReBotArm102Leader, ReBotB601DM, get_urdf_path


def _get_rebot_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root

    plugin_package_root = Path(physicalai_rebot_b601_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        logger.warning(
            "ReBot plugin get_urdf_path() returned missing path=%s; falling back to %s",
            configured_root,
            site_packages_urdf_root,
        )
        return site_packages_urdf_root

    return configured_root


class ReBotB601DMPayload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)
    can_adapter: str = "damiao"
    dm_serial_baud: int = 921600
    disable_torque_on_disconnect: bool = True
    force_pos_torque_ratio: float = 0.1


class ReBotArm102Payload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)
    baudrate: int = 1_000_000
    unlock_on_connect: bool = True
    reset_multi_turn_on_connect: bool = True
    zero_on_connect: bool = False


async def _discover_rebot_devices(devices):
    return devices


async def _build_rebot_b601_dm_driver(robot, factory):
    payload = robot.payload.model_dump(mode="json")
    serial_number = str(payload["serial_number"])
    port = await factory.find_port_by_serial(serial_number)
    if port is None:
        raise RuntimeError(f"Robot not found: {serial_number}")

    return ReBotB601DM(
        port=port,
        can_adapter=str(payload.get("can_adapter", "damiao")),
        dm_serial_baud=int(payload.get("dm_serial_baud", 921600)),
        role="follower",
        disable_torque_on_disconnect=bool(payload.get("disable_torque_on_disconnect", True)),
        force_pos_torque_ratio=float(payload.get("force_pos_torque_ratio", 0.1)),
    )


async def _build_rebot_arm102_driver(robot, factory):
    payload = robot.payload.model_dump(mode="json")
    serial_number = str(payload["serial_number"])
    port = await factory.find_port_by_serial(serial_number)
    if port is None:
        raise RuntimeError(f"Robot not found: {serial_number}")

    return ReBotArm102Leader(
        port=port,
        baudrate=int(payload.get("baudrate", 1_000_000)),
        unlock_on_connect=bool(payload.get("unlock_on_connect", True)),
        reset_multi_turn_on_connect=bool(payload.get("reset_multi_turn_on_connect", True)),
        zero_on_connect=bool(payload.get("zero_on_connect", False)),
    )


def _definitions() -> list:
    return [
        # DM follower
        # - urdf_relative_path: Path("rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf")
        # - package_root: Path("rebot-b601-dm")
        # - package_map: {"rebot-b601-dm": "/api/robots/catalog/ReBot_B601_DM_Follower"}
        # - robot_builder: _build_rebot_b601_dm_driver
        # - payload_model: ReBotB601DMPayload
        # - adapter_options: include_velocities=True, external_effort_gain=None

        # Arm102 leader
        # - urdf_relative_path: Path("stararm102/urdf/stararm102_description.urdf")
        # - package_root: Path("stararm102")
        # - package_map: {"stararm102": "/api/robots/catalog/ReBot_Arm102_Leader"}
        # - robot_builder: _build_rebot_arm102_driver
        # - payload_model: ReBotArm102Payload
        # - adapter_options: include_velocities=False, external_effort_gain=None
    ]


def register_physicalai_studio_plugin(registry) -> None:
    registry.register_many(_definitions())
```

Important for current Studio integration:

- `robot_builder` must return a `physicalai.robot.interface.Robot` driver, not a Studio `RobotClient`.
- Studio wraps that driver in `PhysicalAIRobotAdapter` using `entry.role` and `adapter_options`.
- ReBot DM and Arm102 must point to distinct URDF/package roots (DM != stararm102).

---

## Entry Point Declaration (Must Be in Plugin Package)

In **plugin** `pyproject.toml` (not Studio):

```toml
[project.entry-points."physicalai.studio.catalog_plugins"]
rebot-b601 = "physicalai_rebot_b601_plugin.studio_catalog:register_physicalai_studio_plugin"
```

This is the mechanism Studio uses to discover the plugin registration function.

---

## Clarification About Studio `pyproject.toml`

If you see an entry point in Studio pointing to an in-repo ReBot module, treat it as temporary migration scaffolding only.

Final expected state:

- remove Studio-side ReBot entry point
- remove in-repo ReBot catalog registration module
- keep only plugin-side entry point above

---

## Migration Checklist

1. Add `register_physicalai_studio_plugin(registry)` to ReBot plugin package.
2. Register both ReBot definitions with:
   - `robot_builder`
   - `payload_model`
   - `adapter_options`
3. Ensure URDF packaging includes both:
   - `rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf`
   - `stararm102/urdf/stararm102_description.urdf`
4. Add plugin entry point in ReBot plugin `pyproject.toml`.
5. Release/publish plugin version and update Studio dependency if needed.
6. Remove Studio in-repo ReBot registration module.
7. Remove Studio temporary ReBot entry point/fallback logic.
8. Verify catalog endpoints still expose ReBot entries and URDF/assets.

---

## Validation

Expected runtime checks:

- ReBot types appear in `GET /api/robots/catalog`.
- `GET /api/robots/catalog/ReBot_Arm102_Leader/urdf` returns 200.
- `GET /api/robots/catalog/ReBot_B601_DM_Follower/urdf` returns 200.
- `GET /api/robots/catalog/ReBot_B601_DM_Follower/discover` returns expected payload.

Recommended backend test command:

```bash
uv run pytest tests/api/test_robot_catalog.py tests/api/test_hardware.py
```

---

## Known Follow-Up Work (Not Required for Plugin Loading)

- Move from hardcoded `RobotType` enum to generic `type_id` string model.
- Add `payload_schema`/`payload_defaults`/`payload_ui_schema` to catalog entries.
- Migrate UI robot forms to schema-driven rendering.
