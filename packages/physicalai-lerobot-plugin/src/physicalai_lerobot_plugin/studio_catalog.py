# ruff: noqa: PLC0415

"""Studio catalog plugin for LeRobot adapters.

Dynamically registers one ``RobotCatalogDefinition`` per LeRobot robot type,
creating a typed Pydantic payload model for each from the lerobot
``RobotConfig`` dataclass.
"""

from __future__ import annotations

import dataclasses
import types
from typing import TYPE_CHECKING, Any, Literal, Union, get_args, get_origin

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    PortScanner,
    RobotAdapterOptions,
    RobotCatalogDefinition,
    RobotProbe,
    SerialPortInfo,
    robot_field_ui,
    robot_payload_ui,
)
from pydantic import BaseModel, ConfigDict, Field, create_model, model_validator
from serial.tools import list_ports

if TYPE_CHECKING:
    from collections.abc import Callable
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register_robot(self, definition: RobotCatalogDefinition) -> None: ...


# ── Robots to exclude from dynamic registration ────────────────────────────

_ROBOTS_TO_SKIP: frozenset[str] = frozenset({
    "bi_so_follower",                # bimanual — nested sub-arm configs
    "bi_rebot_b601_follower",        # bimanual
    "bi_openarm_follower",           # bimanual
    "lekiwi",                        # separate dedicated plugin
    "lekiwi_client",                 # separate dedicated plugin
    "unitree_g1",                    # full-body humanoid (deferred)
    "mock_robot",                    # test-only
})


# ── Lerobot config importing ───────────────────────────────────────────────

_LEROBOT_CONFIGS_IMPORTED: bool = False


def _ensure_lerobot_configs_imported() -> None:
    """Walk the ``lerobot.robots`` package and import every ``config_*`` module.

    This triggers the ``@RobotConfig.register_subclass(...)`` decorators so
    that ``RobotConfig.get_known_choices()`` returns all available types.
    """
    global _LEROBOT_CONFIGS_IMPORTED  # noqa: PLW0603
    if _LEROBOT_CONFIGS_IMPORTED:
        return
    import importlib
    import pkgutil

    import lerobot.robots

    for _importer, modname, is_pkg in pkgutil.walk_packages(
        lerobot.robots.__path__,
        prefix="lerobot.robots.",
    ):
        if "config" in modname and not is_pkg:
            importlib.import_module(modname)
    _LEROBOT_CONFIGS_IMPORTED = True


# ── Scalar field detection ─────────────────────────────────────────────────

_SIMPLE_SCALARS: frozenset[type] = frozenset({str, int, float, bool})
_OPTIONAL_UNION_LEN: int = 2  # Union[T, None]  has exactly 2 type args


def _is_simple_scalar(typ: type) -> bool:
    """Check whether *typ* is renderable by the schema-driven form.

    Accepts plain ``str``, ``int``, ``float``, ``bool``, ``Literal[...]``,
    and ``Optional[str]`` (or ``str | None``).

    Returns:
        True if the type can be rendered as a form field.
    """
    if typ in _SIMPLE_SCALARS:
        return True
    origin = get_origin(typ)
    if origin is Literal:
        return True
    # Optional[T]  →  Union[T, None]  or  T | None  →  types.UnionType
    if origin in {Union, types.UnionType}:
        args = get_args(typ)
        return bool(len(args) == _OPTIONAL_UNION_LEN and args[1] is type(None) and args[0] in _SIMPLE_SCALARS)
    return False


# ── Base payload with connection fields ────────────────────────────────────


class _LeRobotDynPayloadBase(BaseModel):
    """Base payload for every dynamically registered LeRobot robot.

    Provides the ``connection_string`` / ``serial_number`` device-selector
    fields and their group metadata.
    """

    connection_string: str = Field(
        default="",
        description="Serial port path",
        json_schema_extra=robot_field_ui({
            "group": "connection",
            "widget": "device-selector",
            "device_value": "connection_string",
            "manual_entry": True,
        }),
    )
    serial_number: str = Field(
        default="",
        description="USB serial number",
        json_schema_extra=robot_field_ui({
            "group": "connection",
            "widget": "device-selector",
            "device_value": "serial_number",
            "manual_entry": True,
        }),
    )

    model_config = ConfigDict(
        json_schema_extra=robot_payload_ui({
            "groups": {
                "connection": {
                    "title": "Connection",
                    "device_discovery": True,
                    "stable_key": "serial_number",
                    "fallback_key": "connection_string",
                },
            },
        }),
    )

    @model_validator(mode="after")
    def _validate_identifier(self) -> _LeRobotDynPayloadBase:
        """Require a serial number or a manual serial-port path.

        Returns:
            The validated payload.

        Raises:
            ValueError: If neither identifier is configured.
        """
        if not self.connection_string and not self.serial_number:
            msg = "Either serial_number or connection_string is required"
            raise ValueError(msg)
        return self


# ── Payload model factory ──────────────────────────────────────────────────


def _make_payload_model(config_cls: type) -> type[BaseModel]:
    """Dynamically create a Pydantic payload model from a lerobot ``RobotConfig``.

    The resulting model inherits from :class:`_LeRobotDynPayloadBase` and
    adds one ``Field`` per scalar config attribute (``str``, ``int``,
    ``float``, ``bool``, ``Literal``, ``Optional[str]``).  Complex fields
    (``dict``, ``list``, nested dataclasses, union types with dict, …) are
    omitted and will receive their dataclass defaults at build time.

    Args:
        config_cls: A lerobot ``RobotConfig`` subclass.

    Returns:
        A new Pydantic ``BaseModel`` subclass.
    """
    field_defs: dict[str, tuple[type, Any]] = {}

    for f in dataclasses.fields(config_cls):
        if not _is_simple_scalar(f.type):
            continue
        pydantic_type = f.type
        if f.default is not dataclasses.MISSING:
            field_defs[f.name] = (pydantic_type, Field(default=f.default, description=f.name))
        elif f.default_factory is not dataclasses.MISSING:
            field_defs[f.name] = (pydantic_type, Field(default_factory=f.default_factory, description=f.name))
        else:
            field_defs[f.name] = (pydantic_type, Field(..., description=f.name))

    return create_model(
        f"{config_cls.__name__}Payload",
        __base__=_LeRobotDynPayloadBase,
        **field_defs,
    )


# ── Builder factory ────────────────────────────────────────────────────────


def _make_builder(
    config_cls: type,
    payload_cls: type[BaseModel],
) -> Callable[[PayloadContainer[Any], CatalogRobotFactory], Any]:
    """Create an async builder function for a given lerobot robot type.

    Args:
        config_cls: The lerobot ``RobotConfig`` dataclass to instantiate.
        payload_cls: The corresponding Pydantic payload model.

    Returns:
        An async callable ``(robot, factory) -> PhysicalAIRobot``.
    """
    has_str_port = any(
        f.name == "port" and f.type is str
        for f in dataclasses.fields(config_cls)
    )

    async def _build(
        robot: PayloadContainer[Any],
        factory: CatalogRobotFactory,
    ) -> PhysicalAIRobot:
        from lerobot.robots import make_robot_from_config

        from physicalai_lerobot_plugin.lerobot_adapter import LeRobotAdapter

        raw = robot.payload
        validated = raw if isinstance(raw, payload_cls) else payload_cls.model_validate(raw)

        serial_number = validated.serial_number
        port = await factory.find_port(
            SerialPortInfo(connection_string=validated.connection_string, serial_number=serial_number),
        )
        if port is None:
            msg = f"Robot not found: {serial_number}"
            raise RuntimeError(msg)

        config_kwargs: dict[str, Any] = {}
        for f in dataclasses.fields(config_cls):
            if hasattr(validated, f.name):
                config_kwargs[f.name] = getattr(validated, f.name)

        if has_str_port:
            config_kwargs["port"] = port

        lerobot_config = config_cls(**config_kwargs)
        lerobot_robot = make_robot_from_config(lerobot_config)
        return LeRobotAdapter(lerobot_robot, role="follower")

    return _build


# ── Probe ──────────────────────────────────────────────────────────────────


class LeRobotProbe(RobotProbe[BaseModel]):
    """Probe implementation for LeRobot-adapted serial devices."""

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        """Discover available serial devices.

        Returns:
            list[SerialPortInfo]: Detected serial devices.
        """
        _ = self
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: BaseModel,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Request a visual identify action, if supported."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: BaseModel, manager: PortScanner | None = None) -> bool:
        """Check whether the configured robot endpoint is online.

        Returns:
            bool: ``True`` when the configured serial or port is present.
        """
        _ = self
        if manager is not None:
            ports_list = manager.robots
            serial_number = getattr(payload, "serial_number", None)
            if serial_number:
                return any(p.serial_number == serial_number for p in ports_list)
            port = getattr(payload, "port", None)
            if port:
                return port in {p.connection_string for p in ports_list}
            return False

        all_ports = list_ports.comports()
        serial_number = getattr(payload, "serial_number", None)
        if serial_number:
            return any(p.serial_number == serial_number for p in all_ports)
        port = getattr(payload, "port", None)
        if port:
            return port in {p.device for p in all_ports}
        return False


_LEROBOT_PROBE = LeRobotProbe()


# ── Registration ───────────────────────────────────────────────────────────


def _definitions() -> list[RobotCatalogDefinition]:
    """Build catalog definitions for every supported lerobot robot type.

    Returns:
        A list of ``RobotCatalogDefinition`` instances.
    """
    _ensure_lerobot_configs_imported()
    from lerobot.robots.config import RobotConfig

    defs: list[RobotCatalogDefinition] = []
    for type_str, config_cls in RobotConfig.get_known_choices().items():
        if type_str in _ROBOTS_TO_SKIP:
            continue

        payload_cls = _make_payload_model(config_cls)
        builder = _make_builder(config_cls, payload_cls)

        defs.append(
            RobotCatalogDefinition(
                type=f"LeRobot_{type_str}",
                display_name=f"LeRobot {type_str}",
                role="follower",
                robot_builder=builder,
                robot_payload=payload_cls,
                asset=None,
                adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
                probe=_LEROBOT_PROBE,
            ),
        )
    return defs


def _assert_payload_model_resolvable(model: type[BaseModel]) -> None:
    model.model_rebuild(_types_namespace=globals(), raise_errors=True)


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register LeRobot catalog entries with the Physical AI Studio registry."""
    for definition in _definitions():
        payload_model = definition.robot_payload
        if isinstance(payload_model, type) and issubclass(payload_model, BaseModel):
            _assert_payload_model_resolvable(payload_model)
        registry.register_robot(definition)
