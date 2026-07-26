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


# ── Follower → Leader teleoperator mapping ──────────────────────────────────

_FOLLOWER_TO_LEADER: dict[str, str] = {
    "so100_follower": "so100_leader",
    "so101_follower": "so101_leader",
    "koch_follower": "koch_leader",
    "omx_follower": "omx_leader",
    "openarm_follower": "openarm_leader",
    "rebot_b601_follower": "rebot_102_leader",
    "reachy2": "reachy2_teleoperator",
}


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

_REQUIRED_SENTINEL: Any = object()


def _resolve_field_type(
    f: dataclasses.Field[Any],
) -> tuple[type, Any]:
    """Resolve a dataclass field into a (pydantic_type, default) pair.

    Nullable strings (``str | None``) are flattened to plain ``str`` with
    ``default=""`` so the JSON Schema uses ``type: string`` instead of
    ``anyOf: [string, null]`` — the latter is not cleanly rendered by the
    Physical AI Studio form UI.

    Returns:
        A tuple of (pydantic_type, default_value).
    """
    origin = get_origin(f.type)
    is_nullable_str = origin in {Union, types.UnionType} and get_args(f.type) == (str, type(None))

    if is_nullable_str:
        pydantic_type = str
        default_val = f.default if f.default is not dataclasses.MISSING and f.default is not None else ""
    else:
        pydantic_type = f.type
        if f.default is not dataclasses.MISSING:
            default_val = f.default
        elif f.default_factory is not dataclasses.MISSING:
            default_val = f.default_factory()
        else:
            default_val = _REQUIRED_SENTINEL

    return pydantic_type, default_val


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

        pydantic_type, default_val = _resolve_field_type(f)

        if default_val is _REQUIRED_SENTINEL or f.name == "id":
            field_defs[f.name] = (pydantic_type, Field(..., description=f.name))
        else:
            field_defs[f.name] = (pydantic_type, Field(default=default_val, description=f.name))

    return create_model(
        f"{config_cls.__name__}Payload",
        __base__=_LeRobotDynPayloadBase,
        **field_defs,
    )


_LEROBOT_TELEOPERATORS_IMPORTED: bool = False


def _ensure_lerobot_teleoperators_imported() -> None:
    """Walk the ``lerobot.teleoperators`` package and import every ``config_*`` module.

    This triggers the ``@TeleoperatorConfig.register_subclass(...)`` decorators
    so that ``TeleoperatorConfig.get_known_choices()`` returns all types.
    """
    global _LEROBOT_TELEOPERATORS_IMPORTED  # noqa: PLW0603
    if _LEROBOT_TELEOPERATORS_IMPORTED:
        return
    import importlib
    import pkgutil

    import lerobot.teleoperators

    for _importer, modname, is_pkg in pkgutil.walk_packages(
        lerobot.teleoperators.__path__,
        prefix="lerobot.teleoperators.",
    ):
        if "config" in modname and not is_pkg:
            importlib.import_module(modname)
    _LEROBOT_TELEOPERATORS_IMPORTED = True


# ── Builder factory ────────────────────────────────────────────────────────


def _make_builder(
    config_cls: type,
    payload_cls: type[BaseModel],
    role: str,
) -> Callable[[PayloadContainer[Any], CatalogRobotFactory], Any]:
    """Create an async builder function for a given lerobot robot type.

    Args:
        config_cls: The lerobot ``RobotConfig`` dataclass to instantiate.
        payload_cls: The corresponding Pydantic payload model.
        role: ``"follower"`` or ``"leader"``.

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
        if isinstance(raw, BaseModel) and type(raw) is not payload_cls:
            raw = raw.model_dump()
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
        return LeRobotAdapter(
            config_cls, config_kwargs, role=role, _robot=lerobot_robot,
        )

    return _build


def _make_teleop_builder(
    config_cls: type,
    payload_cls: type[BaseModel],
    role: str,
) -> Callable[[PayloadContainer[Any], CatalogRobotFactory], Any]:
    """Create an async builder for a lerobot teleoperator.

    Args:
        config_cls: The lerobot ``TeleoperatorConfig`` dataclass to instantiate.
        payload_cls: The corresponding Pydantic payload model.
        role: ``"leader"`` or ``"follower"``.

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
        from lerobot.teleoperators import make_teleoperator_from_config

        from physicalai_lerobot_plugin.lerobot_adapter import (
            LeRobotTeleoperatorAdapter,
        )

        raw = robot.payload
        if isinstance(raw, BaseModel) and type(raw) is not payload_cls:
            raw = raw.model_dump()
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

        teleop_config = config_cls(**config_kwargs)
        teleoperator = make_teleoperator_from_config(teleop_config)
        return LeRobotTeleoperatorAdapter(
            config_cls, config_kwargs, role=role, _teleoperator=teleoperator,
        )

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

    Followers are built from ``RobotConfig`` + ``make_robot_from_config``.
    Leaders (when a matching teleoperator exists) are built from
    ``TeleoperatorConfig`` + ``make_teleoperator_from_config``.

    Returns:
        A list of ``RobotCatalogDefinition`` instances.
    """
    _ensure_lerobot_configs_imported()
    _ensure_lerobot_teleoperators_imported()
    from lerobot.robots.config import RobotConfig
    from lerobot.teleoperators.config import TeleoperatorConfig

    teleop_choices = TeleoperatorConfig.get_known_choices()

    defs: list[RobotCatalogDefinition] = []
    for type_str, config_cls in RobotConfig.get_known_choices().items():
        if type_str in _ROBOTS_TO_SKIP:
            continue

        display_name = f"LeRobot {type_str}"
        payload_cls = _make_payload_model(config_cls)

        # Follower — type = LeRobot_{follower_name}
        follower_builder = _make_builder(config_cls, payload_cls, "follower")
        defs.append(
            RobotCatalogDefinition(
                type=f"LeRobot_{type_str}",
                display_name=f"{display_name} Follower",
                role="follower",
                robot_builder=follower_builder,
                robot_payload=payload_cls,
                asset=None,
                adapter_options=RobotAdapterOptions(include_velocities=False, external_effort_gain=None),
                probe=_LEROBOT_PROBE,
            ),
        )

        # Leader (via teleoperator config) — type = LeRobot_{teleop_name}
        leader_teleop_type = _FOLLOWER_TO_LEADER.get(type_str)
        if leader_teleop_type is not None and leader_teleop_type in teleop_choices:
            teleop_config_cls = teleop_choices[leader_teleop_type]
            leader_payload_cls = _make_payload_model(teleop_config_cls)
            leader_builder = _make_teleop_builder(teleop_config_cls, leader_payload_cls, "leader")
            defs.append(
                RobotCatalogDefinition(
                    type=f"LeRobot_{leader_teleop_type}",
                    display_name=f"{display_name} Leader",
                    role="leader",
                    robot_builder=leader_builder,
                    robot_payload=leader_payload_cls,
                    asset=None,
                    adapter_options=RobotAdapterOptions(include_velocities=False, external_effort_gain=None),
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
