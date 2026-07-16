# Handoff: Upgrade External Catalog Plugins

## Purpose

Update the catalog plugins in `/home/mark/projects/intel/physicalai-rebot-b601-plugin` for Studio's `RobotAsset` catalog contract.

This applies to the `studio_catalog.py` files in these packages:

- `physicalai-rebot-b601-plugin`
- `physicalai-lekiwi-plugin`
- `physicalai-lerobot-plugin`
- `physicalai-bimanual-so101-plugin`
- `physicalai-websocket-robot-plugin`
- `physicalai-zmq-robot-plugin`

The current Studio backend temporarily adapts the old definition shape, so plugins still load. This work removes that compatibility dependency and stops plugins from embedding Studio REST paths.

## Background

Studio loads catalog plugins from the Python entry-point group:

```toml
[project.entry-points."physicalai.studio.catalog_plugins"]
example = "example_plugin.studio_catalog:register_physicalai_studio_plugin"
```

The registration callable receives Studio's registry:

```python
def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
```

Plugins intentionally duck-type the Studio catalog interfaces. Do not import `robots.catalog.types` or `robots.catalog.registry` from Studio. Define matching local dataclasses and protocols in each plugin's `studio_catalog.py`.

## New Asset Contract

Visual-model configuration is now grouped in one `RobotAsset` object.

Studio's equivalent type is:

```python
@dataclass(frozen=True)
class RobotAsset:
    urdf_relative_path: Path
    packages: dict[str, Path]
    joint_map: dict[str, list[str]]
    root_resolver: Callable[[], Path] | None = None
```

Add an equivalent local duck type to every plugin:

```python
@dataclass(frozen=True)
class _RobotAsset:
    urdf_relative_path: Path
    packages: dict[str, Path]
    joint_map: dict[str, list[str]]
    root_resolver: Callable[[], Path] | None = None
```

Update the local `_CatalogDefinition` to remove all legacy asset fields:

```python
@dataclass
class _CatalogDefinition:
    type: str
    display_name: str
    role: str
    robot_builder: Callable[..., Awaitable[PhysicalAIRobot]] | None = None
    robot_payload: type[BaseModel] | None = None
    asset: _RobotAsset | None = None
    adapter_options: _RobotAdapterOptions = field(default_factory=_RobotAdapterOptions)
    probe: Any = None

    @property
    def robot_type(self) -> str:
        return self.type
```

Remove these fields from `_CatalogDefinition` and all definition instances:

- `urdf_path`
- `urdf_relative_path`
- `asset_root_resolver`
- `package_map`
- `joint_map`

`/api/...` URLs must not appear in plugin catalog definitions after this migration. Studio creates API URLs from the robot type.

## Robot Payload Schema

`robot_payload` must be a Pydantic `BaseModel` subclass. It is the only schema owned by a plugin: Studio constructs the enclosing robot model with its standard `id`, `name`, `type`, timestamps, and calibration fields.

Studio exposes the payload JSON Schema at:

```text
GET /api/robots/catalog/{robot_type}/schema
```

Use the Pydantic payload model directly in each definition. Do not register a full robot model containing a `type` and `payload` field.

Studio returns `robot_payload.model_json_schema()`. A later UI feature will use `Field(..., json_schema_extra=...)` metadata to decide how to render configuration fields, such as displaying `serial_number` as a robot picker. Do not add a Studio-specific metadata convention yet; standard Pydantic field descriptions, examples, and schema extras will be preserved when that convention is introduced.

## Package Mappings

`RobotAsset.packages` maps a package identifier used by a URDF's `package://` URI to a relative directory below `root_resolver()`.

For a URI such as:

```text
package://stararm102/meshes/base_link.STL
```

the asset must contain:

```python
packages={"stararm102": Path("stararm102")}
```

Studio exposes that package at:

```text
/api/robots/catalog/{robot_type}/packages/stararm102
```

Do not put that URL in the plugin. Studio returns it to the UI in the catalog API response.

The mapping matters when the ROS package name differs from the on-disk directory. The built-in WidowX catalog is the reference case:

```python
_TROSSEN_ASSET = _RobotAsset(
    urdf_relative_path=Path("widowx/urdf/generated/wxai/wxai_follower.urdf"),
    packages={"trossen_arm_description": Path("widowx")},
    joint_map=_TROSSEN_TO_URDF,
)
```

## ReBot Example

Keep the existing `_get_rebot_urdf_root()` function. It supplies the asset root for both ReBot models.

```python
_REBOT_B601_DM_ASSET = _RobotAsset(
    urdf_relative_path=Path("rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf"),
    packages={"rebot-b601-dm": Path("rebot-b601-dm")},
    joint_map=_REBOT_B601_DM_TO_URDF,
    root_resolver=_get_rebot_urdf_root,
)

_REBOT_ARM102_ASSET = _RobotAsset(
    urdf_relative_path=Path("stararm102/urdf/stararm102_description.urdf"),
    packages={"stararm102": Path("stararm102")},
    joint_map=_REBOT_ARM102_TO_URDF,
    root_resolver=_get_rebot_urdf_root,
)
```

Definitions then contain only the asset object:

```python
_CatalogDefinition(
    type="ReBot_B601_DM_Follower",
    display_name="ReBot B601 DM Follower",
    role="follower",
    robot_builder=_build_rebot_b601_dm_driver,
    robot_payload=ReBotB601DMPayload,
    asset=_REBOT_B601_DM_ASSET,
    adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
    probe=_REBOT_PROBE,
)
```

Use `_REBOT_ARM102_ASSET` for `ReBot_Arm102_Leader`.

## Other Plugin Assets

Apply the same conversion with these current locations:

| Plugin         | URDF path                    | Package mapping                      |
| -------------- | ---------------------------- | ------------------------------------ |
| LeKiwi         | `lekiwi/urdf/LeKiwi.urdf`    | `{"lekiwi": Path("lekiwi")}`         |
| LeRobot        | `lerobot/urdf/lerobot.urdf`  | `{"lerobot": Path("lerobot")}`       |
| Bimanual SO101 | `so101_dual/so101_dual.urdf` | `{"so101_dual": Path("so101_dual")}` |

Keep each plugin's existing `get_urdf_path()` fallback function as the `root_resolver`.

## Plugins Without Visual Assets

WebSocket and ZMQ catalog plugins currently declare no URDF or meshes. Remove their legacy empty/`None` fields and set:

```python
asset=None
```

Their catalog API response will contain an empty URDF path and empty package/joint maps. Do not invent a placeholder asset.

## Validation

From `/home/mark/projects/intel/physicalai-rebot-b601-plugin`:

```bash
uv run ruff check packages/*/src/*/studio_catalog.py
```

Then reinstall plugin packages into the Studio backend environment:

```bash
uv sync --reinstall-package physicalai-rebot-b601-plugin --reinstall-package physicalai-lekiwi-plugin
```

From `/home/mark/projects/intel/physical-ai-studio/application/backend`, verify registration and the catalog URLs:

```bash
uv run --no-sync python -c "from robots.catalog.registry import RobotCatalogRegistry; print([d.type for d in RobotCatalogRegistry().list_definitions()])"
uv run --no-sync pytest tests/robots/test_catalog_assets.py
```

For each visual plugin, verify:

- `GET /api/robots/catalog` returns the expected `urdf_path` and `package_map`.
- `GET /api/robots/catalog/{robot_type}/urdf` returns the URDF.
- `GET /api/robots/catalog/{robot_type}/packages/{package_name}/{asset_path}` returns a mesh referenced by the URDF.

## Compatibility Note

Studio currently converts the old plugin fields into `RobotAsset` at registration time. This is temporary migration support only. Updated plugins should use `asset` directly so Studio can eventually remove the legacy adapter.
