# LeRobot Ecosystem Plugin Report

Date: 2026-07-30

This document summarizes third-party Python packages that extend the LeRobot ecosystem and evaluates if/how they can be used inside `physicalai-lerobot-plugin`.

## Scope

Analyzed packages:

- https://pypi.org/project/lerobot-teleoperator-spacemouse/
- https://pypi.org/project/lerobot-quality-gates/
- https://pypi.org/project/lerobot-teleoperator-rebot-arm-102/
- https://pypi.org/project/lerobot-robot-viola/
- https://pypi.org/project/lerobot-robot-yam/
- https://pypi.org/project/lerobot-robot-livekit/
- https://pypi.org/project/lerobot-teleoperator-stararm102/
- https://pypi.org/project/lerobot-teleoperator-pipermate/
- https://pypi.org/project/lerobot-camera-berxel/
- https://pypi.org/project/lerobot-motor-starai/
- https://pypi.org/project/lerobot-teleoperator-hex-arm/
- https://pypi.org/project/lerobot-robot-bimanual-follower/
- https://pypi.org/project/lerobot-teleoperator-bimanual-leader/
- https://github.com/pham-tuan-binh/leslider

Cross-check source:

- https://huggingface.co/docs/lerobot/main/en/third_party_robots

## Package Overview

Notes:

- `Robot types` counts follower/robot integrations only.
- Teleoperator/camera/motor/QA packages show `0` robot types even when useful.
- `Registered IDs` are the discovered LeRobot registration keys where detectable.
- `GitHub` links come from PyPI `project_urls` when available; otherwise they are best-match public repos from GitHub search.

| Package                                | PyPI                                                           | GitHub                                                                                        | Category                        | Registered IDs                         | Robot types | Author(s)                    | Useful notes                                           | Works in `physicalai-lerobot-plugin` today?   |
|----------------------------------------|----------------------------------------------------------------|-----------------------------------------------------------------------------------------------|---------------------------------|----------------------------------------|------------:|------------------------------|--------------------------------------------------------|-----------------------------------------------|
| `lerobot-teleoperator-spacemouse`      | https://pypi.org/project/lerobot-teleoperator-spacemouse/      | https://github.com/Jas000n/lerobot-teleoperator-spacemouse                                    | Teleoperator                    | `spacemouse`                           |           0 | Shunyu Yao                   | Good docs, IK + EEF modes, SpaceMouse support          | No (not auto-discovered)                      |
| `lerobot-quality-gates`                | https://pypi.org/project/lerobot-quality-gates/                | https://github.com/auraoneai/lerobot-quality-gates                                            | Dataset QA CLI                  | none                                   |           0 | AuraOne                      | Dataset validation gate, CI/reporting focused          | Not applicable to robot catalog               |
| `lerobot-teleoperator-rebot-arm-102`   | https://pypi.org/project/lerobot-teleoperator-rebot-arm-102/   | https://github.com/KillingJacky/lerobot-teleoperator-rebot-arm-102                            | Teleoperator                    | `rebot_arm_102_leader`                 |           0 | Jack Shao                    | reBot Arm 102 leader for B601 follower workflows       | No (not auto-discovered)                      |
| `lerobot-robot-viola`                  | https://pypi.org/project/lerobot-robot-viola/                  | https://github.com/servodevelop/fashionstar-lerobot-robot-viola (official docs naming)        | Robot                           | `lerobot_robot_viola`                  |           1 | Welt-liu                     | Depends on `lerobot-motor-starai` and Fashion Star SDK | No (not auto-discovered)                      |
| `lerobot-robot-yam`                    | https://pypi.org/project/lerobot-robot-yam/                    | https://github.com/pravsels/lerobot_yam                                                       | Robot                           | `yam_follower`                         |           1 | Praveen Selvaraj             | YAM follower plugin                                    | No (not auto-discovered)                      |
| `lerobot-robot-livekit`                | https://pypi.org/project/lerobot-robot-livekit/                | No public repository found from PyPI metadata or GitHub search                                | Robot                           | `livekit`                              |           1 | (PyPI owner: `binhpham_lk`)  | LiveKit portal/operator-side integration               | No (not auto-discovered)                      |
| `lerobot-teleoperator-stararm102`      | https://pypi.org/project/lerobot-teleoperator-stararm102/      | No public repository found from PyPI metadata or GitHub search                                | Teleoperator                    | `lerobot_teleoperator_stararm102`      |           0 | Welt-liu                     | Minimal metadata/docs                                  | No (not auto-discovered)                      |
| `lerobot-teleoperator-pipermate`       | https://pypi.org/project/lerobot-teleoperator-pipermate/       | No public repository found from PyPI metadata or GitHub search                                | Teleoperator                    | `lerobot_teleoperator_pipermate`       |           0 | Welt-liu                     | Minimal metadata/docs                                  | No (not auto-discovered)                      |
| `fashionstar-lerobot-robot-cello`      | n/a (name listed in official docs)                             | https://github.com/servodevelop/fashionstar-lerobot-robot-cello                               | Robot                           | unknown (not inspected in this report) |           1 | FashionStar community         | Mentioned by official docs and used by bimanual setup  | Unknown (not yet validated)                   |
| `lerobot-camera-berxel`                | https://pypi.org/project/lerobot-camera-berxel/                | https://github.com/hexfellow/hex_lerobot_drivers                                              | Camera                          | `berxel` (camera config)               |           0 | Dong Zhaorui (+ maintainers) | Camera extension package                               | Not part of current robot/teleop catalog flow |
| `lerobot-motor-starai`                 | https://pypi.org/project/lerobot-motor-starai/                 | https://github.com/servodevelop/fashionstar-lerobot-motor-starai (best match)                 | Motor                           | none detected                          |           0 | Hunk                         | Base motor dependency used by multiple StarAI plugins  | Indirect only                                 |
| `lerobot-teleoperator-hex-arm`         | https://pypi.org/project/lerobot-teleoperator-hex-arm/         | https://github.com/hexfellow/hex_lerobot_drivers                                              | Teleoperator                    | `hex_arm_leader`                       |           0 | Dong Zhaorui (+ maintainers) | HEX leader teleop plugin                               | No (not auto-discovered)                      |
| `lerobot-teleoperator-livekit`         | n/a (listed in official docs)                                  | No public repository found from docs/PyPI metadata in this report                              | Teleoperator                    | unknown (not inspected in this report) |           0 | Unknown                       | Official docs list this as networked remote teleop     | Unknown (not yet validated)                   |
| `lerobot-robot-bimanual-follower`      | https://pypi.org/project/lerobot-robot-bimanual-follower/      | https://github.com/servodevelop/fashionstar-lerobot-robot-bimanual-follower (best match)      | Robot (bimanual wrapper)        | `lerobot_robot_bimanual_follower`      |           1 | Welt-liu                     | Composes two robot plugins (`viola` + `cello`)         | No (not auto-discovered)                      |
| `lerobot-teleoperator-bimanual-leader` | https://pypi.org/project/lerobot-teleoperator-bimanual-leader/ | https://github.com/servodevelop/fashionstar-lerobot-teleoperator-bimanual-leader (best match) | Teleoperator (bimanual wrapper) | `lerobot_teleoperator_bimanual_leader` |           0 | Welt-liu                     | Composes two teleop plugins (`violin`)                 | No (not auto-discovered)                      |
| `leslider` (workspace) | n/a (GitHub workspace) | https://github.com/pham-tuan-binh/leslider | Robot + Teleoperator | `so101_slider_follower`, `so101_slider_pos_follower`, `so101_with_slider_leader`, `so101_with_slider_pos_leader` | 2 | Pham Tuan Binh | SO-101 slider add-on; source install via `uv sync` | No (not auto-discovered) |

### GitHub Link Confidence

- High confidence: links sourced directly from PyPI `project_urls`.
- Medium confidence: links marked `(best match)` came from GitHub search because PyPI metadata did not include repository URLs.
- Missing: some packages have no discoverable repository URL in PyPI metadata and no unambiguous search match.

### Additional Lookup Attempt (2026-07-30)

- We inspected package sdists/wheels (`PKG-INFO`, `METADATA`, `README`, `setup.py`) for embedded GitHub URLs.
- We also ran GitHub repository searches with package-name and variant queries.
- Remaining unresolved packages after this pass:
  - `lerobot-robot-livekit`
  - `lerobot-teleoperator-stararm102`
  - `lerobot-teleoperator-pipermate`
  - `lerobot-teleoperator-livekit`

## Official Docs Cross-check

What we missed before reviewing LeRobot docs:

- LeRobot's third-party docs explicitly state drop-in auto-discovery by import-name prefix:
  - `lerobot_robot_`
  - `lerobot_teleoperator_`
- This means correctly packaged third-party plugins can be picked up by LeRobot once installed and imported.
- LeSlider fits this convention (`lerobot_robot_so101_slider*`, `lerobot_teleoperator_so101_with_slider*`).

Corrections applied:

- Added `leslider` and documented its four registered IDs.
- Added `fashionstar-lerobot-robot-cello` because it is part of the FashionStar ecosystem and referenced by bimanual follower workflows.
- Distinguished `lerobot-teleoperator-livekit` (official docs entry) from `lerobot-robot-livekit` (separate package analyzed earlier).
- Updated Viola GitHub link to the naming used in official docs (`servodevelop/fashionstar-lerobot-robot-viola`).

Still out of scope in this file:

- Many additional integrations listed in official docs (UR5e variants, Lebai, Trossen, Piper variants, ROS2 bridges, etc.) are not yet individually analyzed here.
- We can add a second pass table for those if we want a complete ecosystem index.

## Compatibility With `physicalai-lerobot-plugin`

Current behavior in our plugin:

- It discovers built-in LeRobot configs by walking/importing `lerobot.robots.*config*` and `lerobot.teleoperators.*config*`.
- Most external packages above install modules like `lerobot_robot_viola` or `lerobot_teleoperator_hex_arm` (outside the `lerobot.*` namespace).
- LeRobot official docs indicate third-party plugin discovery by top-level package prefixes (`lerobot_robot_*`, `lerobot_teleoperator_*`), which is broader than our current walk.

Result:

- These plugins can be compatible with LeRobot itself, but they are not auto-loaded by our current discovery path.
- Therefore they usually do not appear in Studio catalog definitions unless explicitly imported first.

## `register_subclass(...)` Pattern

Why this matters:

- LeRobot uses decorator-based registries (for robots/teleoperators/cameras).
- A type becomes selectable only after the module containing the decorator is imported.

Example from LeSlider:

- `@RobotConfig.register_subclass("so101_slider_follower")` in `packages/lerobot_robot_so101_slider/src/lerobot_robot_so101_slider/config.py`.
- This is exactly the mechanism we rely on when calling `RobotConfig.get_known_choices()` and `TeleoperatorConfig.get_known_choices()`.

Do all analyzed packages do this?

- Robot plugins: yes, they register with `@RobotConfig.register_subclass(...)`.
- Teleoperator plugins: yes, they register with `@TeleoperatorConfig.register_subclass(...)`.
- Camera plugins: use camera equivalent (e.g., `@CameraConfig.register_subclass(...)`).
- Motor/QA packages: no registry decorator expected (they are not robot/teleop config providers).

Important operational note:

- Even when a package uses the correct decorator, registration still will not happen unless its module is imported.
- This is the key reason external plugins (including LeSlider) are currently not auto-discovered by `physicalai-lerobot-plugin`.

## Common Scenarios

### 1) Add a new follower robot

- Example: `lerobot-robot-yam`, `lerobot-robot-viola`.
- Need: install package, ensure module import runs so LeRobot `register_subclass(...)` executes.

### 2) Add a new leader teleoperator

- Example: `spacemouse`, `hex_arm_leader`, `rebot_arm_102_leader`.
- Need: install package plus hardware dependencies (hid/serial/vendor SDK), and import module before catalog build.

### 3) Bimanual setup

- Example: `lerobot-robot-bimanual-follower`, `lerobot-teleoperator-bimanual-leader`.
- Need: install transitive dependencies (`cello`, `viola`, `violin`, motor SDKs).

### 4) Camera and motor stack extensions

- Example: `lerobot-camera-berxel`, `lerobot-motor-starai`.
- Useful as low-level dependencies for robot plugins; not directly cataloged as robots in current plugin logic.

### 5) Dataset quality checks in CI

- Example: `lerobot-quality-gates`.
- Independent from runtime robot plugin discovery; useful for dataset release workflows.

## Auto-Discovery Options

### Option A: Static external import list (quickest)

- Add a maintained list of known external modules in `physicalai_lerobot_plugin/studio_catalog.py`.
- Import each module before `get_known_choices()` calls.
- Pros: simple, deterministic.
- Cons: manual upkeep.

### Option B: Plugin manifest config

- Add config key/env var listing extra modules, e.g. `PHYSICALAI_LEROBOT_EXTRA_MODULES=lerobot_robot_yam,lerobot_teleoperator_hex_arm`.
- Import at startup if present.
- Pros: no code change per package.
- Cons: operator configuration required.

### Option C: Python entry points (best long-term)

- Define an entry point group for external LeRobot extensions (e.g. `physicalai.lerobot.plugins`).
- Third-party packages expose import targets via entry points.
- Catalog loader resolves all entry points and imports them.
- Pros: scalable ecosystem model.
- Cons: requires package author adoption and a small ecosystem contract.

### Option D: Hybrid fallback

- Use entry points first.
- Also load optional configured module list for legacy plugins.

Recommended approach: Option D.

## Integration implementation

This section describes how `physicalai-lerobot-plugin` currently integrates with LeRobot, and how to extend it to support any external plugin that correctly uses `register_subclass(...)`.

### Current implementation (today)

- `studio_catalog.py` imports config modules under:
  - `lerobot.robots.*config*`
  - `lerobot.teleoperators.*config*`
- After those imports, it reads:
  - `RobotConfig.get_known_choices()`
  - `TeleoperatorConfig.get_known_choices()`
- For each known choice, it builds a typed payload model and a runtime builder for Studio catalog entries.

What this means:

- Built-in LeRobot robot/teleoperator types are discovered correctly.
- External packages outside the `lerobot.*` namespace are not imported automatically, so their decorators do not run, and their types do not appear.

### Target behavior

Goal: support any LeRobot extension that uses:

- `@RobotConfig.register_subclass(...)`
- `@TeleoperatorConfig.register_subclass(...)`
- (optionally) `@CameraConfig.register_subclass(...)`

without hardcoding individual package names.

### Extension design

Load external modules before reading `get_known_choices()`:

1. Import built-in `lerobot.*config*` modules (existing behavior).
2. Import external extension modules from one or more discovery sources.
3. Call `RobotConfig.get_known_choices()` and `TeleoperatorConfig.get_known_choices()`.
4. Build Studio catalog definitions from the resulting registry.

Recommended discovery sources (in order):

1. Entry points (`physicalai.lerobot.plugins`)
   - Best long-term ecosystem contract.
   - External packages publish import targets declaratively.
2. Env/config module list
   - Example: `PHYSICALAI_LEROBOT_EXTRA_MODULES=lerobot_robot_so101_slider,lerobot_teleoperator_so101_with_slider`.
   - Good for local testing and private integrations.
3. Optional prefix scan fallback
   - Attempt import of installed top-level modules matching prefixes such as:
     - `lerobot_robot_`
     - `lerobot_teleoperator_`
     - `lerobot_camera_`
   - Keep this behind a flag because broad import can pull heavy dependencies.

### Why this works for "any" plugin

If a plugin module is importable and its config class has a valid `register_subclass(...)` decorator, LeRobot's registry is updated during import. Once imported, our existing catalog builder path already handles it.

So the scalability problem is not schema generation; it is import discovery.

### Practical implementation notes

- Import errors from optional plugins should be non-fatal by default (warn and continue).
- Add debug logging listing:
  - modules attempted
  - modules imported
  - registrations added (before/after registry key sets)
- Keep a strict mode for CI to fail when expected plugins are missing.
- Add tests with a tiny fixture package that registers a fake robot and fake teleoperator via decorators, then assert both appear in generated catalog definitions.

## Next Steps

1. Implement external module loading in `studio_catalog.py` before robot/teleoperator choice discovery.
2. Start with env/config module list support, then add entry-point discovery.
3. Add an integration test that installs a sample external plugin and verifies its type appears in generated catalog definitions.
4. Document required plugin dependencies per scenario (serial/can/hid/vendor SDK) to reduce setup friction.
5. Optionally rank plugins by maintenance signals (release recency, docs, issue tracker, license clarity) if we plan a recommended list.

## Other (ignore)

- https://pypi.org/project/franky-control/
- https://pypi.org/project/panda-robot/
- https://github.com/jashshah999/lerobot-doctor
