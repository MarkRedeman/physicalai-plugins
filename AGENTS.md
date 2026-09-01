# physicalai-plugins Agent Guide

Third-party robot plugins monorepo for the [PhysicalAI](https://github.com/openvinotoolkit/physicalai) workflow: concrete `Robot` implementations and PhysicalAI Studio catalog plugins for the LeKiwi, reBot B601, bimanual SO-101, LeRobot and MuJoCo SO-101, all driven through the `physicalai` runtime.

## Repository Layout

- `packages/<name>/src/<pkg>/`: per-plugin Python package (src layout).
- `packages/physicalai-common-extras/`: shared action sources / callbacks reused across plugins (git dependency — not published to PyPI).
- `packages/<name>/examples/runtime/*.yaml`: `physicalai run --config` runtime configs.
- `packages/<name>/tests/`: per-plugin pytest tests.
- `scripts/smoke.py`: import + version smoke test for released wheels.
- `.github/`: CI (`lint-test`, `pr-title`, release-please, `publish`) and release config.
- Untracked scratch at the root (`docs/`, `junk/`, `plans/`, `temp-assets/`, `temp-thumbnails/`, `readme.org`, `opencode.jsonc`) — never commit or remove it.

## Setup

- Run `uv sync` from the repo root (installs all packages + dev tooling).
- `physicalai`, `physicalai-studio-plugin` and `physicalai-common-extras` are wired via `[tool.uv.sources]` in the root `pyproject.toml`.

## Build, Test, Lint

- Run tests: `uv run pytest packages/*/tests/`
- Run repo hooks: `uv run prek` (or `uv run prek run --all-files`)
  - `ruff` (select `ALL`, line-length 120, google-style docstrings), `pyrefly` (covers only bimanual + rebot `src`), `markdownlint`, `zizmor`, `gitleaks`, `prettier`.
- Validate a runtime config without hardware: `uv run physicalai run --config <pkg>/examples/runtime/<name>.yaml --print_config`

## Code Conventions

- Robots implement the `physicalai.robot.Robot` protocol (no inheritance); use `@export_config(class_path=...)` on config-constructible classes.
- Action sources / callbacks (`CompositeSource`, `KeyboardTeleop`, `SineWaveSource`, `HoldPoseSource`, `JointLogger`) live in `physicalai-common-extras`, implement the runtime's `ActionSource`/callback seam, and are referenced by `class_path` in the runtime YAMLs.
- Studio catalog registration happens via `[project.entry-points."physicalai.studio.catalog_plugins"]`.
- Versions come from git tags via `hatch-vcs` (per-package `tag-pattern`); never hardcode versions.
- `examples/**` is excluded from the published wheels/sdists — keep examples dev-only.

## Release Process

- `release-please` runs on every push to `main`; PR titles must be [Conventional Commits](https://www.conventionalcommits.org/) (`pr-title.yml` enforces this).
- `release.yml` runs Release Please and builds, smoke-tests, and publishes each released package to PyPI.

## Agent Gotchas

- Package READMEs double as PyPI long descriptions — use absolute `raw.githubusercontent.com` URLs for images, never relative paths.
- Follow the PhysicalAI runtime's own conventions (see the [`physicalai` repo's AGENTS.md](https://github.com/openvinotoolkit/physicalai/blob/main/AGENTS.md)).
