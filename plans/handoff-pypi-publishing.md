# PyPI Publishing Strategy

## Overview

The `physicalai-studio` Python package is built from `application/backend/` and published to two registries:

| Registry | Purpose | Trigger | Version scheme |
|----------|---------|---------|----------------|
| **TestPyPI** | Continuous integration | Push to `main` or manual `workflow_dispatch` | `X.Y.Z.dev<TIMESTAMP>` (unique per commit) |
| **PyPI** (production) | Stable releases | Tag `app/vX.Y.Z` | `X.Y.Z` (canonical from `pyproject.toml`) |

---

## Workflows

### `.github/workflows/publish-app-testpypi.yml`

- **Triggers:** Push to `main` branch, or manual via `workflow_dispatch`
- **Publishes to:** https://test.pypi.org/project/physicalai-studio/
- **Version:** `X.Y.Z.dev<UTC-timestamp>` — computed from `application/VERSION` + `date -u +%Y%m%d%H%M%S`
- **Key detail:** The `VERSION_OVERRIDE` env var is set before `build_package.sh` runs. This overrides the version in `pyproject.toml` right before `python -m build` (after `uv sync`, so the lock file is never invalidated). The original version is restored by a `trap EXIT` handler.

### `.github/workflows/publish-app-pypi.yml`

- **Triggers:** Tag push `app/vX.Y.Z`, or manual via `workflow_dispatch` (publish only fires for tag pushes)
- **Publishes to:** https://pypi.org/project/physicalai-studio/
- **Version:** The canonical version from `pyproject.toml` (no `VERSION_OVERRIDE` set)
- **Validation:** Checks that the tag version matches both `application/VERSION` and `application/backend/pyproject.toml` before building

---

## Versioning

### Source of truth

The canonical version lives in two files that must be kept in sync:

- `application/VERSION` — single line, no newline (used by CI for validation and dev version derivation)
- `application/backend/pyproject.toml` — `version = "X.Y.Z"` (used by hatchling when building the wheel)

Both are validated against the git tag (`app/vX.Y.Z`) in the production workflow.

### Dev versions (TestPyPI)

```
0.1.1.dev20260622120000
```

- Suffix: `.dev<14-digit UTC timestamp>` (PEP 440 pre-release)
- Sorts **lower** than the stable release, so `pip install physicalai-studio` will never pull a dev version
- Uniqueness is guaranteed by the timestamp; TestPyPI will never reject a re-upload due to duplicate version

### Production versions (PyPI)

```
0.1.1
```

- Straight semver, read directly from `pyproject.toml`
- Only published via explicit tag push

---

## Build Process

### `application/backend/scripts/build_package.sh`

The build script performs these steps in order:

1. **README transformation** — Runs `_readme_pypi.py` to back up `application/README.md` and rewrite relative URLs (e.g. `./docs/`) to absolute GitHub URLs (e.g. `https://github.com/.../blob/main/application/docs/`). The original is restored on exit via `trap EXIT`.
2. **Dependency sync** — `uv sync --frozen --extra xpu`
3. **OpenAPI spec generation** — `uv run physicalai-studio gen-api`
4. **UI build** — `npm ci && npm run build:api && npm run build` in `application/ui/`
5. **Version override** — If `VERSION_OVERRIDE` is set, patches `pyproject.toml` in-place right before the wheel build (after `uv sync`, so the lock file is never invalidated). The trap handler restores the original version on exit.
6. **Wheel build** — `python -m build --wheel`
7. **twine check** — Validates the wheel metadata

### `application/backend/scripts/hatch_build.py`

A hatchling build hook that:

- Blocks wheel builds if `application/ui/dist/index.html` is missing (prevents publishing a wheel without the frontend)
- Runs the same README transformation as the build script (backup in `initialize()`, restore in `finalize()`)

Hatch hooks are invoked by `python -m build`, not by `uv sync`, so they only fire during actual wheel creation.

### Key files

| File | Role |
|------|------|
| `application/backend/pyproject.toml` | Build config, dependencies, metadata |
| `application/backend/scripts/build_package.sh` | Full build orchestration script |
| `application/backend/scripts/hatch_build.py` | Hatchling build hook (UI guard, README transform) |
| `application/backend/scripts/_readme_pypi.py` | Shared module for README URL transformation + backup/restore |
| `application/VERSION` | Single-source version for CI validation |
| `.github/workflows/publish-app-pypi.yml` | Production PyPI workflow |
| `.github/workflows/publish-app-testpypi.yml` | TestPyPI workflow (continuous + manual) |

---

## How to release a new version

1. Update `application/VERSION` and `application/backend/pyproject.toml` `version` field
2. Commit and merge to `main`
3. Push a tag: `git tag app/vX.Y.Z && git push origin app/vX.Y.Z`
4. The production workflow will validate the tag matches both version files, build, smoke-test, and publish to PyPI

---

## Trusted publishing

Both workflows use [trusted publishing](https://docs.pypi.org/trusted-publishers/) (OIDC `id-token: write`). No API tokens are stored as secrets. The two PyPI projects must have the corresponding GitHub environment/ workflow configured:

- **PyPI:** `https://pypi.org/project/physicalai-studio/` — trusted publisher configured for `.github/workflows/publish-app-pypi.yml`
- **TestPyPI:** `https://test.pypi.org/project/physicalai-studio/` — trusted publisher configured for `.github/workflows/publish-app-testpypi.yml`

---

## Gotchas

- **`uv sync --frozen` and version changes** — The version override in `build_package.sh` is applied *after* `uv sync --frozen`, so the lock file always matches the original `pyproject.toml` during sync. The hatch build hook reads the modified version at build time.
- **Duplicate uploads** — If you manually run the TestPyPI workflow twice within the same second, the timestamp suffix could collide. Practically impossible, but safe to wait a second between runs.
- **`application/README.md` is mutated during build** — The build script backs up and restores it, but if a build is killed with `SIGKILL` (not `SIGTERM`), the backup won't be restored. Run `git checkout application/README.md` to recover.

