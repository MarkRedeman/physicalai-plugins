# Publishing a PhysicalAI Studio Plugin

This guide describes how to publish a new Studio plugin package to PyPI
and configure automated releases. It covers the full lifecycle:

1. **Scaffold** — generate a minimal placeholder package
2. **Manual first upload** — claim the PyPI package name
3. **OIDC trust** — connect PyPI to GitHub Actions
4. **CI automation** — release-please + OIDC-based publishing

> **Why a manual first upload?**
>
> PyPI's trusted publishing (OIDC) requires the package to already exist
> before you can configure trust. The first version must be uploaded
> manually with an API token. After that, automation takes over.

---

## 1. Prerequisites

- A [PyPI account](https://pypi.org/account/register/)
- A [PyPI API token](https://pypi.org/manage/account/token/) (classic or
  per-project — scope it to the new project you are about to create)
- `uv` installed (see [docs.astral.sh/uv](https://docs.astral.sh/uv/))
- This repository cloned

---

## 2. Generate the scaffold

Run the interactive scaffold script from the repo root:

```bash
uv run scripts/scaffold-plugin.py
```

You will be prompted for:

| Prompt           | Example                          | Notes                             |
| ---------------- | -------------------------------- | --------------------------------- |
| Package name     | `physicalai-my-robot`            | Must be valid PyPI name           |
| Description      | `My robot plugin for PhysicalAI` | One-liner                         |
| Author name      | `Jane Doe`                       |                                   |
| Author email     | `jane@example.com`               |                                   |
| Initial version  | `0.0.1`                          | Default is fine for a draft       |
| Min Python       | `3.12`                           | Keep default                      |
| Output directory | `./physicalai-my-robot`          | Default is current dir + pkg name |

The script shows a summary and asks for confirmation before writing.

---

## 3. Build and publish (manual — first release only)

```bash
cd physicalai-my-robot

# Build
uv build

# Upload (requires a PyPI API token)
uv publish --token pypi-xxxxxxxx
```

Your package is now live on PyPI.

---

## 4. Configure OIDC trusted publishing

Once the package exists on PyPI, connect it to this repository so that
GitHub Actions can publish future releases without tokens.

1. Go to `https://pypi.org/manage/project/<your-package>/settings/`
2. Under **"Publishing"** → **"Add a new publisher"**:

   | Field                 | Value                              |
   | --------------------- | ---------------------------------- |
   | **PyPI Project Name** | `physicalai-<your-robot>`          |
   | **Owner**             | `MarkRedeman` (or your GitHub org) |
   | **Repository name**   | `physicalai-rebot-b601-plugin`     |
   | **Workflow name**     | `publish.yml`                      |
   | **Environment**       | `pypi`                             |

3. Click **"Add"**.

After this step, any future GitHub Release for your package will be
automatically published to PyPI via the `publish.yml` workflow.

---

## 5. Add the package to this repo's automation

### 5a. `release-please-config.json`

This file at `.github/release-please-config.json` maps package names to
their directory and tag patterns. Add an entry for your plugin:

```json
{
  "packages": {
    "packages/physicalai-my-robot": {
      "package-name": "physicalai-my-robot",
      "changelog-path": "CHANGELOG.md",
      "release-type": "python"
    }
  }
}
```

### 5b. Update `.github/release-please-manifest.json`

For the initial release, set the version to `0.0.1`:

```json
{
  "physicalai-my-robot": "0.0.1"
}
```

### 5c. Update `RELEASE.md`

Add your package to the versioning table in `RELEASE.md`.

### 5d. Include in uv workspace (optional)

If the plugin lives inside this repo, add it to the workspace in the
root `pyproject.toml`:

```toml
[tool.uv.workspace]
members = ["packages/*"]
```

The existing `packages/*` glob already picks up new directories under
`packages/`, so no change is needed if you place your plugin there.

### 5e. Add `CHANGELOG.md`

Create a minimal changelog for your package:

```markdown
# Changelog

## 0.0.1

- Initial placeholder release
```

---

## 6. From placeholder to full plugin

Replace the generated scaffold files with a real implementation:

1. Follow [`docs/creating-a-studio-plugin.md`](docs/creating-a-studio-plugin.md)
   to implement the `Robot` protocol and `studio_catalog.py`.
2. Add hardware dependencies (e.g. `motorbridge`, `feetech-servo-sdk`) to
   `pyproject.toml`.
3. Register the entry point:
   ```toml
   [project.entry-points."physicalai.studio.catalog_plugins"]
   my-robot = "physicalai_my_plugin.studio_catalog:register_physicalai_studio_plugin"
   ```
4. Switch from static `version` to `hatch-vcs` dynamic versioning:

   ```toml
   [project]
   dynamic = ["version"]

   [tool.hatch.version]
   source = "vcs"
   ```

5. Add URDF assets and `force-include` in the build config.
6. Write tests.

---

## 7. Release workflow

Once everything is wired up:

1. Merge PRs with conventional commit titles.
2. `release-please` creates/updates a release PR.
3. Merge the release PR → tag + GitHub Release created.
4. `publish.yml` fires on `release: published`:
   - Identifies the package from the tag
   - Builds distributions
   - Runs a smoke-test (import + version check)
   - Publishes to PyPI via OIDC

No manual tokens needed after step 3.

---

## 8. Troubleshooting

| Problem                               | Solution                                                                                                                           |
| ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `uv publish` asks for a token         | Pass `--token` with a PyPI API token scoped to the project                                                                         |
| OIDC "403 Forbidden"                  | Verify the publisher is configured for the **exact** workflow path (`.github/workflows/publish.yml`) and environment name (`pypi`) |
| `hatch-vcs` version wrong             | Ensure the matching git tag exists. Tags follow the pattern `physicalai-<name>-v<semver>`                                          |
| Package not found by `release-please` | Check `release-please-config.json` entry — the `package-name` must match the `name` in `pyproject.toml`                            |
