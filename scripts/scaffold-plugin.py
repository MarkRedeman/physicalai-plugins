#!/usr/bin/env python3
"""Interactive scaffold generator for a new PhysicalAI Studio plugin.

Generates a minimal uploadable package from the template in
``scaffold/minimal-plugin/``, then the user can publish it to PyPI
as a "work in progress" to claim the package name.

Usage::

    uv run scripts/scaffold-plugin.py

After generation, build and publish::

    cd <output-dir>
    uv build
    uv publish

See PUBLISHING.md at the repo root for the full guide.
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TEMPLATE_DIR = REPO_ROOT / "scaffold" / "minimal-plugin"

TOKENS = {
    "{{ package_name }}": "",
    "{{ import_name }}": "",
    "{{ organization }}": "",
    "{{ repository_name }}": "",
    "{{ description }}": "",
    "{{ author_name }}": "",
    "{{ author_email }}": "",
    "{{ version }}": "0.0.1",
    "{{ python_min_version }}": "3.12",
}


def _validate_package_name(name: str) -> str | None:
    if not re.match(r"^[a-z][a-z0-9_.-]*$", name):
        return "Must start with a letter and contain only lowercase letters, digits, hyphens, underscores, or dots."
    return None


def _prompt(
    label: str,
    *,
    default: str = "",
    validate: callable | None = None,
) -> str:
    while True:
        prompt_text = f"{label} [{default}]: " if default else f"{label}: "
        raw = input(prompt_text)
        value = raw.strip() if raw else default
        if not value:
            print("  Value cannot be empty.")
            continue
        if validate:
            err = validate(value)
            if err:
                print(f"  {err}")
                continue
        return value


def _collect_input() -> dict[str, str]:
    print("\n== PhysicalAI Studio Plugin Scaffold ==\n")
    print("Enter the details for your new plugin. Press Enter to accept defaults.\n")

    pkg = _prompt("Package name (e.g. physicalai-my-robot)", validate=_validate_package_name)
    org = _prompt("GitHub organization", default="your-org")
    repo = _prompt("GitHub repository name", default=pkg)
    imp = pkg.replace("-", "_")
    desc_default = f"{pkg} — PhysicalAI Studio plugin (work in progress)"
    desc = _prompt("Description", default=desc_default)
    author = _prompt("Author name")
    email = _prompt("Author email")
    version = _prompt("Initial version", default="0.0.1")
    py_ver = _prompt("Minimum Python version", default="3.12")

    return {
        "{{ package_name }}": pkg,
        "{{ import_name }}": imp,
        "{{ organization }}": org,
        "{{ repository_name }}": repo,
        "{{ description }}": desc,
        "{{ author_name }}": author,
        "{{ author_email }}": email,
        "{{ version }}": version,
        "{{ python_min_version }}": py_ver,
    }


def _show_summary(values: dict[str, str], target: Path) -> None:
    print("\n── Summary ──────────────────────────────")
    print(f"  Package name:     {values['{{ package_name }}']}")
    print(f"  Import name:      {values['{{ import_name }}']}")
    print(f"  GitHub org:       {values['{{ organization }}']}")
    print(f"  GitHub repo:      {values['{{ repository_name }}']}")
    print(f"  Description:      {values['{{ description }}']}")
    print(f"  Author:           {values['{{ author_name }}']} <{values['{{ author_email }}']}>")
    print(f"  Version:          {values['{{ version }}']}")
    print(f"  Python min:       {values['{{ python_min_version }}']}")
    print(f"  Output directory: {target}")
    print("──────────────────────────────────────────\n")


def _copy_and_render(src: Path, dst: Path, values: dict[str, str]) -> None:
    """Copy template directory to dst, replacing tokens in filenames and content."""
    for item in src.rglob("*"):
        rel = item.relative_to(src)
        # Replace tokens in the relative path segments
        parts = []
        for raw_part in rel.parts:
            resolved = raw_part
            for token, value in values.items():
                resolved = resolved.replace(token, value)
            parts.append(resolved)
        target_rel = Path(*parts)
        target_path = dst / target_rel

        if item.is_dir():
            target_path.mkdir(parents=True, exist_ok=True)
        else:
            target_path.parent.mkdir(parents=True, exist_ok=True)
            content = item.read_text(encoding="utf-8")
            for token, value in values.items():
                content = content.replace(token, value)
            target_path.write_text(content, encoding="utf-8")

    print(f"  ✓ Generated {len(list(dst.rglob('*')))} files")


def main() -> None:
    if not TEMPLATE_DIR.is_dir():
        print(f"Error: template directory not found at {TEMPLATE_DIR}", file=sys.stderr)
        print("Run this script from the repo root: uv run scripts/scaffold-plugin.py", file=sys.stderr)
        sys.exit(1)

    values = _collect_input()
    default_dir = Path.cwd() / values["{{ package_name }}"]
    out_dir = Path(_prompt("Output directory", default=str(default_dir)))

    _show_summary(values, out_dir)

    confirm = input("Generate scaffold? [Y/n]: ").strip().lower()
    if confirm not in {"", "y", "yes"}:
        print("Aborted.")
        sys.exit(0)

    if out_dir.exists():
        overwrite = input(f"  Directory {out_dir} already exists. Overwrite? [y/N]: ").strip().lower()
        if overwrite not in {"y", "yes"}:
            print("Aborted.")
            sys.exit(0)
        shutil.rmtree(out_dir)

    _copy_and_render(TEMPLATE_DIR, out_dir, values)
    print(f"\nDone! Your scaffold is ready at: {out_dir}\n")
    print("Next steps:")
    print(f"  cd {out_dir}")
    print("  uv build")
    print("  uv publish")
    print("\nThen follow PUBLISHING.md to configure OIDC + CI.")


if __name__ == "__main__":
    main()
