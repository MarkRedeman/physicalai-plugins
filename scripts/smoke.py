"""Verify installed wheels can be imported and report their versions.

Usage:
    python scripts/smoke.py [package-name ...]
    python scripts/smoke.py   # reads all packages from release-please-config.json
"""

import importlib
import importlib.metadata
import json
import pathlib
import sys


def main() -> None:
    pkg_names = _get_package_names()

    for pkg_name in pkg_names:
        print(f"{pkg_name}: ", end="", flush=True)
        try:
            import_name = pkg_name.replace("-", "_")
            importlib.import_module(import_name)
            v = importlib.metadata.version(pkg_name)
            print(f"{v} OK")
        except (ImportError, importlib.metadata.PackageNotFoundError) as exc:
            print(f"FAIL ({exc})")
            sys.exit(1)


def _get_package_names() -> list[str]:
    if len(sys.argv) > 1:
        return sys.argv[1:]

    config_path = pathlib.Path(__file__).parent.parent / ".github" / "release-please-config.json"
    with config_path.open(encoding="utf-8") as f:
        config = json.load(f)
    return [pkg["package-name"] for pkg in config["packages"].values()]


if __name__ == "__main__":
    main()
