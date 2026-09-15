# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import warnings

ASSETS_ROOT = Path(__file__).resolve().parent


def get_urdf_path(robot: str, *parts: str) -> Path:
    """
    get_urdf_path("openarm_v2.0", "example", "v2.urdf")
    get_urdf_path("openarm_v1.0", "openarm_v10.urdf.xacro")
    """
    path = ASSETS_ROOT / "robot" / robot / "urdf" / Path(*parts)
    if not path.exists():
        warnings.warn(f"URDF not found at: {path}", stacklevel=2)
    return path


def get_config_path(robot: str, *parts: str) -> Path:
    path = ASSETS_ROOT / "robot" / robot / "config" / Path(*parts)
    if not path.exists():
        warnings.warn(f"Config not found at: {path}", stacklevel=2)
    return path


__all__ = ["get_urdf_path", "get_config_path", "ASSETS_ROOT"]
