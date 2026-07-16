from __future__ import annotations

from pathlib import Path

import pytest


class _FakeRegistry:
    def __init__(self) -> None:
        self.definitions: list = []

    def register(self, definition: object) -> None:
        self.definitions.append(definition)

    def register_many(self, definitions: list[object]) -> None:
        self.definitions.extend(definitions)


class _StubFactory:
    def __init__(self, port: str | None = "/dev/ttyACM0") -> None:
        self._port = port

    async def find_port_by_serial(self, serial_number: str) -> str | None:
        return self._port

    async def get_calibration_by_id(self, calibration_id: str) -> object | None:
        return None


class _StubRobot:
    def __init__(self, payload: object) -> None:
        self.payload = payload


class TestRegistration:
    def test_register_plugin(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            register_physicalai_studio_plugin,
        )

        registry = _FakeRegistry()
        register_physicalai_studio_plugin(registry)
        assert len(registry.definitions) == 2

    def test_definitions_have_expected_types(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import _definitions

        types = {d.type for d in _definitions()}
        assert types == {"BimanualSO101_Follower", "BimanualSO101_Leader"}

    def test_follower_structure(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import BimanualSO101Payload
        from physicalai_bimanual_so101_plugin.studio_catalog import _definitions

        follower = next(d for d in _definitions() if d.type == "BimanualSO101_Follower")

        assert follower.display_name == "Bimanual SO-101 Follower"
        assert follower.role == "follower"
        assert follower.asset is not None
        assert follower.asset.urdf_relative_path == Path("so101_dual/so101_dual.urdf")
        assert follower.asset.root_resolver is not None
        assert callable(follower.robot_builder)
        assert follower.robot_payload is BimanualSO101Payload
        assert follower.probe is not None
        assert follower.asset.packages == {"so101_dual": Path("so101_dual")}
        assert len(follower.asset.joint_map) == 12

    def test_leader_structure(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import _definitions
        from physicalai_bimanual_so101_plugin.studio_catalog import BimanualSO101Payload

        leader = next(d for d in _definitions() if d.type == "BimanualSO101_Leader")

        assert leader.display_name == "Bimanual SO-101 Leader"
        assert leader.role == "leader"
        assert leader.asset is not None
        assert leader.asset.urdf_relative_path == Path("so101_dual/so101_dual.urdf")
        assert leader.asset.root_resolver is not None
        assert callable(leader.robot_builder)
        assert leader.robot_payload is BimanualSO101Payload


class TestPayload:
    def test_payload_defaults(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
        )

        payload = BimanualSO101Payload(
            left_serial_number="SN-LEFT-001",
            right_serial_number="SN-RIGHT-001",
        )
        assert payload.left_serial_number == "SN-LEFT-001"
        assert payload.right_serial_number == "SN-RIGHT-001"
        assert payload.baudrate == 1_000_000
        assert payload.role == "follower"
        assert payload.disable_torque_on_disconnect is True
        assert payload.left_calibration_id is None
        assert payload.right_calibration_id is None

    def test_payload_requires_both_serials(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
        )

        with pytest.raises(Exception):
            BimanualSO101Payload()

    def test_payload_with_calibration_ids(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
        )

        payload = BimanualSO101Payload(
            left_serial_number="SN-L",
            right_serial_number="SN-R",
            left_calibration_id="cal-left-uuid",
            right_calibration_id="cal-right-uuid",
        )
        assert payload.left_calibration_id == "cal-left-uuid"
        assert payload.right_calibration_id == "cal-right-uuid"


class TestURDF:
    def test_urdf_path_exists(self) -> None:
        from physicalai_bimanual_so101_plugin import get_urdf_path

        path = get_urdf_path()
        assert path.exists()
        urdf_file = path / "so101_dual" / "so101_dual.urdf"
        assert urdf_file.exists(), f"URDF not found at {urdf_file}"
        assert urdf_file.stat().st_size > 0

    def test_asset_root_resolver(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            _get_bimanual_urdf_root,
        )

        root = _get_bimanual_urdf_root()
        assert isinstance(root, Path)
        assert root.exists()


class TestBuilder:
    @pytest.mark.anyio
    async def test_build_uncalibrated(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
            _build_bimanual_driver,
        )

        payload = BimanualSO101Payload(
            left_serial_number="SN-L",
            right_serial_number="SN-R",
        )
        robot = _StubRobot(payload)
        factory = _StubFactory(port="/dev/ttyACM0")
        driver = await _build_bimanual_driver(robot, factory)
        assert driver is not None
        assert driver.is_connected() is False

    @pytest.mark.anyio
    async def test_build_from_dict(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            _build_bimanual_driver,
        )

        payload = {
            "left_serial_number": "SN-L",
            "right_serial_number": "SN-R",
            "role": "follower",
        }
        robot = _StubRobot(payload)
        factory = _StubFactory(port="/dev/ttyACM1")
        driver = await _build_bimanual_driver(robot, factory)
        assert driver is not None

    @pytest.mark.anyio
    async def test_build_left_port_not_found(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
            _build_bimanual_driver,
        )

        payload = BimanualSO101Payload(
            left_serial_number="SN-MISSING",
            right_serial_number="SN-R",
        )
        robot = _StubRobot(payload)
        factory = _StubFactory(port=None)
        with pytest.raises(RuntimeError, match="Left arm not found"):
            await _build_bimanual_driver(robot, factory)

    @pytest.mark.anyio
    async def test_build_right_port_not_found(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
            _build_bimanual_driver,
        )

        class _SelectiveFactory:
            async def find_port_by_serial(self, serial_number: str) -> str | None:
                return "/dev/ttyACM0" if serial_number == "SN-L" else None

            async def find_so101_port(self, robot: object) -> str:
                return "/dev/ttyACM0"

            async def get_calibration_by_id(self, calibration_id: str) -> object | None:
                return None

        payload = BimanualSO101Payload(
            left_serial_number="SN-L",
            right_serial_number="SN-MISSING",
        )
        robot = _StubRobot(payload)
        factory = _SelectiveFactory()
        with pytest.raises(RuntimeError, match="Right arm not found"):
            await _build_bimanual_driver(robot, factory)

    @pytest.mark.anyio
    async def test_build_mixed_calibration_raises(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
            _build_bimanual_driver,
        )

        payload = BimanualSO101Payload(
            left_serial_number="SN-L",
            right_serial_number="SN-R",
            left_calibration_id="cal-left",
            right_calibration_id=None,
        )
        robot = _StubRobot(payload)
        factory = _StubFactory()
        with pytest.raises(ValueError, match="Both arms must have calibration"):
            await _build_bimanual_driver(robot, factory)

    @pytest.mark.anyio
    async def test_build_calibration_not_found(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Payload,
            _build_bimanual_driver,
        )

        payload = BimanualSO101Payload(
            left_serial_number="SN-L",
            right_serial_number="SN-R",
            left_calibration_id="cal-left",
            right_calibration_id="cal-right",
        )
        robot = _StubRobot(payload)
        factory = _StubFactory()
        with pytest.raises(RuntimeError, match="Calibration not found"):
            await _build_bimanual_driver(robot, factory)


class TestProbe:
    @pytest.mark.anyio
    async def test_is_online_no_manager(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            BimanualSO101Probe,
        )

        probe = BimanualSO101Probe()
        result = await probe.is_online(
            {
                "left_serial_number": "SN-L",
                "right_serial_number": "SN-R",
            }
        )
        assert result is False

    def test_joint_map(self) -> None:
        from physicalai_bimanual_so101_plugin.studio_catalog import (
            _BIMANUAL_SO101_TO_URDF,
        )

        assert len(_BIMANUAL_SO101_TO_URDF) == 12
        assert "left_shoulder_pan.pos" in _BIMANUAL_SO101_TO_URDF
        assert "right_shoulder_pan.pos" in _BIMANUAL_SO101_TO_URDF
        assert _BIMANUAL_SO101_TO_URDF["left_gripper.pos"] == ["left_gripper"]
        assert _BIMANUAL_SO101_TO_URDF["right_gripper.pos"] == ["right_gripper"]
