from __future__ import annotations

from unittest.mock import MagicMock

from physicalai_ros2_plugin.studio_catalog import ROS2RobotPayload, _definitions, register_physicalai_studio_plugin


class TestROS2Catalog:
    def test_payload_defaults(self) -> None:
        payload = ROS2RobotPayload(joint_names=["joint_1"])
        assert payload.state_topic == "/joint_states"
        assert payload.angle_unit == "radians"

    def test_definition_has_no_generic_asset(self) -> None:
        definition = _definitions()[0]
        assert definition.type == "ROS2_Generic_Follower"
        assert definition.role == "follower"
        assert definition.asset is None
        assert definition.adapter_options.include_velocities is True

    def test_registration(self) -> None:
        registry = MagicMock()
        register_physicalai_studio_plugin(registry)
        registry.register_robot.assert_called_once()
