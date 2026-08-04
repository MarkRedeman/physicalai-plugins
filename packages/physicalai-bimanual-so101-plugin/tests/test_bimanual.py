from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

import numpy as np
import pytest

from physicalai_bimanual_so101_plugin.constants import (
    BIMANUAL_SO101_JOINT_ORDER,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
)


@dataclass
class _StubObservation:
    joint_positions: np.ndarray
    timestamp: float = 0.0
    sensor_data: dict | None = None
    images: dict | None = None

    @property
    def state(self) -> np.ndarray:
        return self.joint_positions


class _StubSO101:
    """Minimal stub for physicalai.robot.so101.SO101."""

    JOINT_ORDER: ClassVar[tuple[str, ...]] = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )
    NUM_JOINTS = 6

    def __init__(
        self,
        role: str = "follower",
        port: str = "/dev/ttyACM0",
    ) -> None:
        self._role = role
        self._port = port
        self._connected = False

    @property
    def role(self) -> str:
        return self._role

    @property
    def joint_names(self) -> list[str]:
        return list(self.JOINT_ORDER)

    def connect(self) -> None:
        if self._connected:
            return
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_observation(self) -> _StubObservation:
        return _StubObservation(
            joint_positions=np.zeros(self.NUM_JOINTS, dtype=np.float32),
            timestamp=0.0,
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        if action.shape != (self.NUM_JOINTS,):
            msg = f"Expected ({self.NUM_JOINTS},), got {action.shape}"
            raise ValueError(msg)


class TestBimanualSO101Construction:
    def test_construct(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101(role="follower")
        right = _StubSO101(role="follower")
        robot = BimanualSO101(left=left, right=right)
        assert robot.role == "follower"

    def test_role_mismatch_raises(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101(role="follower")
        right = _StubSO101(role="leader")
        with pytest.raises(ValueError, match="same role"):
            BimanualSO101(left=left, right=right)

    def test_joint_names(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101()
        right = _StubSO101()
        robot = BimanualSO101(left=left, right=right)

        names = robot.joint_names
        assert len(names) == 12
        for name in LEFT_ARM_JOINTS:
            assert name in names
        for name in RIGHT_ARM_JOINTS:
            assert name in names

    def test_joint_order_and_count(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101()
        right = _StubSO101()
        robot = BimanualSO101(left=left, right=right)
        assert BIMANUAL_SO101_JOINT_ORDER == robot.JOINT_ORDER
        assert robot.NUM_JOINTS == 12

    def test_device_ids_are_sorted_and_deduplicated(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101(port="/dev/ttyACM1")
        right = _StubSO101(port="/dev/ttyACM0")
        left.device_ids = ("serial:shared", "serial:left")
        right.device_ids = ("serial:right", "serial:shared")

        robot = BimanualSO101(left=left, right=right)
        assert robot.device_ids == ("serial:left", "serial:right", "serial:shared")


class TestBimanualSO101Lifecycle:
    def test_connect_disconnect(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101()
        right = _StubSO101()
        robot = BimanualSO101(left=left, right=right)

        assert not robot.is_connected()
        robot.connect()
        assert robot.is_connected()
        robot.disconnect()
        assert not robot.is_connected()

    def test_connect_right_failure_disconnects_left(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        class _FailingSO101(_StubSO101):
            def __init__(self, *, should_fail: bool = True) -> None:
                super().__init__()
                self._should_fail = should_fail
                self._connected = False

            def connect(self) -> None:
                if self._should_fail:
                    msg = "Failed to connect"
                    raise ConnectionError(msg)
                self._connected = True

        left = _FailingSO101(should_fail=False)
        right = _FailingSO101()
        robot = BimanualSO101(left=left, right=right)
        with pytest.raises(ConnectionError):
            robot.connect()
        assert not left.is_connected()
        assert not robot.is_connected()

    def test_idempotent_connect(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101()
        right = _StubSO101()
        robot = BimanualSO101(left=left, right=right)
        robot.connect()
        robot.connect()
        assert robot.is_connected()

    def test_idempotent_disconnect(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101()
        right = _StubSO101()
        robot = BimanualSO101(left=left, right=right)
        robot.disconnect()
        assert not robot.is_connected()

    def test_disconnect_attempts_both_arms_when_left_fails(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        class _FailingDisconnectSO101(_StubSO101):
            def __init__(self, *, should_fail: bool) -> None:
                super().__init__()
                self.should_fail = should_fail
                self.disconnect_calls = 0

            def disconnect(self) -> None:
                self.disconnect_calls += 1
                if self.should_fail:
                    msg = "disconnect failure"
                    raise RuntimeError(msg)
                super().disconnect()

        left = _FailingDisconnectSO101(should_fail=True)
        right = _FailingDisconnectSO101(should_fail=False)
        robot = BimanualSO101(left=left, right=right)

        with pytest.raises(RuntimeError, match="disconnect failure"):
            robot.disconnect()

        assert left.disconnect_calls == 1
        assert right.disconnect_calls == 1


class TestBimanualSO101Observation:
    def test_get_observation_shape(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101()
        right = _StubSO101()
        robot = BimanualSO101(left=left, right=right)
        robot.connect()

        obs = robot.get_observation()
        assert obs.joint_positions.shape == (12,)

    def test_get_observation_concatenates_left_right(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        class _PosStubSO101(_StubSO101):
            def get_observation(self) -> _StubObservation:
                positions = np.full(self.NUM_JOINTS, 42.0, dtype=np.float32)
                return _StubObservation(joint_positions=positions, timestamp=1.0)

        left = _PosStubSO101()
        right = _PosStubSO101()
        robot = BimanualSO101(left=left, right=right)
        robot.connect()

        obs = robot.get_observation()
        np.testing.assert_array_equal(obs.joint_positions[:6], 42.0)
        np.testing.assert_array_equal(obs.joint_positions[6:], 42.0)

    def test_state_property(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101Observation

        obs = BimanualSO101Observation(
            joint_positions=np.zeros(12, dtype=np.float32),
            timestamp=0.0,
        )
        np.testing.assert_array_equal(obs.state, obs.joint_positions)

    def test_sensor_data_merges_only_shared_keys(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        class _SensorStubSO101(_StubSO101):
            def __init__(self, sensor_data: dict[str, np.ndarray]) -> None:
                super().__init__()
                self._sensor_data = sensor_data

            def get_observation(self) -> _StubObservation:
                return _StubObservation(
                    joint_positions=np.zeros(self.NUM_JOINTS, dtype=np.float32),
                    timestamp=1.0,
                    sensor_data=self._sensor_data,
                )

        left = _SensorStubSO101(
            {
                "effort": np.ones(6, dtype=np.float32),
                "temperature": np.full(6, 35.0, dtype=np.float32),
            },
        )
        right = _SensorStubSO101(
            {
                "effort": np.full(6, 2.0, dtype=np.float32),
            },
        )
        robot = BimanualSO101(left=left, right=right)
        obs = robot.get_observation()

        assert obs.sensor_data is not None
        assert set(obs.sensor_data) == {"effort"}
        np.testing.assert_array_equal(
            obs.sensor_data["effort"],
            np.concatenate([
                np.ones(6, dtype=np.float32),
                np.full(6, 2.0, dtype=np.float32),
            ]),
        )


class TestBimanualSO101Action:
    def test_send_action(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        class _TrackingStubSO101(_StubSO101):
            def __init__(self) -> None:
                super().__init__()
                self.last_action: np.ndarray | None = None

            def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
                self.last_action = action.copy()
                super().send_action(action, goal_time=goal_time)

        left = _TrackingStubSO101()
        right = _TrackingStubSO101()
        robot = BimanualSO101(left=left, right=right)
        robot.connect()

        action = np.arange(12, dtype=np.float32)
        robot.send_action(action)

        np.testing.assert_array_equal(left.last_action, action[:6])
        np.testing.assert_array_equal(right.last_action, action[6:])

    def test_wrong_shape_raises(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101()
        right = _StubSO101()
        robot = BimanualSO101(left=left, right=right)
        robot.connect()

        with pytest.raises(ValueError, match="Expected action shape"):
            robot.send_action(np.zeros(6, dtype=np.float32))

    def test_leader_send_action_raises(self) -> None:
        from physicalai_bimanual_so101_plugin.bimanual import BimanualSO101

        left = _StubSO101(role="leader")
        right = _StubSO101(role="leader")
        robot = BimanualSO101(left=left, right=right)
        robot.connect()

        with pytest.raises(RuntimeError, match="Cannot send actions to a leader"):
            robot.send_action(np.zeros(12, dtype=np.float32))
