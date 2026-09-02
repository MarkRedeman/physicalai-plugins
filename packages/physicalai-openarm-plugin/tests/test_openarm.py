from __future__ import annotations

import numpy as np
import pytest

from physicalai_openarm_plugin.bimanual import BimanualOpenArmFollower, BimanualOpenArmLeader
from physicalai_openarm_plugin.constants import OPENARM_JOINT_ORDER
from physicalai_openarm_plugin.damiao import MotorState
from physicalai_openarm_plugin.openarm import OpenArmFollower, OpenArmLeader


class FakeTransport:
    def __init__(self) -> None:
        self.is_connected = False
        self.disable_calls = 0
        self.commands: dict[str, tuple[float, float, float]] = {}
        self.states = {
            name: MotorState(float(index), 1.0, 2.0, 30.0, 31.0) for index, name in enumerate(OPENARM_JOINT_ORDER)
        }

    def connect(self) -> None:
        self.is_connected = True

    def disconnect(self, *, disable_torque: bool) -> None:
        self.is_connected = False
        self.disable_calls += int(disable_torque)

    def disable_torque(self) -> None:
        self.disable_calls += 1

    def read_states(self) -> dict[str, MotorState]:
        return self.states

    def send_positions(self, commands: dict[str, tuple[float, float, float]]) -> None:
        self.commands = commands


def test_follower_uses_fixed_order_and_clips_right_limits() -> None:
    transport = FakeTransport()
    follower = OpenArmFollower("can0", side="right", _transport=transport)
    follower.connect()

    observation = follower.get_observation()
    follower.send_action(np.array([100.0, -20.0, 90.0, 200.0, 90.0, 50.0, -90.0, 1.0], dtype=np.float32))

    assert follower.joint_names == list(OPENARM_JOINT_ORDER)
    assert observation.joint_positions.tolist() == list(range(8))
    assert transport.commands["joint_1"][2] == 75.0
    assert transport.commands["joint_2"][2] == -9.0
    assert transport.commands["joint_4"][2] == 135.0
    assert transport.commands["gripper"][2] == 0.0


def test_follower_relative_target_and_action_validation() -> None:
    transport = FakeTransport()
    follower = OpenArmFollower("can0", side="left", max_relative_target=2.0, _transport=transport)
    follower.connect()
    follower.send_action(np.full(8, 50.0, dtype=np.float32))
    assert transport.commands["joint_1"][2] == 2.0
    assert transport.commands["joint_4"][2] == 5.0
    with pytest.raises(ValueError, match="Expected action"):
        follower.send_action(np.zeros(7, dtype=np.float32))


def test_leader_ignores_actions_and_disables_torque() -> None:
    transport = FakeTransport()
    leader = OpenArmLeader("can1", _transport=transport)
    leader.connect()
    assert transport.disable_calls == 1
    leader.send_action(np.zeros(8, dtype=np.float32))
    leader.disconnect()
    assert transport.disable_calls == 2


def test_bimanual_follower_splits_actions_and_rejects_shared_bus() -> None:
    left_transport, right_transport = FakeTransport(), FakeTransport()
    left = OpenArmFollower("can0", side="left", _transport=left_transport)
    right = OpenArmFollower("can1", side="right", _transport=right_transport)
    robot = BimanualOpenArmFollower(left, right)
    robot.connect()
    robot.send_action(np.arange(16, dtype=np.float32))
    assert robot.joint_names[0] == "left_joint_1"
    assert robot.joint_names[8] == "right_joint_1"
    assert left_transport.commands["joint_1"][2] == 0.0
    assert right_transport.commands["joint_1"][2] == 8.0
    with pytest.raises(ValueError, match="distinct"):
        BimanualOpenArmLeader(OpenArmLeader("can2"), OpenArmLeader("can2"))
