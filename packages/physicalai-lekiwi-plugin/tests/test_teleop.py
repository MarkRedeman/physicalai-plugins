# ruff: noqa: SLF001

"""Tests for the teleop action sources in physicalai_lekiwi_plugin.teleop."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from physicalai.config import to_config

from physicalai_lekiwi_plugin.teleop import CompositeTeleop, KeyboardTeleop


class _FakeObservation:
    """Minimal observation with a joint_positions array."""

    def __init__(self, joint_positions: np.ndarray) -> None:
        self.joint_positions = joint_positions


class _FakeLeader:
    """Minimal leader robot for CompositeTeleop tests."""

    def __init__(self, joint_positions: np.ndarray, *, connected: bool = False) -> None:
        self._positions = np.asarray(joint_positions, dtype=np.float32)
        self._connected = connected
        self.connect_calls = 0
        self.disconnect_calls = 0

    def get_observation(self) -> _FakeObservation:
        return _FakeObservation(self._positions)

    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self.connect_calls += 1
        self._connected = True

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self._connected = False


class _FakeBaseSource:
    """Minimal base action source for CompositeTeleop tests."""

    def __init__(self, action: np.ndarray) -> None:
        self._action = np.asarray(action, dtype=np.float32)
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.last_update: tuple[object, object, int] | None = None

    def connect(self, *, bus: object, session_id: str) -> None:
        self.connect_calls += 1

    def update(self, robot_state: object, camera_frames: object, step: int) -> np.ndarray:
        self.last_update = (robot_state, camera_frames, step)
        return self._action

    def disconnect(self) -> None:
        self.disconnect_calls += 1


def _obs(*, arm: float = 1.0) -> _FakeObservation:
    return _FakeObservation(np.full(9, arm, dtype=np.float32))


class TestKeyboardTeleop:
    def test_update_holds_arm_and_applies_base(self) -> None:
        teleop = KeyboardTeleop(vx=0.2, vy=0.1, vtheta=0.5)
        teleop._apply_key("w")
        teleop._apply_key("a")

        action = teleop.update(_obs(), {}, 0)

        np.testing.assert_allclose(action[:6], np.full(6, 1.0))
        np.testing.assert_allclose(action[6:], [0.2, 0.0, 0.5])

    def test_stop_key_zeroes_base(self) -> None:
        teleop = KeyboardTeleop()
        teleop._apply_key("w")
        teleop._apply_key(" ")
        action = teleop.update(_obs(), {}, 0)
        np.testing.assert_allclose(action[6:], [0.0, 0.0, 0.0])

    def test_opposing_keys_override_per_axis(self) -> None:
        teleop = KeyboardTeleop(vx=0.2)
        teleop._apply_key("w")
        teleop._apply_key("s")
        teleop._apply_key("d")
        teleop._apply_key("a")
        action = teleop.update(_obs(), {}, 0)
        np.testing.assert_allclose(action[6:], [-0.2, 0.0, 0.5])

    def test_unknown_keys_ignored(self) -> None:
        teleop = KeyboardTeleop()
        teleop._apply_key("x")
        action = teleop.update(_obs(), {}, 0)
        np.testing.assert_allclose(action[6:], [0.0, 0.0, 0.0])

    def test_update_before_connect_returns_held_arm(self) -> None:
        teleop = KeyboardTeleop()
        action = teleop.update(_obs(arm=3.0), {}, 0)
        np.testing.assert_allclose(action[:6], np.full(6, 3.0))
        np.testing.assert_allclose(action[6:], [0.0, 0.0, 0.0])

    def test_connect_requires_tty(self) -> None:
        teleop = KeyboardTeleop()
        fake_stdin = MagicMock()
        fake_stdin.isatty.return_value = False
        with (
            patch("physicalai_lekiwi_plugin.teleop.keyboard.sys.stdin", fake_stdin),
            pytest.raises(
                RuntimeError,
                match="interactive TTY",
            ),
        ):
            teleop.connect(bus=object(), session_id="test")

    def test_connect_disconnect_restores_terminal(self) -> None:
        teleop = KeyboardTeleop()
        fake_stdin = MagicMock()
        fake_stdin.isatty.return_value = True
        fake_stdin.fileno.return_value = 5
        old_settings = object()
        with (
            patch("physicalai_lekiwi_plugin.teleop.keyboard.sys.stdin", fake_stdin),
            patch(
                "physicalai_lekiwi_plugin.teleop.keyboard.termios.tcgetattr",
                return_value=old_settings,
            ) as tcgetattr,
            patch("physicalai_lekiwi_plugin.teleop.keyboard.tty.setcbreak") as setcbreak,
            patch("physicalai_lekiwi_plugin.teleop.keyboard.termios.tcsetattr") as tcsetattr,
        ):
            teleop.connect(bus=object(), session_id="test")
            teleop.disconnect()

        tcgetattr.assert_called_once_with(5)
        setcbreak.assert_called_once_with(5)
        tcsetattr.assert_called_once()
        assert tcsetattr.call_args[0][2] is old_settings

    def test_drain_keys_applies_pending_input(self) -> None:
        teleop = KeyboardTeleop(vx=0.2)
        fake_stdin = MagicMock()
        fake_stdin.isatty.return_value = True
        fake_stdin.fileno.return_value = 5
        readable = [fake_stdin]
        calls = {"n": 0}

        def _fake_select(*_: object) -> tuple[list[object], list[object], list[object]]:
            calls["n"] += 1
            return (readable if calls["n"] == 1 else [], [], [])

        def _fake_read(_fd: int, _size: int) -> bytes:
            return b"w"

        with (
            patch("physicalai_lekiwi_plugin.teleop.keyboard.sys.stdin", fake_stdin),
            patch("physicalai_lekiwi_plugin.teleop.keyboard.termios.tcgetattr", return_value=object()),
            patch("physicalai_lekiwi_plugin.teleop.keyboard.tty.setcbreak"),
            patch(
                "physicalai_lekiwi_plugin.teleop.keyboard.select.select",
                side_effect=_fake_select,
            ),
            patch("physicalai_lekiwi_plugin.teleop.keyboard.os.read", side_effect=_fake_read),
        ):
            teleop.connect(bus=object(), session_id="test")
            teleop._drain_keys()
            action = teleop.update(_obs(), {}, 0)

        np.testing.assert_allclose(action[6:], [0.2, 0.0, 0.0])

    def test_constructor_rejects_non_positive_speeds(self) -> None:
        with pytest.raises(ValueError, match="positive"):
            KeyboardTeleop(vx=0.0)
        with pytest.raises(ValueError, match="num_base_joints"):
            KeyboardTeleop(num_base_joints=0)

    def test_export_config_roundtrip(self) -> None:
        from physicalai_lekiwi_plugin.teleop import KeyboardTeleop

        cfg = to_config(KeyboardTeleop(vx=0.2, vy=0.1, vtheta=0.5))
        assert cfg["class_path"] == "physicalai_lekiwi_plugin.teleop.KeyboardTeleop"
        assert cfg["init_args"]["vx"] == 0.2


class TestCompositeTeleop:
    def test_update_combines_leader_arm_and_base_source(self) -> None:
        leader = _FakeLeader(np.arange(9, dtype=np.float32))
        base = _FakeBaseSource(np.array([1, 1, 1, 1, 1, 1, 0.3, 0.4, 0.5], dtype=np.float32))
        teleop = CompositeTeleop(leader, base)

        action = teleop.update(_obs(), {"cam": object()}, 7)

        np.testing.assert_allclose(action[:6], np.arange(6, dtype=np.float32))
        np.testing.assert_allclose(action[6:], [0.3, 0.4, 0.5])

    def test_update_forwards_to_base_source(self) -> None:
        leader = _FakeLeader(np.zeros(9, dtype=np.float32))
        base = _FakeBaseSource(np.zeros(9, dtype=np.float32))
        teleop = CompositeTeleop(leader, base)
        obs = _obs()
        frames = {"cam": object()}

        teleop.update(obs, frames, 3)

        assert base.last_update == (obs, frames, 3)

    def test_connect_connects_leader_and_base_source(self) -> None:
        leader = _FakeLeader(np.zeros(9, dtype=np.float32))
        base = _FakeBaseSource(np.zeros(9, dtype=np.float32))
        teleop = CompositeTeleop(leader, base)
        bus = object()

        teleop.connect(bus=bus, session_id="sess")

        assert leader.connect_calls == 1
        assert base.connect_calls == 1
        assert teleop._leader_owned

    def test_connect_skips_already_connected_leader(self) -> None:
        leader = _FakeLeader(np.zeros(9, dtype=np.float32), connected=True)
        base = _FakeBaseSource(np.zeros(9, dtype=np.float32))
        teleop = CompositeTeleop(leader, base)

        teleop.connect(bus=object(), session_id="sess")

        assert leader.connect_calls == 0
        assert not teleop._leader_owned

    def test_disconnect_forwards_and_disconnects_owned_leader(self) -> None:
        leader = _FakeLeader(np.zeros(9, dtype=np.float32))
        base = _FakeBaseSource(np.zeros(9, dtype=np.float32))
        teleop = CompositeTeleop(leader, base)
        teleop.connect(bus=object(), session_id="sess")

        teleop.disconnect()

        assert base.disconnect_calls == 1
        assert leader.disconnect_calls == 1

    def test_disconnect_leaves_unowned_leader(self) -> None:
        leader = _FakeLeader(np.zeros(9, dtype=np.float32), connected=True)
        base = _FakeBaseSource(np.zeros(9, dtype=np.float32))
        teleop = CompositeTeleop(leader, base)

        teleop.disconnect()

        assert leader.disconnect_calls == 0
