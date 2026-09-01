# ruff: noqa: SLF001

"""Tests for physicalai_common_extras.KeyboardTeleop."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from physicalai.config import to_config

from physicalai_common_extras import KeyboardTeleop


class _FakeObservation:
    """Minimal observation with a joint_positions array."""

    def __init__(self, joint_positions: np.ndarray) -> None:
        self.joint_positions = joint_positions


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
            patch("physicalai_common_extras.keyboard.sys.stdin", fake_stdin),
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
            patch("physicalai_common_extras.keyboard.sys.stdin", fake_stdin),
            patch(
                "physicalai_common_extras.keyboard.termios.tcgetattr",
                return_value=old_settings,
            ) as tcgetattr,
            patch("physicalai_common_extras.keyboard.tty.setcbreak") as setcbreak,
            patch("physicalai_common_extras.keyboard.termios.tcsetattr") as tcsetattr,
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
            patch("physicalai_common_extras.keyboard.sys.stdin", fake_stdin),
            patch("physicalai_common_extras.keyboard.termios.tcgetattr", return_value=object()),
            patch("physicalai_common_extras.keyboard.tty.setcbreak"),
            patch(
                "physicalai_common_extras.keyboard.select.select",
                side_effect=_fake_select,
            ),
            patch("physicalai_common_extras.keyboard.os.read", side_effect=_fake_read),
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
        cfg = to_config(KeyboardTeleop(vx=0.2, vy=0.1, vtheta=0.5))
        assert cfg["class_path"] == "physicalai_common_extras.KeyboardTeleop"
        assert cfg["init_args"]["vx"] == 0.2
