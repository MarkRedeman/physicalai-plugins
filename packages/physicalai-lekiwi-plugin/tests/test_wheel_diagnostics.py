from __future__ import annotations

from unittest.mock import MagicMock, patch

from physicalai_lekiwi_plugin.wheel_diagnostics import read_wheel_states


def test_read_wheel_states_reports_decoded_registers() -> None:
    port_handler = MagicMock()
    port_handler.openPort.return_value = True
    port_handler.setBaudRate.return_value = True
    packet_handler = MagicMock()
    packet_handler.read1ByteTxRx.side_effect = [(1, 0, 0)] * 6
    packet_handler.read2ByteTxRx.side_effect = [(1023, 0, 0), (25, 0, 0), (0x8010, 0, 0)] * 3

    with (
        patch("physicalai_lekiwi_plugin.wheel_diagnostics.PortHandler", return_value=port_handler),
        patch("physicalai_lekiwi_plugin.wheel_diagnostics.PacketHandler", return_value=packet_handler),
    ):
        states = read_wheel_states(port="/dev/ttyACM0")

    assert [(state.name, state.servo_id) for state in states] == [
        ("base_left_wheel", 7),
        ("base_back_wheel", 8),
        ("base_right_wheel", 9),
    ]
    assert states[0].operating_mode == 1
    assert states[0].torque_enabled == 1
    assert states[0].torque_limit == 1023
    assert states[0].goal_velocity == 25
    assert states[0].present_velocity == -16
    port_handler.closePort.assert_called_once()
