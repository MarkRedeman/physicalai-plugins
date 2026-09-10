from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from physicalai_lekiwi_plugin.calibration_reader import read_calibration
from physicalai_lekiwi_plugin.constants import LEKIWI_MOTOR_IDS, STS3215Addr


@pytest.fixture
def mock_sdk() -> tuple[MagicMock, MagicMock]:
    port_handler = MagicMock()
    port_handler.openPort.return_value = True
    port_handler.setBaudRate.return_value = True

    packet_handler = MagicMock()

    def read1_byte(_port: object, servo_id: int, address: STS3215Addr) -> tuple[int, int, int]:
        assert address == STS3215Addr.ID
        return servo_id, 0, 0

    def read2_byte(_port: object, servo_id: int, address: STS3215Addr) -> tuple[int, int, int]:
        if address == STS3215Addr.MINIMUM_POSITION_LIMIT:
            return 100, 0, 0
        if address == STS3215Addr.MAXIMUM_POSITION_LIMIT:
            return 4000, 0, 0
        assert address == STS3215Addr.HOMING_OFFSET
        return 0x0805 if servo_id == 2 else 5, 0, 0

    packet_handler.read1ByteTxRx.side_effect = read1_byte
    packet_handler.read2ByteTxRx.side_effect = read2_byte
    return port_handler, packet_handler


def test_read_calibration_exports_persisted_registers(mock_sdk: tuple[MagicMock, MagicMock]) -> None:
    port_handler, packet_handler = mock_sdk
    with (
        patch("physicalai_lekiwi_plugin.calibration_reader.PortHandler", return_value=port_handler),
        patch("physicalai_lekiwi_plugin.calibration_reader.PacketHandler", return_value=packet_handler),
    ):
        calibration = read_calibration(port="/dev/ttyACM0")

    assert calibration.joints.keys() == LEKIWI_MOTOR_IDS.keys()
    assert calibration.joints["arm_shoulder_lift"].homing_offset == -5
    assert calibration.joints["arm_wrist_roll"].to_dict() == {
        "id": 5,
        "drive_mode": 0,
        "homing_offset": 5,
        "range_min": 100,
        "range_max": 4000,
    }
    port_handler.setBaudRate.assert_called_once_with(1_000_000)
    port_handler.closePort.assert_called_once()
    assert packet_handler.read1ByteTxRx.call_count == 9
    assert packet_handler.read2ByteTxRx.call_count == 27


def test_read_calibration_closes_port_after_read_failure(mock_sdk: tuple[MagicMock, MagicMock]) -> None:
    port_handler, packet_handler = mock_sdk
    packet_handler.read1ByteTxRx.return_value = (0, -1001, 0)
    with (
        patch("physicalai_lekiwi_plugin.calibration_reader.PortHandler", return_value=port_handler),
        pytest.raises(ConnectionError, match="Failed to read ID"),
    ):
        read_calibration(port="/dev/ttyACM0")

    port_handler.closePort.assert_called_once()
