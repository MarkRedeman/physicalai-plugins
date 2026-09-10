"""Read persistent STS3215 calibration registers from a LeKiwi."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Protocol

from scservo_sdk import PacketHandler, PortHandler

from physicalai_lekiwi_plugin.calibration import LeKiwiCalibration, LeKiwiJointCalibration
from physicalai_lekiwi_plugin.constants import LEKIWI_MOTOR_IDS, PROTOCOL_VERSION, STS3215Addr

DEFAULT_BAUDRATE = 1_000_000


class _PortHandler(Protocol):
    def openPort(self) -> bool: ...  # noqa: N802

    def closePort(self) -> None: ...  # noqa: N802

    def setBaudRate(self, baudrate: int) -> bool: ...  # noqa: N802

    def setPacketTimeoutMillis(self, timeout_ms: float) -> None: ...  # noqa: N802


class _PacketHandler(Protocol):
    def read1ByteTxRx(self, port: _PortHandler, servo_id: int, address: int) -> tuple[int, int, int]: ...  # noqa: N802

    def read2ByteTxRx(self, port: _PortHandler, servo_id: int, address: int) -> tuple[int, int, int]: ...  # noqa: N802


def read_calibration(*, port: str, baudrate: int = DEFAULT_BAUDRATE) -> LeKiwiCalibration:
    """Read LeKiwi calibration values persisted in each STS3215 motor.

    The STS3215 EEPROM has no register corresponding to LeRobot's ``drive_mode``.
    LeKiwi uses its standard non-inverted value (``0``) for every exported joint.

    Args:
        port: Serial port connected to the LeKiwi motor bus.
        baudrate: Serial bus baudrate.

    Returns:
        The calibration read from the motor EEPROM.

    Raises:
        ConnectionError: If the port cannot be opened or a motor/register cannot be read.
    """
    port_handler = PortHandler(port)
    if not port_handler.openPort():
        msg = f"Failed to open serial port {port}"
        raise ConnectionError(msg)

    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        msg = f"Failed to set baudrate {baudrate} on {port}"
        raise ConnectionError(msg)

    port_handler.setPacketTimeoutMillis(50.0)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    try:
        joints = {
            name: _read_joint_calibration(packet_handler, port_handler, name, servo_id)
            for name, servo_id in LEKIWI_MOTOR_IDS.items()
        }
    finally:
        port_handler.closePort()

    return LeKiwiCalibration(joints=joints)


def _read_joint_calibration(
    packet_handler: _PacketHandler,
    port_handler: _PortHandler,
    name: str,
    expected_id: int,
) -> LeKiwiJointCalibration:
    actual_id = _read_register(packet_handler, port_handler, name, expected_id, STS3215Addr.ID, 1)
    if actual_id != expected_id:
        msg = f"Servo '{name}' responded at ID {expected_id}, but its EEPROM ID is {actual_id}"
        raise ConnectionError(msg)

    range_min = _read_register(
        packet_handler,
        port_handler,
        name,
        expected_id,
        STS3215Addr.MINIMUM_POSITION_LIMIT,
        2,
    )
    range_max = _read_register(
        packet_handler,
        port_handler,
        name,
        expected_id,
        STS3215Addr.MAXIMUM_POSITION_LIMIT,
        2,
    )
    homing_offset_raw = _read_register(
        packet_handler,
        port_handler,
        name,
        expected_id,
        STS3215Addr.HOMING_OFFSET,
        2,
    )

    return LeKiwiJointCalibration(
        id=actual_id,
        drive_mode=0,
        homing_offset=_decode_homing_offset(homing_offset_raw),
        range_min=range_min,
        range_max=range_max,
    )


def _read_register(
    packet_handler: _PacketHandler,
    port_handler: _PortHandler,
    joint_name: str,
    servo_id: int,
    address: STS3215Addr,
    width: int,
) -> int:
    if width == 1:
        value, comm_result, error = packet_handler.read1ByteTxRx(port_handler, servo_id, address)
    else:
        value, comm_result, error = packet_handler.read2ByteTxRx(port_handler, servo_id, address)

    if comm_result != 0:
        msg = f"Failed to read {address.name} from servo '{joint_name}' (ID {servo_id}): comm={comm_result}"
        raise ConnectionError(msg)
    if error != 0:
        msg = f"Servo '{joint_name}' (ID {servo_id}) returned an error reading {address.name}: error={error}"
        raise ConnectionError(msg)
    return int(value)


def _decode_homing_offset(raw_value: int) -> int:
    """Decode the STS3215 12-bit signed-magnitude homing-offset register.

    Returns:
        The signed homing offset in encoder ticks.
    """
    magnitude = raw_value & 0x07FF
    return -magnitude if raw_value & 0x0800 else magnitude


def main() -> None:
    """Export the calibration persisted in a LeKiwi motor bus to JSON."""
    parser = argparse.ArgumentParser(description="Read the STS3215 EEPROM calibration from a LeKiwi robot.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="LeKiwi motor-bus serial port")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="Motor-bus baudrate")
    parser.add_argument("--output", type=Path, default=Path("calibration.json"), help="Output JSON path")
    args = parser.parse_args()

    calibration = read_calibration(port=args.port, baudrate=args.baudrate)
    args.output.write_text(json.dumps(calibration.to_dict(), indent=2) + "\n", encoding="utf-8")
    print(f"Calibration read from {args.port} and written to {args.output}")  # noqa: T201
    print("drive_mode is set to 0 because STS3215 does not persist that LeRobot field.")  # noqa: T201


if __name__ == "__main__":
    main()
