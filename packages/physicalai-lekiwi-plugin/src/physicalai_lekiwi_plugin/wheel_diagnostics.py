"""Inspect the current STS3215 wheel-motor state without changing it."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Protocol

from scservo_sdk import PacketHandler, PortHandler

from physicalai_lekiwi_plugin.constants import LEKIWI_BASE_JOINTS, LEKIWI_MOTOR_IDS, PROTOCOL_VERSION, STS3215Addr

DEFAULT_BAUDRATE = 1_000_000


class _PortHandler(Protocol):
    def openPort(self) -> bool: ...  # noqa: N802

    def closePort(self) -> None: ...  # noqa: N802

    def setBaudRate(self, baudrate: int) -> bool: ...  # noqa: N802

    def setPacketTimeoutMillis(self, timeout_ms: float) -> None: ...  # noqa: N802


class _PacketHandler(Protocol):
    def read1ByteTxRx(self, port: _PortHandler, servo_id: int, address: int) -> tuple[int, int, int]: ...  # noqa: N802

    def read2ByteTxRx(self, port: _PortHandler, servo_id: int, address: int) -> tuple[int, int, int]: ...  # noqa: N802


@dataclass(frozen=True)
class WheelState:
    """Read-only state from one wheel motor."""

    name: str
    servo_id: int
    operating_mode: int
    torque_enabled: int
    torque_limit: int
    goal_velocity: int
    present_velocity: int


def read_wheel_states(*, port: str, baudrate: int = DEFAULT_BAUDRATE) -> list[WheelState]:
    """Read control and feedback registers from all three LeKiwi wheel motors.

    Args:
        port: Serial port connected to the LeKiwi motor bus.
        baudrate: Serial bus baudrate.

    Returns:
        The current state of every configured wheel motor.

    Raises:
        ConnectionError: If the port cannot be opened or a register cannot be read.
    """
    port_handler = PortHandler(port)
    if not port_handler.openPort():
        msg = f"Failed to open serial port {port}"
        raise ConnectionError(msg)

    try:
        if not port_handler.setBaudRate(baudrate):
            msg = f"Failed to set baudrate {baudrate} on {port}"
            raise ConnectionError(msg)
        port_handler.setPacketTimeoutMillis(50.0)
        packet_handler = PacketHandler(PROTOCOL_VERSION)
        return [
            _read_wheel_state(packet_handler, port_handler, name, LEKIWI_MOTOR_IDS[name]) for name in LEKIWI_BASE_JOINTS
        ]
    finally:
        port_handler.closePort()


def _read_wheel_state(
    packet_handler: _PacketHandler,
    port_handler: _PortHandler,
    name: str,
    servo_id: int,
) -> WheelState:
    return WheelState(
        name=name,
        servo_id=servo_id,
        operating_mode=_read1(packet_handler, port_handler, name, servo_id, STS3215Addr.OPERATING_MODE),
        torque_enabled=_read1(packet_handler, port_handler, name, servo_id, STS3215Addr.TORQUE_ENABLE),
        torque_limit=_read2(packet_handler, port_handler, name, servo_id, STS3215Addr.TORQUE_LIMIT),
        goal_velocity=_decode_velocity(
            _read2(packet_handler, port_handler, name, servo_id, STS3215Addr.GOAL_VELOCITY),
        ),
        present_velocity=_decode_velocity(
            _read2(packet_handler, port_handler, name, servo_id, STS3215Addr.PRESENT_VELOCITY),
        ),
    )


def _read1(
    packet_handler: _PacketHandler,
    port_handler: _PortHandler,
    name: str,
    servo_id: int,
    address: STS3215Addr,
) -> int:
    value, comm_result, error = packet_handler.read1ByteTxRx(port_handler, servo_id, address)
    _raise_for_read_error(name, servo_id, address, comm_result, error)
    return int(value)


def _read2(
    packet_handler: _PacketHandler,
    port_handler: _PortHandler,
    name: str,
    servo_id: int,
    address: STS3215Addr,
) -> int:
    value, comm_result, error = packet_handler.read2ByteTxRx(port_handler, servo_id, address)
    _raise_for_read_error(name, servo_id, address, comm_result, error)
    return int(value)


def _raise_for_read_error(
    name: str,
    servo_id: int,
    address: STS3215Addr,
    comm_result: int,
    error: int,
) -> None:
    if comm_result != 0:
        msg = f"Failed to read {address.name} from '{name}' (ID {servo_id}): comm={comm_result}"
        raise ConnectionError(msg)
    if error != 0:
        msg = f"Servo '{name}' (ID {servo_id}) returned an error reading {address.name}: error={error}"
        raise ConnectionError(msg)


def _decode_velocity(value: int) -> int:
    magnitude = value & 0x7FFF
    return -magnitude if value & 0x8000 else magnitude


def main() -> None:
    """Print current wheel motor states without writing to the robot."""
    parser = argparse.ArgumentParser(description="Read the current wheel-motor state from a LeKiwi robot.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="LeKiwi motor-bus serial port")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="Motor-bus baudrate")
    args = parser.parse_args()

    print("wheel                 ID  mode  torque  limit  goal velocity  present velocity")  # noqa: T201
    for state in read_wheel_states(port=args.port, baudrate=args.baudrate):
        print(  # noqa: T201
            f"{state.name:20} {state.servo_id:>2}  {state.operating_mode:>4}  "
            f"{state.torque_enabled:>6}  {state.torque_limit:>5}  {state.goal_velocity:>13}  "
            f"{state.present_velocity:>16}",
        )


if __name__ == "__main__":
    main()
