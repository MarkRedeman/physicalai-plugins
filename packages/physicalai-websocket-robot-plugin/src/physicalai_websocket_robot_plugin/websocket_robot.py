from __future__ import annotations

import json
import time
from dataclasses import dataclass
from threading import Lock
from typing import TYPE_CHECKING

import numpy as np
from loguru import logger
from websockets.sync.client import connect as ws_connect

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation


@dataclass
class WebSocketRobotObservation:
    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        return self.joint_positions


class WebSocketRobot:
    """Robot client that proxies commands to a remote robot via WebSocket.

    Implements the ``physicalai.robot.Robot`` protocol.

    The remote robot server is expected to support these commands:
        - ``features`` — returns available joint names.
        - ``read_state`` — returns current joint positions.
        - ``set_joints_state`` — sends target joint positions.
        - ``enable_torque`` / ``disable_torque`` — torque control.
        - ``ping`` — health check.

    The server may also push ``joints_state_was_updated`` events at any time;
    these are consumed and cached during command wait cycles.
    """

    def __init__(
        self,
        websocket_url: str,
        *,
        connect_timeout: float = 10.0,
        command_timeout: float = 5.0,
    ) -> None:
        self._url = websocket_url
        self._connect_timeout = connect_timeout
        self._command_timeout = command_timeout
        self._ws = None
        self._joint_names: list[str] = []
        self._cached_state: dict | None = None
        self._state_lock = Lock()

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def connect(self) -> None:
        if self.is_connected():
            return

        logger.info("Connecting to WebSocket robot at {}", self._url)
        try:
            self._ws = ws_connect(self._url, timeout=self._connect_timeout)
        except TimeoutError:
            msg = f"WebSocket connection timed out after {self._connect_timeout}s"
            raise ConnectionError(msg) from None

        resp = self._command("features", {}, timeout=self._command_timeout)
        self._joint_names = resp.get("features") or resp.get("payload", {}).get("features", [])
        if not self._joint_names:
            logger.warning("Remote robot returned no features; joint_names will be empty")

        logger.info("Connected to WebSocket robot at {} ({} joints)", self._url, len(self._joint_names))

    def disconnect(self) -> None:
        if self._ws is None:
            return

        logger.info("Disconnecting from WebSocket robot at {}", self._url)
        self._ws.close()
        self._ws = None
        self._cached_state = None

    def is_connected(self) -> bool:
        return self._ws is not None

    def get_observation(self) -> RobotObservation:
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        resp = self._command("read_state", {}, timeout=self._command_timeout)
        state = resp.get("state") or resp.get("payload", {}).get("state", {})
        if not state:
            state = self._cached_state or {}

        positions = np.array([state.get(j, 0.0) for j in self._joint_names], dtype=np.float32)

        return WebSocketRobotObservation(
            joint_positions=positions,
            timestamp=time.monotonic(),
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        joints = {name: float(action[i]) for i, name in enumerate(self._joint_names)}
        self._command("set_joints_state", {"joints": joints, "goal_time": goal_time}, timeout=self._command_timeout)

    def enable_torque(self) -> None:
        self._command("enable_torque", {}, timeout=self._command_timeout)

    def disable_torque(self) -> None:
        self._command("disable_torque", {}, timeout=self._command_timeout)

    def _command(self, event: str, payload: dict, *, timeout: float | None = None) -> dict:
        if self._ws is None:
            msg = "Not connected"
            raise ConnectionError(msg)

        self._ws.send(json.dumps({"event": event, "payload": payload}))

        deadline = time.monotonic() + (timeout or self._command_timeout)

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            try:
                raw = self._ws.recv(timeout=min(remaining, 1.0))
            except TimeoutError:
                continue

            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                logger.warning("Ignoring invalid JSON from remote: {}", raw)
                continue

            event_type = msg.get("event", "")

            if event_type == "joints_state_was_updated":
                with self._state_lock:
                    self._cached_state = msg.get("state")
                continue

            if event_type in {
                "state_read",
                "joints_state_was_set",
                "torque_was_enabled",
                "torque_was_disabled",
                "pong",
                "features_read",
            }:
                return msg

            if event in {"read_state", "features"} and msg.get("state"):
                return msg

            if event == "set_joints_state" and msg.get("event", "").endswith("_was_set"):
                return msg

            if event == "enable_torque" and msg.get("event", "") == "torque_was_enabled":
                return msg

            if event == "disable_torque" and msg.get("event", "") == "torque_was_disabled":
                return msg

            if event == "ping" and msg.get("event", "") == "pong":
                return msg

            with self._state_lock:
                self._cached_state = msg.get("state") or self._cached_state

        msg = f"No response to '{event}' within {timeout or self._command_timeout}s"
        raise TimeoutError(msg)
