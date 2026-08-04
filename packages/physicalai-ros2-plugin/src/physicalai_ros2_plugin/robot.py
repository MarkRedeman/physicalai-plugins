"""ROS 2 JointState and JointTrajectory adapter."""

from __future__ import annotations

import importlib
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, Mapping, cast

import numpy as np
from physicalai.config import export_config

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation

AngleUnit = Literal["radians", "degrees"]
_JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
_JOINT_TRAJECTORY_TYPE = "trajectory_msgs/msg/JointTrajectory"


@dataclass
class ROS2Observation:
    """PhysicalAI observation assembled from ROS 2 topics."""

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """Return joint positions as the primary state."""
        return self.joint_positions


@export_config
class ROS2Robot:
    """Expose a ROS 2 robot using JointState and JointTrajectory topics.

    The adapter subscribes to ``sensor_msgs/msg/JointState`` and publishes
    ``trajectory_msgs/msg/JointTrajectory``. Positions sent to and returned from
    PhysicalAI use ``angle_unit``; ROS messages always use radians.
    """

    def __init__(
        self,
        joint_names: list[str],
        *,
        state_topic: str = "/joint_states",
        command_topic: str = "/joint_trajectory_controller/joint_trajectory",
        node_name: str = "physicalai_ros2_robot",
        namespace: str = "",
        state_message_type: str = _JOINT_STATE_TYPE,
        command_message_type: str = _JOINT_TRAJECTORY_TYPE,
        command_timeout: float = 1.0,
        connect_timeout: float = 10.0,
        goal_time: float = 0.1,
        angle_unit: AngleUnit = "radians",
        sensor_topics: Mapping[str, str] | None = None,
        camera_topics: Mapping[str, str] | None = None,
    ) -> None:
        """Configure a ROS 2 robot without importing ROS 2 until ``connect``."""
        if not joint_names or len(set(joint_names)) != len(joint_names):
            msg = "joint_names must be a non-empty list of unique names."
            raise ValueError(msg)
        if state_message_type != _JOINT_STATE_TYPE or command_message_type != _JOINT_TRAJECTORY_TYPE:
            msg = "Only sensor_msgs/msg/JointState and trajectory_msgs/msg/JointTrajectory are supported."
            raise ValueError(msg)
        if command_timeout <= 0 or connect_timeout <= 0 or goal_time <= 0:
            msg = "command_timeout, connect_timeout, and goal_time must be positive."
            raise ValueError(msg)
        if angle_unit not in {"radians", "degrees"}:
            msg = "angle_unit must be 'radians' or 'degrees'."
            raise ValueError(msg)

        self.joint_names = list(joint_names)
        self._state_topic = state_topic
        self._command_topic = command_topic
        self._node_name = node_name
        self._namespace = namespace
        self._state_message_type = state_message_type
        self._command_message_type = command_message_type
        self._command_timeout = command_timeout
        self._connect_timeout = connect_timeout
        self._goal_time = goal_time
        self._angle_unit = angle_unit
        self._sensor_topics = dict(sensor_topics or {})
        self._camera_topics = dict(camera_topics or {})
        self._state_lock = threading.Lock()
        self._state_ready = threading.Event()
        self._positions: np.ndarray | None = None
        self._velocities: np.ndarray | None = None
        self._effort: np.ndarray | None = None
        self._state_timestamp = 0.0
        self._sensor_data: dict[str, np.ndarray] = {}
        self._images: dict[str, Frame] = {}
        self._rclpy: Any = None
        self._node: Any = None
        self._executor: Any = None
        self._publisher: Any = None
        self._spin_thread: threading.Thread | None = None
        self._stop_spin = threading.Event()
        self._owns_ros_context = False

    @property
    def device_ids(self) -> tuple[str, ...]:
        """ROS topics are shared graph resources, not owned devices."""
        return ()

    def connect(self) -> None:
        """Create ROS resources and wait for the first complete JointState."""
        if self.is_connected():
            return
        try:
            self._load_ros()
            if not self._rclpy.ok():
                self._rclpy.init()
                self._owns_ros_context = True
            self._node = self._rclpy.create_node(self._node_name, namespace=self._namespace)
            types = self._message_types()
            self._publisher = self._node.create_publisher(types["trajectory"], self._command_topic, 10)
            self._node.create_subscription(types["joint_state"], self._state_topic, self._on_joint_state, 10)
            for name, topic in self._sensor_topics.items():
                self._node.create_subscription(
                    types["float_array"], topic, lambda message, key=name: self._on_sensor(key, message), 10
                )
            for name, topic in self._camera_topics.items():
                self._node.create_subscription(
                    types["image"],
                    topic,
                    lambda message, key=name: self._on_image(key, message),
                    10,
                )
            executor_module = importlib.import_module("rclpy.executors")
            self._executor = executor_module.SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._stop_spin.clear()
            self._spin_thread = threading.Thread(target=self._spin, name=self._node_name, daemon=True)
            self._spin_thread.start()
            if not self._state_ready.wait(self._connect_timeout):
                msg = f"Timed out waiting for complete JointState on {self._state_topic!r}."
                raise TimeoutError(msg)
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        """Destroy ROS entities without shutting down a caller-owned context."""
        self._stop_spin.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        self._spin_thread = None
        if self._executor is not None:
            self._executor.shutdown()
        self._executor = None
        if self._node is not None:
            self._node.destroy_node()
        self._node = None
        self._publisher = None
        self._state_ready.clear()
        if self._owns_ros_context and self._rclpy is not None:
            self._rclpy.shutdown()
        self._owns_ros_context = False

    def is_connected(self) -> bool:
        """Return whether the ROS node and command publisher are available."""
        return self._node is not None and self._publisher is not None

    def get_observation(self) -> RobotObservation:
        """Return the latest complete JointState and topic extension data."""
        with self._state_lock:
            if self._positions is None:
                msg = "No complete JointState has been received. Call connect() first."
                raise ConnectionError(msg)
            if time.monotonic() - self._state_timestamp > self._command_timeout:
                msg = f"JointState on {self._state_topic!r} is stale."
                raise ConnectionError(msg)
            sensor_data = dict(self._sensor_data)
            if self._velocities is not None:
                sensor_data["velocities"] = self._velocities.copy()
            if self._effort is not None:
                sensor_data["effort"] = self._effort.copy()
            return ROS2Observation(
                joint_positions=self._positions.copy(),
                timestamp=self._state_timestamp,
                sensor_data=sensor_data or None,
                images=dict(self._images) or None,
            )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Publish an ordered JointTrajectory point after validating fresh state."""
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)
        if action.shape != (len(self.joint_names),) or not np.isfinite(action).all():
            msg = f"Action must contain {len(self.joint_names)} finite joint positions."
            raise ValueError(msg)
        self.get_observation()
        duration = self._goal_time if goal_time == 0.1 else goal_time
        if duration <= 0:
            msg = "goal_time must be positive."
            raise ValueError(msg)
        types = self._message_types()
        message = types["trajectory"]()
        message.joint_names = self.joint_names
        point = types["trajectory_point"]()
        positions = np.asarray(action, dtype=np.float64)
        if self._angle_unit == "degrees":
            positions = np.deg2rad(positions)
        point.positions = positions.tolist()
        seconds, nanoseconds = divmod(round(duration * 1_000_000_000), 1_000_000_000)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int(nanoseconds)
        message.points = [point]
        self._publisher.publish(message)

    def _load_ros(self) -> None:
        try:
            self._rclpy = importlib.import_module("rclpy")
        except ImportError as exc:
            msg = "ROS 2 is required. Source your ROS 2 distribution before using physicalai-ros2-plugin."
            raise ImportError(msg) from exc

    @staticmethod
    def _message_types() -> dict[str, type]:
        return {
            "joint_state": importlib.import_module("sensor_msgs.msg").JointState,
            "trajectory": importlib.import_module("trajectory_msgs.msg").JointTrajectory,
            "trajectory_point": importlib.import_module("trajectory_msgs.msg").JointTrajectoryPoint,
            "float_array": importlib.import_module("std_msgs.msg").Float64MultiArray,
            "image": importlib.import_module("sensor_msgs.msg").Image,
        }

    def _spin(self) -> None:
        while not self._stop_spin.is_set():
            self._executor.spin_once(timeout_sec=0.1)

    def _on_joint_state(self, message: object) -> None:
        names = cast(list[str], getattr(message, "name"))
        values = cast(list[float], getattr(message, "position"))
        positions_by_name = dict(zip(names, values, strict=True))
        if not all(name in positions_by_name for name in self.joint_names):
            return
        def ordered(attribute: str) -> np.ndarray | None:
            source = cast(list[float], getattr(message, attribute))
            if len(source) != len(names):
                return None
            source_by_name = dict(zip(names, source, strict=True))
            return np.asarray(
                [source_by_name[name] for name in self.joint_names],
                dtype=np.float32,
            )

        positions = np.asarray([positions_by_name[name] for name in self.joint_names], dtype=np.float32)
        if self._angle_unit == "degrees":
            positions = np.rad2deg(positions).astype(np.float32)
        with self._state_lock:
            self._positions = positions
            self._velocities = ordered("velocity")
            self._effort = ordered("effort")
            self._state_timestamp = time.monotonic()
            self._state_ready.set()

    def _on_sensor(self, name: str, message: object) -> None:
        with self._state_lock:
            self._sensor_data[name] = np.asarray(getattr(message, "data"), dtype=np.float32)

    def _on_image(self, name: str, message: object) -> None:
        height, width = int(getattr(message, "height")), int(getattr(message, "width"))
        encoding = str(getattr(message, "encoding")).lower()
        channels = 3 if encoding in {"rgb8", "bgr8"} else 1
        image = np.frombuffer(getattr(message, "data"), dtype=np.uint8).reshape(height, width, channels)
        with self._state_lock:
            self._images[name] = cast("Frame", image)
