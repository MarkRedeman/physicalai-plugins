# PhysicalAI ROS 2 Plugin

`physicalai-ros2-plugin` adapts a ROS 2 robot that publishes
`sensor_msgs/msg/JointState` and accepts
`trajectory_msgs/msg/JointTrajectory` into the PhysicalAI `Robot` protocol.
ROS positions are radians; set `angle_unit="degrees"` when PhysicalAI should
use degrees.

## Install

Install and source a ROS 2 distribution first (including `rclpy`,
`sensor_msgs`, `trajectory_msgs`, and `std_msgs`). The `ros2` extra deliberately
has no PyPI requirements because these packages are provided by the ROS distro.

```bash
source /opt/ros/<distro>/setup.bash
uv add physicalai-ros2-plugin
```

## Direct use

```python
from physicalai_ros2_plugin import ROS2Robot

robot = ROS2Robot(
    joint_names=["joint_1", "joint_2"],
    state_topic="/joint_states",
    command_topic="/joint_trajectory_controller/joint_trajectory",
)
robot.connect()
try:
    robot.send_action(robot.get_observation().joint_positions)
finally:
    robot.disconnect()
```

`get_observation()` returns ordered positions plus `velocities` and `effort`
when present in `JointState`. Configure `sensor_topics` for
`std_msgs/msg/Float64MultiArray` extensions and `camera_topics` for
`sensor_msgs/msg/Image` extensions.

## Studio

The package registers **Generic ROS 2 Follower**. Supply the joint order and
topic configuration. It intentionally has no URDF asset; robot-specific plugins
can add an asset and joint map later.

## Remote deployment through Zenoh

Run the owner beside the ROS graph:

```bash
physicalai-ros2-owner --name lab-arm --joint-names joint_1 joint_2
```

Then attach from another PhysicalAI process with
`SharedRobot.attach("lab-arm")`. This reuses PhysicalAI's existing Zenoh
transport; direct ROS 2 use does not require Zenoh.
