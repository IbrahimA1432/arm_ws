#!/usr/bin/env python3

import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class Ps5ArmTeleop(Node):
    """
    Subscribes: /joy, /joint_states
    Publishes: /arm_controller/joint_trajectory, /gripper_controller/joint_trajectory

    It integrates joystick axes into joint position targets (position control).
    """

    def __init__(self):
        super().__init__("ps5_arm_teleop")

        # Controller topics
        self.arm_traj_topic = self.declare_parameter(
            "arm_traj_topic", "/arm_controller/joint_trajectory"
        ).value
        self.gripper_traj_topic = self.declare_parameter(
            "gripper_traj_topic", "/gripper_controller/joint_trajectory"
        ).value

        # Joints
        self.arm_joints = self.declare_parameter(
            "arm_joints",
            ["base_rotator_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "end_joint"],
        ).value
        self.gripper_joint = self.declare_parameter(
            "gripper_joint", "gear_right_joint"
        ).value

        # Limits (radians). Use your real limits.
        self.arm_lower = self.declare_parameter(
            "arm_lower", [-1.57, -1.57, -1.57, -1.57, -1.57]
        ).value
        self.arm_upper = self.declare_parameter(
            "arm_upper", [1.57, 1.57, 1.57, 1.57, 1.57]
        ).value

        # Gripper limits (radians). Tune these to your mechanism.
        self.grip_lower = float(self.declare_parameter("grip_lower", -0.6).value)
        self.grip_upper = float(self.declare_parameter("grip_upper", 0.6).value)

        # Rates
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.traj_time = float(self.declare_parameter("traj_time", 0.12).value)

        # Deadband
        self.axis_deadband = float(self.declare_parameter("axis_deadband", 0.08).value)

        # PS5 axis mapping defaults (often correct, but verify with /joy echo)
        # Typical:
        # axes[0]=LS left/right, axes[1]=LS up/down
        # axes[3]=RS left/right, axes[4]=RS up/down
        # triggers sometimes axes[2], axes[5] (varies)
        self.axis_base = int(self.declare_parameter("axis_base", 0).value)       # yaw
        self.axis_shoulder = int(self.declare_parameter("axis_shoulder", 1).value)
        self.axis_elbow = int(self.declare_parameter("axis_elbow", 4).value)
        self.axis_wrist = int(self.declare_parameter("axis_wrist", 3).value)
        self.axis_end = int(self.declare_parameter("axis_end", 2).value)         # optional

        # Scale (rad/s) for each axis when fully deflected
        self.scale_base = float(self.declare_parameter("scale_base", 1.2).value)
        self.scale_shoulder = float(self.declare_parameter("scale_shoulder", 1.0).value)
        self.scale_elbow = float(self.declare_parameter("scale_elbow", 1.0).value)
        self.scale_wrist = float(self.declare_parameter("scale_wrist", 1.2).value)
        self.scale_end = float(self.declare_parameter("scale_end", 1.0).value)

        # Buttons for gripper open/close (verify indices)
        # Common: L1=4, R1=5 on many mappings
        self.btn_grip_open = int(self.declare_parameter("btn_grip_open", 4).value)
        self.btn_grip_close = int(self.declare_parameter("btn_grip_close", 5).value)
        self.grip_step = float(self.declare_parameter("grip_step", 0.04).value)

        # Optional enable button (set to -1 to always enabled)
        self.enable_button = int(self.declare_parameter("enable_button", -1).value)

        self.arm_pub = self.create_publisher(JointTrajectory, self.arm_traj_topic, 10)
        self.grip_pub = self.create_publisher(JointTrajectory, self.gripper_traj_topic, 10)

        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)

        self.axes: List[float] = []
        self.buttons: List[int] = []

        self.joint_pos: Dict[str, float] = {}
        self.have_joint_state = False

        self.arm_target: Optional[List[float]] = None
        self.grip_target: Optional[float] = None

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(f"Publishing arm to {self.arm_traj_topic}")
        self.get_logger().info(f"Publishing gripper to {self.gripper_traj_topic}")

    def on_joy(self, msg: Joy):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

    def on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = float(pos)
        self.have_joint_state = True

        if self.arm_target is None:
            self.arm_target = [self.joint_pos.get(j, 0.0) for j in self.arm_joints]
        if self.grip_target is None:
            self.grip_target = self.joint_pos.get(self.gripper_joint, 0.0)

    def _axis(self, idx: int) -> float:
        if idx < 0 or idx >= len(self.axes):
            return 0.0
        v = float(self.axes[idx])
        if abs(v) < self.axis_deadband:
            return 0.0
        return v

    def _btn(self, idx: int) -> int:
        if idx < 0 or idx >= len(self.buttons):
            return 0
        return int(self.buttons[idx])

    def _enabled(self) -> bool:
        if self.enable_button < 0:
            return True
        return self._btn(self.enable_button) == 1

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if not self.have_joint_state or self.arm_target is None or self.grip_target is None:
            return

        if not self._enabled():
            return

        # Build joint velocity commands from axes
        v_base = self._axis(self.axis_base) * self.scale_base
        v_sh = -self._axis(self.axis_shoulder) * self.scale_shoulder
        v_el = -self._axis(self.axis_elbow) * self.scale_elbow
        v_wr = self._axis(self.axis_wrist) * self.scale_wrist
        v_end = self._axis(self.axis_end) * self.scale_end

        v = [v_base, v_sh, v_el, v_wr, v_end]

        # Integrate to position targets and clamp to limits
        for i in range(len(self.arm_target)):
            self.arm_target[i] = clamp(self.arm_target[i] + v[i] * dt,
                                       float(self.arm_lower[i]), float(self.arm_upper[i]))

        # Gripper buttons step
        if self._btn(self.btn_grip_open):
            self.grip_target = clamp(self.grip_target + self.grip_step, self.grip_lower, self.grip_upper)
        if self._btn(self.btn_grip_close):
            self.grip_target = clamp(self.grip_target - self.grip_step, self.grip_lower, self.grip_upper)

        self.publish_arm(self.arm_target)
        self.publish_gripper(self.grip_target)

    def publish_arm(self, positions: List[float]):
        msg = JointTrajectory()
        msg.joint_names = list(self.arm_joints)

        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(self.traj_time),
                                      nanosec=int((self.traj_time % 1.0) * 1e9))
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def publish_gripper(self, pos: float):
        msg = JointTrajectory()
        msg.joint_names = [self.gripper_joint]

        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start = Duration(sec=int(self.traj_time),
                                      nanosec=int((self.traj_time % 1.0) * 1e9))
        msg.points = [pt]
        self.grip_pub.publish(msg)


def main():
    rclpy.init()
    node = Ps5ArmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

