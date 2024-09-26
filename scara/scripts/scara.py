#!/usr/bin/env python3
from rclpy import init, shutdown, spin
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import JointState


class SimpleSCARA(Node):
    def __init__(self) -> None:
        super().__init__("scara")

        self._joint_state = JointState()
        self._joint_state.header.frame_id = "base_link"
        self._joint_state.name = ["joint1", "joint2", "joint3", "joint4"]
        self._joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self._joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        self._joint_state.effort = [0.0, 0.0, 0.0, 0.0]
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        self._loop = self.create_timer(0.05, self.joint_publish)

    def joint_publish(self):
        self._joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self._joint_state)


if __name__ == "__main__":
    init()
    spin(SimpleSCARA())
    shutdown()
