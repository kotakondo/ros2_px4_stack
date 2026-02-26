#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np


class InitialPose(Node):
    def __init__(self, namespace):
        super().__init__("InitialPose")
        self.received = False
        self.create_subscription(PoseStamped, namespace + "/world", self.pose_cb, 10)
        # Timer prints waiting message at 1 Hz to stderr (won't be eval'd)
        self.timer = self.create_timer(1.0, self.waiting_cb)

    def waiting_cb(self):
        if not self.received:
            print("\033[1;33m****** [INIT POSE] WAITING FOR INITIAL POSE ******\033[0m",
                  file=sys.stderr, flush=True)

    def pose_cb(self, msg):
        if self.received:
            return
        self.received = True
        self.timer.cancel()

        position, orientation = msg.pose.position, msg.pose.orientation
        x, y, z = position.x, position.y, position.z
        roll, pitch, yaw = self.euler_from_quat(orientation)

        # Export statements go to stdout (captured by eval in shell)
        print(f'export INIT_X={x}')
        print(f'export INIT_Y={y}')
        print(f'export INIT_Z={z}')
        print(f'export INIT_ROLL={roll}')
        print(f'export INIT_PITCH={pitch}')
        print(f'export INIT_YAW={yaw}')

    def euler_from_quat(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init()
    node = InitialPose(namespace=os.environ["VEH_NAME"])
    while rclpy.ok() and not node.received:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
