#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np


class InitialPose(Node):
    def __init__(self, namespace, odom_type="mocap"):
        super().__init__("InitialPose")
        self.received = False
        self.odom_type = odom_type

        if odom_type == "mocap":
            self.create_subscription(PoseStamped, namespace + "/world", self.pose_cb, 10)
        elif odom_type == "livox":
            # Wait for LiDAR odometry (DLIO or Fast-LIO) for gravity-aligned orientation.
            self.create_subscription(Odometry, namespace + "/fast_lio/Odometry", self.odom_cb, 10)
            # Also subscribe to mocap for the world-frame position at init
            self.mocap_pose = None
            self.create_subscription(PoseStamped, namespace + "/world", self.mocap_cb, 10)

        # Timer prints waiting message at 1 Hz to stderr (won't be eval'd)
        self.timer = self.create_timer(1.0, self.waiting_cb)

    def waiting_cb(self):
        if not self.received:
            print("\033[1;33m****** [INIT POSE] WAITING FOR INITIAL POSE ******\033[0m",
                  file=sys.stderr, flush=True)

    def mocap_cb(self, msg):
        self.mocap_pose = msg

    def pose_cb(self, msg):
        """Mocap mode: use mocap pose directly as init pose."""
        if self.received:
            return
        self.received = True
        self.timer.cancel()

        position, orientation = msg.pose.position, msg.pose.orientation
        x, y, z = position.x, position.y, position.z
        roll, pitch, yaw = self.euler_from_quat(orientation)

        self._export(x, y, z, roll, pitch, yaw)

    def odom_cb(self, msg):
        """Livox mode: use mocap position but Fast-LIO orientation (gravity-aligned)."""
        if self.received:
            return
        if self.mocap_pose is None:
            return  # need mocap for world-frame position

        self.received = True
        self.timer.cancel()

        # Position from mocap (world frame ground truth)
        mp = self.mocap_pose.pose.position
        x, y, z = mp.x, mp.y, mp.z

        # Orientation from Fast-LIO's gravity-aligned frame
        # Fast-LIO starts at identity orientation in camera_init, but its gravity
        # vector defines the z-axis. Use mocap yaw (horizontal heading) but
        # Fast-LIO's pitch/roll (gravity alignment).
        flio_o = msg.pose.pose.orientation
        flio_roll, flio_pitch, _ = self.euler_from_quat(flio_o)

        mocap_o = self.mocap_pose.pose.orientation
        _, _, mocap_yaw = self.euler_from_quat(mocap_o)

        # Use mocap yaw + Fast-LIO pitch/roll (better gravity estimate)
        self._export(x, y, z, flio_roll, flio_pitch, mocap_yaw)

    def _export(self, x, y, z, roll, pitch, yaw):
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
    odom_type = os.environ.get("ODOM_TYPE", "mocap")
    print(f"\033[1;33m****** [INIT POSE] Mode: {odom_type} ******\033[0m", file=sys.stderr, flush=True)
    node = InitialPose(namespace=os.environ["VEH_NAME"], odom_type=odom_type)
    while rclpy.ok() and not node.received:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
