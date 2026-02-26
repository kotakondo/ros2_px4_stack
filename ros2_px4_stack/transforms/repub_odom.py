#! /usr/bin/env python3
"""A script to republish motion capture data from a ROS topic to a different topic."""


import rclpy
from rclpy.node import Node
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
import threading
from threading import Thread
from nav_msgs.msg import Odometry

class OdomRepublisher(Node):
    def __init__(self, pub_hz=100.0):
        super().__init__('odom_publisher')

        namespace = self.get_namespace()
        self.odom_type = self.declare_parameter("~odom_type", "mocap").value # Odom type defaults to mocap

        # Create subscription for mocap or lidar odometry
        if self.odom_type == "mocap":
            self.pose_sub_topic_name = self.declare_parameter(
                "~pose_sub_topic_name", namespace + "/world"
            ).value
            self.twist_sub_topic_name = self.declare_parameter(
                "~twist_sub_topic_name", namespace + "/mocap/twist"
            ).value
            self._mocap_pose_sub = self.create_subscription(PoseStamped, self.pose_sub_topic_name, self._mocap_pose_cb, 10)
            self._mocap_twist_sub = self.create_subscription(TwistStamped, self.twist_sub_topic_name, self._mocap_twist_cb, 10)
        elif self.odom_type == "livox":
            self.odom_sub_topic_name = self.declare_parameter(
                "~odom_sub_topic_name", namespace + "/dlio/odom_node/odom"
            ).value
            self._odom_sub = self.create_subscription(Odometry, self.odom_sub_topic_name, self._livox_odom_cb, 10)

        # Create pose and twist publishers
        self.pose_pub_topic_name = self.declare_parameter(
            "~pose_pub_topic_name",  namespace + "/mavros/vision_pose/pose_cov"
        ).value

        self.twist_pub_topic_name = self.declare_parameter(
            "~twist_pub_topic_name", namespace + "/mavros/vision_speed/speed_twist_cov"
        ).value

        self._last_pose_msg = None
        self._last_twist_msg = None

        # Publish pose and twist with covariance at the specified rate in a separate thread
        self._pub_hz = pub_hz
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_pub_topic_name, 10)
        self._twist_pub = self.create_publisher(TwistWithCovarianceStamped, self.twist_pub_topic_name, 10)
        self._pub_thread = Thread(target=self._publish_loop, args=())
        self._pub_thread.daemon = True
        self._pub_thread.start()

    def _mocap_pose_cb(self, msg):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = msg.header.frame_id
        pose_msg.pose.pose = msg.pose

        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.001   # x  variance [m^2]  (mocap is more precise than lidar)
        pose_cov[7]  = 0.001   # y  variance [m^2]
        pose_cov[14] = 0.001   # z  variance [m^2]
        pose_cov[21] = 0.005   # roll  variance [rad^2]
        pose_cov[28] = 0.005   # pitch variance [rad^2]
        pose_cov[35] = 0.005   # yaw   variance [rad^2]
        pose_msg.pose.covariance = pose_cov
        self._last_pose_msg = pose_msg

    def _mocap_twist_cb(self, msg):
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = msg.header.stamp
        twist_msg.header.frame_id = msg.header.frame_id
        twist_msg.twist.twist = msg.twist

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.001   # vx variance [(m/s)^2]
        twist_cov[7]  = 0.001   # vy variance [(m/s)^2]
        twist_cov[14] = 0.001   # vz variance [(m/s)^2]
        twist_cov[21] = 0.005   # wx variance [(rad/s)^2]
        twist_cov[28] = 0.005   # wy variance [(rad/s)^2]
        twist_cov[35] = 0.005   # wz variance [(rad/s)^2]
        twist_msg.twist.covariance = twist_cov
        self._last_twist_msg = twist_msg

    def _livox_odom_cb(self, msg):
        # Populate pose_cov msg
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = msg.header.frame_id
        pose_msg.pose.pose = msg.pose.pose

        # DLIO publishes zero covariance — PX4 EKF2 needs nonzero values
        # to trust the external vision estimate.  Diagonal: [x, y, z, roll, pitch, yaw]
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.001  # x  variance [m^2]
        pose_cov[7]  = 0.001  # y  variance [m^2]
        pose_cov[14] = 0.001  # z  variance [m^2]
        pose_cov[21] = 0.002  # roll  variance [rad^2]
        pose_cov[28] = 0.002  # pitch variance [rad^2]
        pose_cov[35] = 0.002  # yaw   variance [rad^2]
        pose_msg.pose.covariance = pose_cov
        self._last_pose_msg = pose_msg

        # Populate twist_cov msg
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = msg.header.stamp
        twist_msg.header.frame_id = msg.header.frame_id
        twist_msg.twist.twist = msg.twist.twist

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.001  # vx variance [(m/s)^2]
        twist_cov[7]  = 0.001  # vy variance [(m/s)^2]
        twist_cov[14] = 0.001  # vz variance [(m/s)^2]
        twist_cov[21] = 0.002  # wx variance [(rad/s)^2]
        twist_cov[28] = 0.002  # wy variance [(rad/s)^2]
        twist_cov[35] = 0.002  # wz variance [(rad/s)^2]
        twist_msg.twist.covariance = twist_cov
        self._last_twist_msg = twist_msg

    def _publish_loop(self):
        rate = self.create_rate(self._pub_hz)
        while rclpy.ok():
            if self._last_pose_msg:
                self._pose_pub.publish(self._last_pose_msg)
            if self._last_twist_msg:
                self._twist_pub.publish(self._last_twist_msg)
            rate.sleep()

def main():
    rclpy.init()
    node = OdomRepublisher(pub_hz=100.0)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
