#!/usr/bin/env python3
"""Check IMU topic rate and print pass/fail in color."""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

GREEN = "\033[1;32m"
RED = "\033[1;31m"
RESET = "\033[0m"

class RateChecker(Node):
    def __init__(self, topic):
        super().__init__("imu_rate_checker")
        self.timestamps = []
        self.sub = self.create_subscription(Imu, topic, self.cb, qos_profile_sensor_data)
        self.timer = self.create_timer(5.0, self.check)
        self.get_logger().info(f"Measuring rate on {topic} for 5 seconds...")

    def cb(self, msg):
        self.timestamps.append(time.monotonic())

    def check(self):
        if len(self.timestamps) < 2:
            print(f"{RED}ERROR: No messages received on topic{RESET}")
        else:
            duration = self.timestamps[-1] - self.timestamps[0]
            rate = (len(self.timestamps) - 1) / duration
            if rate >= 180:
                print(f"{GREEN}IMU rate: {rate:.1f} Hz [OK]{RESET}")
            else:
                print(f"{RED}ERROR: IMU rate: {rate:.1f} Hz (expected ~200 Hz){RESET}")
        raise SystemExit(0)

def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/mavros/imu/data_raw"
    rclpy.init()
    node = RateChecker(topic)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
