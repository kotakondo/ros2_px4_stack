#!/usr/bin/env python3
"""Monitor orientation from mavros pose topic. Stops when |w| > 0.999."""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

GREEN = "\033[1;32m"
RESET = "\033[0m"


class OrientationMonitor(Node):
    def __init__(self, topic):
        super().__init__("orientation_monitor")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(PoseStamped, topic, self.cb, qos)

    def cb(self, msg):
        o = msg.pose.orientation
        w_str = f"{GREEN}  w: {o.w:.6f}{RESET}"
        print(
            f"orientation:\n"
            f"  x: {o.x:.6f}\n"
            f"  y: {o.y:.6f}\n"
            f"  z: {o.z:.6f}\n"
            f"{w_str}\n"
            f"---"
        )
        if abs(o.w) > 0.999:
            print(f"\n{GREEN}****** [ORIENTATION] STATE CONVERGED ******{RESET}")
            raise SystemExit(0)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "mavros/local_position/pose"
    rclpy.init()
    node = OrientationMonitor(topic)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
