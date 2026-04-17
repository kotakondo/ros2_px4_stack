#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, Twist, Vector3
from std_msgs.msg import Header
from mavros import mavlink
from mavros_msgs.msg import (
    Altitude,
    ExtendedState,
    HomePosition,
    ParamValue,
    State,
    WaypointList,
    Waypoint,
    Mavlink,
    CommandCode,
)
from mavros_msgs.srv import (
    CommandBool,
    ParamGet,
    ParamSet,
    SetMode,
    WaypointClear,
    WaypointPush,
)
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import ParamValue
from sensor_msgs.msg import NavSatFix, Imu
from pymavlink import mavutil
from typing import List, Tuple
from six.moves import xrange

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from snapstack_msgs2.msg import Goal as GoalSnap
from snapstack_msgs2.msg import State as StateSnape

LOCAL_NAVIGATION = 0
GLOBAL_NAVIGATION = 1
NAVIGATION_MODES = [LOCAL_NAVIGATION, GLOBAL_NAVIGATION]

from .trajgen_offboard_node import OffboardTrajgenFollower

class SmoothTrajectoryTracker(OffboardTrajgenFollower):

    def __init__(self,
        node_name: str="smooth_trajectory_tracker",
        navigation_mode: int=LOCAL_NAVIGATION,
    ):

        super().__init__(node_name=node_name, navigation_mode=navigation_mode)

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # param/set client already created by the base class (ParamSet v1);
        # reuse it via self.set_param_srv rather than binding a second type.

    def _set_px4_param(self, name, value, is_int=False):
        if not self.set_param_srv.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"param/set unavailable; skip {name}")
            return
        req = ParamSet.Request()
        req.param_id = name
        req.value = ParamValue()
        if is_int:
            req.value.integer = int(value)
            req.value.real = 0.0
        else:
            req.value.integer = 0
            req.value.real = float(value)
        self.set_param_srv.call_async(req)
        self.get_logger().info(f"set PX4 param {name} = {value}")

    def _kick_offboard(self):
        """PX4 requires a stream of setpoints before OFFBOARD latches.
        This runs in a background thread — it must NOT spin the node's
        executor (the main thread does that). `call_async` is fire-and-forget;
        we observe the outcome by polling `self.state`, which is updated by
        the main thread's spin via the MAVROS state callback."""
        import time as _time
        self.get_logger().info("Waiting for trajectory setpoints...")
        while rclpy.ok() and self.trajectory_setpoint is None:
            _time.sleep(0.1)

        # Relax PX4 SITL failsafes that have bitten us:
        #   COM_DISARM_PRFLT: auto-disarm if no takeoff detected within N s of
        #     arming. Our first post-arm setpoints aren't always enough climb
        #     to trigger PX4's "takeoff detected", so it kills motors at 10 s.
        #   COM_OF_LOSS_T: offboard loss timeout (default 1 s — too tight if
        #     the setpoint stream briefly hiccups).
        #   COM_DISARM_LAND: auto-disarm on landed-detected; a momentary
        #     altitude dip can trip this mid-flight.
        self._set_px4_param("COM_DISARM_PRFLT", -1.0)
        self._set_px4_param("COM_OF_LOSS_T",    5.0)
        self._set_px4_param("COM_DISARM_LAND",  -1.0)

        self.get_logger().info("Setpoints flowing; priming OFFBOARD (1s)...")
        _time.sleep(1.0)

        set_mode_req = SetMode.Request()
        set_mode_req.base_mode = 0
        set_mode_req.custom_mode = "OFFBOARD"
        arm_req = CommandBool.Request()
        arm_req.value = True

        for _ in range(20):
            if self.state.mode != "OFFBOARD":
                self.set_mode_srv.call_async(set_mode_req)
            if not self.state.armed:
                self.set_arming_srv.call_async(arm_req)
            _time.sleep(0.5)
            if self.state.mode == "OFFBOARD" and self.state.armed:
                break

        self.get_logger().info(
            f"PX4 mode={self.state.mode} armed={self.state.armed}"
        )

    def track_smooth_trajectory(self):
        Thread(target=self._kick_offboard, daemon=True).start()
        self.track_trajectory()

def main(args=None):
    rclpy.init(args=args)
    traj_tracker = SmoothTrajectoryTracker()
    traj_tracker.track_smooth_trajectory()

if __name__ == '__main__':
    main()

