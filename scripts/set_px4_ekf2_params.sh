#!/bin/bash
# Set PX4 EKF2 and position controller parameters for external vision (DLIO/Mocap)
# Run this after MAVROS is connected to the FCU.
#
# Usage: bash set_px4_ekf2_params.sh [NAMESPACE]
#   e.g.: bash set_px4_ekf2_params.sh /PX04

NS="${1:-/$VEH_NAME}"

echo "Setting PX4 parameters on namespace: $NS"

# --- EKF2: External Vision Fusion ---
# Use vision for position/velocity/yaw
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EKF2_EV_CTRL', value: {integer: 15}}"
# Vision position noise [m] — lower = more trust in DLIO
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EKF2_EVP_NOISE', value: {real: 0.01}}"
# Vision velocity noise [m/s]
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EKF2_EVV_NOISE', value: {real: 0.01}}"
# Vision angle noise [rad]
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EKF2_EVA_NOISE', value: {real: 0.01}}"
# Vision delay compensation [ms] — match DLIO pipeline latency
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EKF2_EV_DELAY', value: {real: 25.0}}"
# Height reference: 3 = Vision
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EKF2_HGT_REF', value: {integer: 3}}"

# --- Position Controller ---
# XY position P gain (default 0.95)
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'MPC_XY_P', value: {real: 1.1}}"
# Z position P gain (default 1.0)
ros2 service call ${NS}/mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'MPC_Z_P', value: {real: 1.2}}"

echo "Done. Parameters set on $NS."
echo "Note: Some parameters require a PX4 reboot to take effect (e.g. EKF2_EV_CTRL, EKF2_HGT_REF)."
echo "Reboot FCU with:  ros2 service call ${NS}/mavros/cmd/command mavros_msgs/srv/CommandLong \"{command: 246, param1: 1.0}\""
