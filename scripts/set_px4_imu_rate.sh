#!/bin/bash
# Set PX4 MAVLink raw sensor stream rate to 200 Hz (includes IMU data_raw).
# Run this after MAVROS is connected to the FCU.
#
# Usage: bash set_px4_imu_rate.sh [NAMESPACE]
#   e.g.: bash set_px4_imu_rate.sh /PX04

NS="${1:-/$VEH_NAME}"

echo "Setting raw sensor stream rate to 200 Hz on namespace: $NS"

# HIGHRES_IMU (message_id=105) -> /mavros/imu/data_raw
ros2 service call ${NS}/mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"

echo "Done. IMU raw sensor rate set to 200 Hz on $NS."
echo "Note: This must be re-run each time MAVROS starts."
