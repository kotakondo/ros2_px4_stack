"""
Hardware onboard launch: brings up everything that must run on the drone's
companion computer.

Starts:
  - mavros px4.launch (wired to the Pixhawk over USB)
  - trajectory_generator_ros2   (publishes <ns>/goal at pub_freq)
  - track_gen_traj_py           (Goal -> MAVROS setpoint stream, arm+OFFBOARD)
  - repub_odom                  (mocap/lidar pose -> mavros/vision_pose/pose)
  - mocap frame tf2 statics

The base station runs hw_base.launch.py separately (behavior_selector GUI,
optional rviz). Both machines must share ROS_DOMAIN_ID; zenoh handles
discovery across the network — no RMW/localhost tweaks needed.

Args (all have env-var / sensible defaults):
  fcu_url       Pixhawk serial URL (default /dev/ttyACM0:921600)
  tgt_system    MAVLink target system id (default $MAV_SYS_ID or 1)
  ns            Vehicle namespace (default $VEH_NAME)
  veh, num      Trajectory generator ns parts (defaults $VEHTYPE, $VEHNUM)
  odom_type     mocap | livox (default mocap)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:921600',
        description='Pixhawk FCU URL (serial for hardware, udp://... for SITL)',
    )
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value=EnvironmentVariable('MAV_SYS_ID', default_value='1'),
        description='MAVLink target system id',
    )
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value=EnvironmentVariable('VEH_NAME', default_value='SQ01'),
        description='Vehicle namespace (must match config/default.yaml key)',
    )
    veh_arg = DeclareLaunchArgument(
        'veh',
        default_value=EnvironmentVariable('VEHTYPE', default_value='SQ'),
    )
    num_arg = DeclareLaunchArgument(
        'num',
        default_value=EnvironmentVariable('VEHNUM', default_value='01'),
    )
    odom_type_arg = DeclareLaunchArgument(
        'odom_type',
        default_value='mocap',
        description='Pose source for MAVROS vision_pose (mocap or livox)',
    )

    ns = LaunchConfiguration('ns')

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'px4.launch')
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'namespace': [ns, '/mavros'],
        }.items(),
    )

    trajgen = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('trajectory_generator_ros2'),
                'launch', 'onboard.launch.py',
            )
        ),
        launch_arguments={
            'veh': LaunchConfiguration('veh'),
            'num': LaunchConfiguration('num'),
        }.items(),
    )

    track_gen_traj = Node(
        package='ros2_px4_stack',
        executable='track_gen_traj',
        name='track_gen_traj_py',
        namespace=ns,
        output='screen',
    )

    repub = Node(
        package='ros2_px4_stack',
        executable='repub_odom',
        name='repub_odom_py',
        namespace=ns,
        output='screen',
        parameters=[{'~odom_type': LaunchConfiguration('odom_type')}],
    )

    # Mocap frame plumbing — matches what offboard_gen_traj.launch.py sets up.
    mocap_to_mavros_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mocap_to_mavros_tf',
        arguments=['0', '0', '0', '-1.57', '3.14', '0',
                   'world_mocap', 'world_mavros'],
    )
    mocap_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mocap_to_world',
        arguments=['0', '0', '0', '0', '0', '0',
                   'world', 'world_mocap'],
    )

    return LaunchDescription([
        fcu_url_arg,
        tgt_system_arg,
        ns_arg,
        veh_arg,
        num_arg,
        odom_type_arg,
        mavros,
        trajgen,
        track_gen_traj,
        repub,
        mocap_to_mavros_tf,
        mocap_to_world_tf,
    ])
