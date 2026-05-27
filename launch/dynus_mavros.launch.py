from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

def generate_launch_description():
    namespace = LaunchConfiguration("ns")
    init_x, init_y, init_z = os.environ.get("INIT_X"), os.environ.get("INIT_Y"), os.environ.get("INIT_Z")
    init_roll, init_pitch, init_yaw = os.environ.get("INIT_ROLL"), os.environ.get("INIT_PITCH"), os.environ.get("INIT_YAW")
    veh = os.environ.get("VEH_NAME")

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('argname', default_value='val'),
        DeclareLaunchArgument('hostname', default_value='nuc6'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value=EnvironmentVariable("VEH_NAME")),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
        DeclareLaunchArgument('odom_type', default_value='livox'),

        # Run dynus node
        Node(
            package='ros2_px4_stack',
            executable='track_dynus_traj',
            name='track_dynus_traj_py',
            namespace=namespace,
            output='screen',
        ),

        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{veh}_odom_to_mocap',
            arguments=[init_x, init_y, init_z, init_yaw, init_pitch, init_roll, 'world_mocap', f'{veh}/init_pose']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'world_mocap']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),
        # camera_init (Fast-LIO origin) = PX03/init_pose (DLIO origin)
        # Both represent the drone's starting position. Linking them with identity
        # ensures DYNUS's TF lookup (map -> PX03/init_pose) produces the correct
        # transform for converting goals to camera_init frame (which PX4 operates in).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_pose_to_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', f'{veh}/init_pose', 'camera_init']
        ),
        # Fast-LIO publishes camera_init -> body; map this to PX03/base_link so the
        # mapper's TF chain (world -> camera_init -> body -> PX03/base_link) works
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'body', f'{veh}/base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_d455',
            arguments=['0', '0', '0', '0', '0', '0', f"{veh}/base_link", f"{veh}/d455_link"]
        ),

        # Run repub_livox node
        Node(
            package='ros2_px4_stack',
            executable='repub_odom',
            name='repub_odom_py',
            namespace=namespace,
            output='screen',
            parameters=[{
                '~odom_type': LaunchConfiguration('odom_type'),
            }]
        ),

        # Run mocap to livox command frame conversion
        Node(
            package='ros2_px4_stack',
            executable='mocap_to_livox_frame',
            name='mocap_to_livox_frame',
            namespace=namespace,
            output='screen',
        )
    ])
