"""
Hardware base-station launch: brings up the operator-side nodes.

Starts:
  - behavior_selector2 rqt GUI (GO / END / KILL buttons)
  - behavior_selector_node (service that publishes /globalflightmode)

The drone runs hw_onboard.launch.py separately. Both machines share
ROS_DOMAIN_ID; zenoh handles discovery — no RMW/localhost tweaks needed.

RViz is intentionally not started here (it's optional and usually run in its
own terminal with a custom config); uncomment the rviz_node block below if
you want a default RViz2 brought up alongside the GUI.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


def generate_launch_description():
    behavior_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('behavior_selector2'),
                'launch', 'gui.launch.py',
            )
        )
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=[
    #         '-d',
    #         os.path.join(
    #             get_package_share_directory('trajectory_generator_ros2'),
    #             'config', 'default.rviz',
    #         ),
    #     ],
    # )

    return LaunchDescription([
        behavior_gui,
        # rviz_node,
    ])
