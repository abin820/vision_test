import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('sp_nav_bringup')
    params_file = os.path.join(bringup_share, 'config', 'nav_params.yaml')

    return LaunchDescription([
        Node(
            package='sp_global_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
    ])
