import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sp_nav_sim')
    params_file = os.path.join(pkg_share, 'config', 'sim_robot.yaml')

    return LaunchDescription([
        Node(
            package='sp_nav_sim',
            executable='sim_robot',
            name='sim_robot_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
