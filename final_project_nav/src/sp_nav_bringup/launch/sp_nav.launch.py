import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('sp_nav_bringup')
    params_file = os.path.join(bringup_share, 'config', 'nav_params.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rviz.rviz')

    return LaunchDescription([
        Node(
            package='sp_map_server',
            executable='esdf_map_publisher',
            name='esdf_map_publisher',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='sp_global_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='sp_controller_server',
            executable='controller_node',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='sp_decision',
            executable='sp_decision_node',
            name='sp_decision',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='sp_nav_bt',
            executable='nav_interface_node',
            name='nav_interface_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
