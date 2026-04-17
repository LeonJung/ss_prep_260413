"""
teleop_dummy.launch.py — Launch with dummy.yaml config (no real hardware).

Usage:
  ros2 launch ur10e_teleop_real teleop_dummy.launch.py
  ros2 launch ur10e_teleop_real teleop_dummy.launch.py robot:=ur3e
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_real')
    pkg_lib = pkg_share.replace('/share/', '/lib/')
    config = f'{pkg_share}/config/dummy.yaml'

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='ur3e',
        description='Leader robot type (ur10e or ur3e)')

    leader = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/leader_real_node.py',
            '--robot', LaunchConfiguration('robot'),
            '--config', config,
        ],
        name='leader_real_node',
        output='screen',
    )

    follower = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/follower_real_node.py',
            '--config', config,
        ],
        name='follower_real_node',
        output='screen',
    )

    return LaunchDescription([robot_arg, leader, follower])
