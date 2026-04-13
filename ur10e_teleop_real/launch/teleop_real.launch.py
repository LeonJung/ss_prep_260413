"""
teleop_real.launch.py — Launch with real UR robots via RTDE.

Usage:
  ros2 launch ur10e_teleop_real teleop_real.launch.py leader_ip:=192.168.1.100 follower_ip:=192.168.1.101
  ros2 launch ur10e_teleop_real teleop_real.launch.py robot:=ur3e leader_ip:=192.168.1.100 follower_ip:=192.168.1.101
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_real')
    pkg_lib = pkg_share.replace('/share/', '/lib/')
    config = f'{pkg_share}/config/real_ur.yaml'

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='ur3e',
        description='Leader robot type (ur10e or ur3e)')
    leader_ip_arg = DeclareLaunchArgument(
        'leader_ip', default_value='192.168.1.100',
        description='Leader UR robot IP address')
    follower_ip_arg = DeclareLaunchArgument(
        'follower_ip', default_value='192.168.1.101',
        description='Follower UR robot IP address')

    leader = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/leader_real_node.py',
            '--robot', LaunchConfiguration('robot'),
            '--client', 'rtde',
            '--robot-ip', LaunchConfiguration('leader_ip'),
            '--config', config,
        ],
        name='leader_real_node',
        output='screen',
    )

    follower = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/follower_real_node.py',
            '--client', 'rtde',
            '--robot-ip', LaunchConfiguration('follower_ip'),
            '--config', config,
        ],
        name='follower_real_node',
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        leader_ip_arg,
        follower_ip_arg,
        leader,
        follower,
    ])
