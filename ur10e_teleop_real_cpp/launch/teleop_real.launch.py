"""
teleop_real.launch.py — C++ bilateral teleop, both nodes on one PC.

Usage:
  ros2 launch ur10e_teleop_real_cpp teleop_real.launch.py
  ros2 launch ur10e_teleop_real_cpp teleop_real.launch.py rt:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_real_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'

    robot_arg = DeclareLaunchArgument('robot', default_value='ur3e',
        description='Leader robot type (ur3e or ur10e)')
    leader_ip_arg = DeclareLaunchArgument('leader_ip',
        default_value='169.254.186.94', description='Leader (UR3e) IP')
    follower_ip_arg = DeclareLaunchArgument('follower_ip',
        default_value='169.254.186.92', description='Follower (UR10e) IP')
    rt_arg = DeclareLaunchArgument('rt', default_value='false',
        description='Enable PREEMPT_RT mode (SCHED_FIFO + mlockall)')

    leader = Node(
        package='ur10e_teleop_real_cpp',
        executable='leader_node',
        name='leader_real_node',
        output='screen',
        arguments=[
            '--robot', LaunchConfiguration('robot'),
            '--robot-ip', LaunchConfiguration('leader_ip'),
            '--config', config,
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    follower = Node(
        package='ur10e_teleop_real_cpp',
        executable='follower_node',
        name='follower_real_node',
        output='screen',
        arguments=[
            '--robot-ip', LaunchConfiguration('follower_ip'),
            '--config', config,
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return LaunchDescription([
        robot_arg, leader_ip_arg, follower_ip_arg, rt_arg,
        leader, follower,
    ])
