"""
teleop_real.launch.py — Vive leader + UR10e follower on one PC.

Usage:
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real.launch.py
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real.launch.py \\
      leader_rt:=true follower_rt:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_unilateral_vive_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'
    resources = f'{pkg_share}/resources'

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='ur3e',
        description='UR model the IK targets (matches follower mirror_sign).')
    calib_arg = DeclareLaunchArgument(
        'calib', default_value=f'{pkg_share}/config/calibration.yaml',
        description='YAML file with the tracker→UR-base transform')
    tracker_serial_arg = DeclareLaunchArgument(
        'tracker_serial', default_value='',
        description='Vive tracker serial (empty = first tracker found)')
    follower_ip_arg = DeclareLaunchArgument(
        'follower_ip', default_value='169.254.186.92',
        description='Follower (UR10e) IP')
    leader_rt_arg = DeclareLaunchArgument(
        'leader_rt', default_value='false',
        description='Enable PREEMPT_RT on the Vive leader node')
    follower_rt_arg = DeclareLaunchArgument(
        'follower_rt', default_value='false',
        description='Enable PREEMPT_RT on the follower node')

    leader = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='vive_leader_node',
        name='vive_leader_node',
        output='screen',
        arguments=[
            '--robot', LaunchConfiguration('robot'),
            '--config', config,
            '--calib', LaunchConfiguration('calib'),
            '--tracker-serial', LaunchConfiguration('tracker_serial'),
            '--rt-mode', LaunchConfiguration('leader_rt'),
        ],
    )

    follower = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='follower_node',
        name='follower_real_node',
        output='screen',
        arguments=[
            '--robot-ip', LaunchConfiguration('follower_ip'),
            '--config', config,
            '--resources-dir', resources,
            '--rt-mode', LaunchConfiguration('follower_rt'),
        ],
    )

    return LaunchDescription([
        robot_arg, calib_arg, tracker_serial_arg, follower_ip_arg,
        leader_rt_arg, follower_rt_arg,
        leader, follower,
    ])
