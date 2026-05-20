"""
teleop_real.launch.py — Bimanual Vive leader + UR10e follower(s), single PC.

For bimanual operation you typically run two follower instances (one per
UR10e arm) with distinct robot IPs. The default below covers the
single-arm case; uncomment / duplicate the follower block for the right
arm when bimanual hardware is online.
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

    robot_arg = DeclareLaunchArgument('robot', default_value='ur3e')

    # PC e's paired Vive trackers, left/right confirmed by hand-swing
    # test (2026-05-20). Base stations: LHB-45131F3B, LHB-BB0267D2.
    left_serial_arg = DeclareLaunchArgument(
        'left_serial', default_value='LHR-B4BFDF90',
        description='Left tracker serial (empty = disabled)')
    # Empty calib path → node uses auto-tare. Set to a real YAML once
    # a calibration is captured.
    left_calib_arg = DeclareLaunchArgument(
        'left_calib', default_value='',
        description='Left tracker→UR-base YAML (empty = auto-tare)')

    right_serial_arg = DeclareLaunchArgument(
        'right_serial', default_value='LHR-C21814A6',
        description='Right tracker serial (empty = disabled)')
    right_calib_arg = DeclareLaunchArgument(
        'right_calib', default_value='',
        description='Right tracker→UR-base YAML (empty = auto-tare)')

    follower_ip_arg = DeclareLaunchArgument(
        'follower_ip', default_value='169.254.186.92',
        description='Follower (UR10e) IP — primary arm')

    leader_rt_arg = DeclareLaunchArgument('leader_rt', default_value='false')
    follower_rt_arg = DeclareLaunchArgument('follower_rt', default_value='false')

    leader = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='vive_leader_node',
        name='vive_leader_node',
        output='screen',
        arguments=[
            '--robot', LaunchConfiguration('robot'),
            '--config', config,
            '--left-serial', LaunchConfiguration('left_serial'),
            '--left-calib', LaunchConfiguration('left_calib'),
            '--right-serial', LaunchConfiguration('right_serial'),
            '--right-calib', LaunchConfiguration('right_calib'),
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
        robot_arg,
        left_serial_arg, left_calib_arg,
        right_serial_arg, right_calib_arg,
        follower_ip_arg,
        leader_rt_arg, follower_rt_arg,
        leader, follower,
    ])
