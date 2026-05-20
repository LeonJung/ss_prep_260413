"""
teleop_real_leader.launch.py — Vive-tracker-based leader (distributed setup).

Replaces the UR3e leader from ur10e_teleop_control_unilateral_cpp with a
node driven by a Vive tracker via SteamVR/OpenVR. Same output topic
schema, so the existing follower works without modification.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_unilateral_vive_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='ur3e',
        description='UR model used as the IK target (matches follower mirror_sign)')
    calib_arg = DeclareLaunchArgument(
        'calib', default_value=f'{pkg_share}/config/calibration.yaml',
        description='YAML file with the tracker→UR-base transform')
    tracker_serial_arg = DeclareLaunchArgument(
        'tracker_serial', default_value='',
        description='Vive tracker serial (empty = first tracker found)')
    rate_arg = DeclareLaunchArgument('rate_hz', default_value='500.0')
    rt_arg = DeclareLaunchArgument('rt', default_value='false')

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
            '--rate-hz', LaunchConfiguration('rate_hz'),
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return LaunchDescription([
        robot_arg, calib_arg, tracker_serial_arg, rate_arg, rt_arg,
        leader,
    ])
