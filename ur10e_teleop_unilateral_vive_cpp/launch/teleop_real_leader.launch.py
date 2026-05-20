"""
teleop_real_leader.launch.py — Bimanual Vive-tracker leader (distributed).

Single process drives both trackers (left + right) and publishes to two
topic namespaces. Leave one side's serial empty for single-arm mode.
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
        description='UR model used as the IK target.')

    left_serial_arg = DeclareLaunchArgument(
        'left_serial', default_value='',
        description='Left-hand Vive tracker serial (empty = side disabled)')
    left_calib_arg = DeclareLaunchArgument(
        'left_calib', default_value=f'{pkg_share}/config/calibration_left.yaml',
        description='Left tracker→UR-base YAML transform')

    right_serial_arg = DeclareLaunchArgument(
        'right_serial', default_value='',
        description='Right-hand Vive tracker serial (empty = side disabled)')
    right_calib_arg = DeclareLaunchArgument(
        'right_calib', default_value=f'{pkg_share}/config/calibration_right.yaml',
        description='Right tracker→UR-base YAML transform')

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
            '--left-serial', LaunchConfiguration('left_serial'),
            '--left-calib', LaunchConfiguration('left_calib'),
            '--right-serial', LaunchConfiguration('right_serial'),
            '--right-calib', LaunchConfiguration('right_calib'),
            '--rate-hz', LaunchConfiguration('rate_hz'),
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return LaunchDescription([
        robot_arg,
        left_serial_arg, left_calib_arg,
        right_serial_arg, right_calib_arg,
        rate_arg, rt_arg,
        leader,
    ])
