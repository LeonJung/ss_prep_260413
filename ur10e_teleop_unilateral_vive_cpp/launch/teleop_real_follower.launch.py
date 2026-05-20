"""
teleop_real_follower.launch.py — Bimanual follower nodes (distributed).

Launches two follower_node instances on this PC, one per UR10e arm,
each subscribing to its side's leader/joint_state topic.

Usage:
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_follower.launch.py
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_follower.launch.py \\
      left_ip:=<LEFT_UR10E_IP> right_ip:=<RIGHT_UR10E_IP> rt:=true

Single-arm fallback: leave one side's IP empty to disable that follower.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_unilateral_vive_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'
    resources = f'{pkg_share}/resources'

    # PC e's UR10e IPs (left/right confirmed 2026-05-20).
    # Override via launch args if your IP plan differs.
    left_ip_arg = DeclareLaunchArgument(
        'left_ip', default_value='169.254.186.93',
        description='Left UR10e IP (empty = disable left follower)')
    right_ip_arg = DeclareLaunchArgument(
        'right_ip', default_value='169.254.186.92',
        description='Right UR10e IP (empty = disable right follower)')

    # Distinct PC-side reverse-port bases so the two UrDrivers don't
    # collide on the same host. UrDriver uses 4 consecutive ports.
    left_port_arg  = DeclareLaunchArgument('left_port_base',  default_value='50011')
    right_port_arg = DeclareLaunchArgument('right_port_base', default_value='50021')

    rt_arg = DeclareLaunchArgument('rt', default_value='false')

    left_enabled = PythonExpression(
        ["'", LaunchConfiguration('left_ip'),  "' != ''"])
    right_enabled = PythonExpression(
        ["'", LaunchConfiguration('right_ip'), "' != ''"])

    follower_left = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='follower_node',
        name='follower_left',
        output='screen',
        condition=IfCondition(left_enabled),
        arguments=[
            '--robot-ip', LaunchConfiguration('left_ip'),
            '--config', config,
            '--resources-dir', resources,
            '--topic-prefix', '/ur10e/left',
            '--port-base', LaunchConfiguration('left_port_base'),
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    follower_right = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='follower_node',
        name='follower_right',
        output='screen',
        condition=IfCondition(right_enabled),
        arguments=[
            '--robot-ip', LaunchConfiguration('right_ip'),
            '--config', config,
            '--resources-dir', resources,
            '--topic-prefix', '/ur10e/right',
            '--port-base', LaunchConfiguration('right_port_base'),
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return LaunchDescription([
        left_ip_arg, right_ip_arg,
        left_port_arg, right_port_arg, rt_arg,
        follower_left, follower_right,
    ])
