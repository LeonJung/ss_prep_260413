"""
teleop_real.launch.py — Bimanual Vive leader + two UR10e followers, single PC.

Spawns:
  - vive_leader_node       (left + right trackers via OpenVR)
  - follower_left          (subscribes /ur10e/left/leader/joint_state)
  - follower_right         (subscribes /ur10e/right/leader/joint_state)

Single-arm fallback: leave one side's tracker serial or robot IP empty
to skip that arm.
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

    robot_arg = DeclareLaunchArgument('robot', default_value='ur3e')

    # PC e's paired Vive trackers (left/right confirmed by hand-swing test).
    # Base stations: LHB-45131F3B, LHB-BB0267D2.
    left_serial_arg = DeclareLaunchArgument(
        'left_serial', default_value='LHR-B4BFDF90',
        description='Left tracker serial (empty = disable left)')
    left_calib_arg = DeclareLaunchArgument(
        'left_calib', default_value='',
        description='Left tracker→UR-base YAML (empty = auto-tare)')

    right_serial_arg = DeclareLaunchArgument(
        'right_serial', default_value='LHR-C21814A6',
        description='Right tracker serial (empty = disable right)')
    right_calib_arg = DeclareLaunchArgument(
        'right_calib', default_value='',
        description='Right tracker→UR-base YAML (empty = auto-tare)')

    # UR10e IPs on this PC (left/right confirmed 2026-05-20).
    # Leave a side empty to skip its follower.
    left_ip_arg = DeclareLaunchArgument(
        'left_ip', default_value='169.254.186.93',
        description='Left UR10e IP (empty = disable left follower)')
    right_ip_arg = DeclareLaunchArgument(
        'right_ip', default_value='169.254.186.92',
        description='Right UR10e IP (empty = disable right follower)')

    # Distinct PC-side reverse-port bases so the two UrDrivers don't
    # collide.
    left_port_arg  = DeclareLaunchArgument('left_port_base',  default_value='50011')
    right_port_arg = DeclareLaunchArgument('right_port_base', default_value='50021')

    leader_rt_arg = DeclareLaunchArgument('leader_rt', default_value='false')
    follower_rt_arg = DeclareLaunchArgument('follower_rt', default_value='false')

    left_enabled = PythonExpression(
        ["'", LaunchConfiguration('left_ip'),  "' != ''"])
    right_enabled = PythonExpression(
        ["'", LaunchConfiguration('right_ip'), "' != ''"])

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
            '--rt-mode', LaunchConfiguration('follower_rt'),
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
            '--rt-mode', LaunchConfiguration('follower_rt'),
        ],
    )

    return LaunchDescription([
        robot_arg,
        left_serial_arg, left_calib_arg,
        right_serial_arg, right_calib_arg,
        left_ip_arg, right_ip_arg,
        left_port_arg, right_port_arg,
        leader_rt_arg, follower_rt_arg,
        leader, follower_left, follower_right,
    ])
