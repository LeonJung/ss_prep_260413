"""
teleop_real.launch.py — Bimanual Vive leader + two UR10e followers, single PC.

Spawns:
  - vive_leader_node       (left + right trackers via OpenVR)
  - follower_left          (subscribes /ur10e/left/leader/joint_state)
  - follower_right         (subscribes /ur10e/right/leader/joint_state)

Calibration YAMLs are auto-discovered at
  <pkg-share>/config/calibration_left.yaml
  <pkg-share>/config/calibration_right.yaml
if present. Otherwise the leader auto-tares on the first ACTIVE poll.

Single-arm fallback: leave one side's tracker serial or robot IP empty
to skip that arm.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _build(context, *args, **kwargs):
    pkg_share = get_package_share_directory('ur10e_teleop_unilateral_vive_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'
    resources = f'{pkg_share}/resources'

    # Auto-discover calibration files if no explicit override was given.
    # Look at the user calibration dir FIRST (~/.ros/...) because that's
    # where vive_calibrate writes by default — and unlike pkg_share/config
    # it doesn't get clobbered by `colcon build`. Falls back to pkg_share
    # for legacy setups where the YAMLs were saved into the install tree.
    user_calib_dir = os.path.expanduser(
        '~/.ros/ur10e_teleop_unilateral_vive_cpp')

    def discover_calib(arg_name, side):
        explicit = LaunchConfiguration(arg_name).perform(context).strip()
        if explicit:
            return explicit
        for cand in (f'{user_calib_dir}/calibration_{side}.yaml',
                     f'{pkg_share}/config/calibration_{side}.yaml'):
            if os.path.exists(cand):
                return cand
        return ''

    left_calib = discover_calib('left_calib', 'left')
    right_calib = discover_calib('right_calib', 'right')

    leader = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='vive_leader_node',
        name='vive_leader_node',
        output='screen',
        arguments=[
            '--robot', LaunchConfiguration('robot'),
            '--config', config,
            '--left-serial', LaunchConfiguration('left_serial'),
            '--left-calib', left_calib,
            '--right-serial', LaunchConfiguration('right_serial'),
            '--right-calib', right_calib,
            '--rt-mode', LaunchConfiguration('leader_rt'),
        ],
    )

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

    return [leader, follower_left, follower_right]


def generate_launch_description():
    return LaunchDescription([
        # leader IK robot model. ur10e matches the actual follower so the
        # published JointState is directly UR10e joint space.
        DeclareLaunchArgument('robot', default_value='ur10e'),

        # PC e's paired Vive trackers (left/right confirmed by hand-swing test).
        # Base stations: LHB-45131F3B, LHB-BB0267D2.
        DeclareLaunchArgument('left_serial',  default_value='LHR-B4BFDF90',
            description='Left tracker serial (empty = disable left)'),
        DeclareLaunchArgument('right_serial', default_value='LHR-C21814A6',
            description='Right tracker serial (empty = disable right)'),

        # Explicit calib overrides; empty triggers auto-discover.
        DeclareLaunchArgument('left_calib',  default_value=''),
        DeclareLaunchArgument('right_calib', default_value=''),

        # UR10e IPs on this PC (left/right confirmed 2026-05-20).
        DeclareLaunchArgument('left_ip',  default_value='169.254.186.93',
            description='Left UR10e IP (empty = disable left follower)'),
        DeclareLaunchArgument('right_ip', default_value='169.254.186.92',
            description='Right UR10e IP (empty = disable right follower)'),

        DeclareLaunchArgument('left_port_base',  default_value='50011'),
        DeclareLaunchArgument('right_port_base', default_value='50021'),

        DeclareLaunchArgument('leader_rt',   default_value='false'),
        DeclareLaunchArgument('follower_rt', default_value='false'),

        OpaqueFunction(function=_build),
    ])
