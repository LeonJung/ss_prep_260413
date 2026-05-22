"""
teleop_real_leader.launch.py — Bimanual Vive-tracker leader (distributed).

Single process drives both trackers (left + right) and publishes to two
topic namespaces. Leave one side's serial empty for single-arm mode.

Calibration YAMLs are auto-discovered from
  <pkg-share>/config/calibration_<side>.yaml
unless overridden via the left_calib / right_calib launch args.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _build(context, *args, **kwargs):
    pkg_share = get_package_share_directory('ur10e_teleop_unilateral_vive_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'

    # Auto-discover: ~/.ros first (survives builds), then pkg_share (legacy).
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
            '--rate-hz', LaunchConfiguration('rate_hz'),
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return [leader]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='ur10e',
            description='UR model used as the IK target (ur10e matches actual follower).'),
        # PC e's paired Vive trackers, left/right confirmed by hand-swing test.
        # Base stations: LHB-45131F3B, LHB-BB0267D2.
        DeclareLaunchArgument('left_serial',  default_value='LHR-B4BFDF90',
            description='Left tracker serial (empty = disable)'),
        DeclareLaunchArgument('right_serial', default_value='LHR-C21814A6',
            description='Right tracker serial (empty = disable)'),
        # Explicit override; empty triggers auto-discover.
        DeclareLaunchArgument('left_calib',  default_value=''),
        DeclareLaunchArgument('right_calib', default_value=''),
        DeclareLaunchArgument('rate_hz', default_value='500.0'),
        DeclareLaunchArgument('rt', default_value='false'),
        OpaqueFunction(function=_build),
    ])
