"""
teleop_single.launch.py — One arm only (left OR right).

Usage:
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_single.launch.py side:=left
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_single.launch.py side:=right
                                            [rt:=true] [calib:=PATH]

Brings up:
  - vive_leader_node       (only the requested side's tracker active)
  - follower_node          (only the requested side's UR10e connected)

Calibration: by default, loads <pkg-share>/config/calibration_<side>.yaml
if that file exists; otherwise empty (the leader auto-tares on first
ACTIVE poll). Override with calib:=PATH for ad-hoc files.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# Lab-bound hardware. Mirrors the defaults in teleop_real_leader.launch.py.
SERIAL_BY_SIDE = {
    'left':  'LHR-B4BFDF90',
    'right': 'LHR-C21814A6',
}
IP_BY_SIDE = {
    'left':  '169.254.186.93',
    'right': '169.254.186.92',
}
PORT_BASE_BY_SIDE = {
    'left':  '50011',
    'right': '50021',
}


def _build(context, *args, **kwargs):
    side = LaunchConfiguration('side').perform(context).strip().lower()
    if side not in ('left', 'right'):
        raise RuntimeError(
            f"side must be 'left' or 'right' (got '{side}')")

    pkg_share = get_package_share_directory('ur10e_teleop_unilateral_vive_cpp')
    config_path = f'{pkg_share}/config/real_ur.yaml'
    resources = f'{pkg_share}/resources'

    serial = SERIAL_BY_SIDE[side]
    ur_ip  = IP_BY_SIDE[side]
    port_base = PORT_BASE_BY_SIDE[side]

    # Calibration: explicit override > auto-discovered file > empty (auto-tare).
    calib = LaunchConfiguration('calib').perform(context).strip()
    if not calib:
        candidate = f'{pkg_share}/config/calibration_{side}.yaml'
        calib = candidate if os.path.exists(candidate) else ''

    leader_args = [
        '--config', config_path,
        # Other side stays empty so the leader skips it.
        f'--{side}-serial', serial,
        f'--{side}-calib',  calib,
        '--rt-mode', LaunchConfiguration('rt'),
    ]

    leader = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='vive_leader_node',
        name=f'vive_leader_{side}',
        output='screen',
        arguments=leader_args,
    )

    follower = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='follower_node',
        name=f'follower_{side}',
        output='screen',
        arguments=[
            '--robot-ip', ur_ip,
            '--config', config_path,
            '--resources-dir', resources,
            '--topic-prefix', f'/ur10e/{side}',
            '--port-base', port_base,
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return [leader, follower]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left',
            description='Arm to bring up: left | right'),
        DeclareLaunchArgument('rt', default_value='false',
            description='SCHED_FIFO + mlockall on both nodes'),
        DeclareLaunchArgument('calib', default_value='',
            description='Override calibration YAML; empty = auto-discover '
                        '<pkg-share>/config/calibration_<side>.yaml'),
        OpaqueFunction(function=_build),
    ])
