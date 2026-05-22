"""
teleop_real_hand.launch.py — Bimanual Vive teleop with hand-attached EE.

Same as teleop_real.launch.py but tool_mode defaults to "hand" so the
user-facing EE is at the dg5f gripper palm (instead of the UR flange).
Rotating the Vive about its own center then sweeps the flange around
the palm — multiple joints coordinate — instead of a pure q5 spin.

Everything else (serials, IPs, ports, calibrations) inherits from the
package defaults the same way teleop_real does. Override any of them
on the command line:
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_hand.launch.py \\
      leader_rt:=true follower_rt:=true
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

    left_calib  = discover_calib('left_calib',  'left')
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
            '--tool-mode', 'hand',
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
        DeclareLaunchArgument('robot', default_value='ur10e'),
        DeclareLaunchArgument('left_serial',  default_value='LHR-B4BFDF90'),
        DeclareLaunchArgument('right_serial', default_value='LHR-C21814A6'),
        DeclareLaunchArgument('left_calib',  default_value=''),
        DeclareLaunchArgument('right_calib', default_value=''),
        DeclareLaunchArgument('left_ip',  default_value='169.254.186.93'),
        DeclareLaunchArgument('right_ip', default_value='169.254.186.92'),
        DeclareLaunchArgument('left_port_base',  default_value='50011'),
        DeclareLaunchArgument('right_port_base', default_value='50021'),
        DeclareLaunchArgument('leader_rt',   default_value='false'),
        DeclareLaunchArgument('follower_rt', default_value='false'),
        OpaqueFunction(function=_build),
    ])
