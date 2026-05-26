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

Run one arm only with side:= (like teleop_single):
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_hand.launch.py side:=left
  ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_hand.launch.py side:=right
side:=both (default) brings up both arms.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _build(context, *args, **kwargs):
    pkg_share = get_package_share_directory('ur10e_teleop_unilateral_vive_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'
    resources = f'{pkg_share}/resources'

    side = LaunchConfiguration('side').perform(context).strip().lower()
    if side not in ('both', 'left', 'right'):
        raise RuntimeError(
            f"side must be 'both', 'left', or 'right' (got '{side}')")

    # A side is active when 'side' allows it AND its IP isn't blanked out.
    left_ip  = LaunchConfiguration('left_ip').perform(context).strip()
    right_ip = LaunchConfiguration('right_ip').perform(context).strip()
    left_active  = side in ('both', 'left')  and left_ip != ''
    right_active = side in ('both', 'right') and right_ip != ''

    # Blank a deselected side's serial so the leader skips that tracker.
    left_serial  = (LaunchConfiguration('left_serial').perform(context).strip()
                    if left_active else '')
    right_serial = (LaunchConfiguration('right_serial').perform(context).strip()
                    if right_active else '')

    user_calib_dir = os.path.expanduser(
        '~/.ros/ur10e_teleop_unilateral_vive_cpp')

    def discover_calib(arg_name, side_name, active):
        if not active:
            return ''
        explicit = LaunchConfiguration(arg_name).perform(context).strip()
        if explicit:
            return explicit
        for cand in (f'{user_calib_dir}/calibration_{side_name}.yaml',
                     f'{pkg_share}/config/calibration_{side_name}.yaml'):
            if os.path.exists(cand):
                return cand
        return ''

    left_calib  = discover_calib('left_calib',  'left',  left_active)
    right_calib = discover_calib('right_calib', 'right', right_active)

    leader = Node(
        package='ur10e_teleop_unilateral_vive_cpp',
        executable='vive_leader_node',
        name='vive_leader_node',
        output='screen',
        arguments=[
            '--robot', LaunchConfiguration('robot'),
            '--config', config,
            '--left-serial', left_serial,
            '--left-calib', left_calib,
            '--right-serial', right_serial,
            '--right-calib', right_calib,
            '--tool-mode', 'hand',
            '--rt-mode', LaunchConfiguration('leader_rt'),
        ],
    )

    nodes = [leader]

    if left_active:
        nodes.append(Node(
            package='ur10e_teleop_unilateral_vive_cpp',
            executable='follower_node',
            name='follower_left',
            output='screen',
            arguments=[
                '--robot-ip', left_ip,
                '--config', config,
                '--resources-dir', resources,
                '--topic-prefix', '/ur10e/left',
                '--port-base', LaunchConfiguration('left_port_base'),
                '--rt-mode', LaunchConfiguration('follower_rt'),
            ],
        ))

    if right_active:
        nodes.append(Node(
            package='ur10e_teleop_unilateral_vive_cpp',
            executable='follower_node',
            name='follower_right',
            output='screen',
            arguments=[
                '--robot-ip', right_ip,
                '--config', config,
                '--resources-dir', resources,
                '--topic-prefix', '/ur10e/right',
                '--port-base', LaunchConfiguration('right_port_base'),
                '--rt-mode', LaunchConfiguration('follower_rt'),
            ],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='both',
            description='Arms to bring up: both | left | right'),
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
