"""
teleop_rtde_dummy.launch.py — Launch with RTDE client + ur_server_dummy.

Starts ur_server_dummy.py (×2, different ports for leader/follower),
then leader/follower_real_node.py with --client rtde.

Usage:
  ros2 launch ur10e_teleop_real teleop_rtde_dummy.launch.py
  ros2 launch ur10e_teleop_real teleop_rtde_dummy.launch.py robot:=ur3e
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_real')
    pkg_lib = pkg_share.replace('/share/', '/lib/')
    src_dir = f'{pkg_share}/src'
    config = f'{pkg_share}/config/dummy.yaml'

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='ur10e',
        description='Leader robot type (ur10e or ur3e)')

    # Two dummy servers on different ports (leader: 30004, follower: 30005)
    server_leader = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{src_dir}/ur_server_dummy.py',
            '--port', '30004', '--robot', LaunchConfiguration('robot'),
        ],
        name='ur_server_dummy_leader',
        output='screen',
    )

    server_follower = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{src_dir}/ur_server_dummy.py',
            '--port', '30005', '--robot', 'ur10e',
        ],
        name='ur_server_dummy_follower',
        output='screen',
    )

    leader = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/leader_real_node.py',
            '--robot', LaunchConfiguration('robot'),
            '--config', config,
            '--client', 'rtde',
            '--robot-ip', '127.0.0.1',
        ],
        name='leader_real_node',
        output='screen',
    )

    follower = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/follower_real_node.py',
            '--config', config,
            '--client', 'rtde',
            '--robot-ip', '127.0.0.1',
        ],
        name='follower_real_node',
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        server_leader,
        server_follower,
        leader,
        follower,
    ])
