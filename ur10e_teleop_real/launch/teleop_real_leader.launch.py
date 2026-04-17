"""
teleop_real_leader.launch.py — Launch ONLY the leader (UR3e) node.

For distributed setups where leader and follower run on separate PCs.
Run this on the PC that has network access to the UR3e.

Both PCs must share ROS_DOMAIN_ID and have DDS discovery working.

Usage:
  ros2 launch ur10e_teleop_real teleop_real_leader.launch.py
  ros2 launch ur10e_teleop_real teleop_real_leader.launch.py leader_ip:=<IP>
  ros2 launch ur10e_teleop_real teleop_real_leader.launch.py robot:=ur10e leader_ip:=<IP>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_real')
    pkg_lib = pkg_share.replace('/share/', '/lib/')
    config = f'{pkg_share}/config/real_ur.yaml'

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='ur3e',
        description='Leader robot type (ur10e or ur3e)')
    leader_ip_arg = DeclareLaunchArgument(
        'leader_ip', default_value='169.254.186.94',
        description='Leader (UR3e) IP')

    leader = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/leader_real_node.py',
            '--robot', LaunchConfiguration('robot'),
            '--client', 'rtde',
            '--robot-ip', LaunchConfiguration('leader_ip'),
            '--config', config,
        ],
        name='leader_real_node',
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        leader_ip_arg,
        leader,
    ])
