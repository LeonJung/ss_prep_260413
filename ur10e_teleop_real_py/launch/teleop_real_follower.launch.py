"""
teleop_real_follower.launch.py — Launch ONLY the follower (UR10e) node.

For distributed setups where leader and follower run on separate PCs.
Run this on the PC that has network access to the UR10e.

Both PCs must share ROS_DOMAIN_ID and have DDS discovery working.

Usage:
  ros2 launch ur10e_teleop_real_py teleop_real_follower.launch.py
  ros2 launch ur10e_teleop_real_py teleop_real_follower.launch.py follower_ip:=<IP>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_real_py')
    pkg_lib = pkg_share.replace('/share/', '/lib/')
    config = f'{pkg_share}/config/real_ur.yaml'

    follower_ip_arg = DeclareLaunchArgument(
        'follower_ip', default_value='169.254.186.92',
        description='Follower (UR10e) IP')

    follower = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/follower_real_node.py',
            '--client', 'rtde',
            '--robot-ip', LaunchConfiguration('follower_ip'),
            '--config', config,
        ],
        name='follower_real_node',
        output='screen',
    )

    return LaunchDescription([
        follower_ip_arg,
        follower,
    ])
