"""
teleop_real_follower.launch.py — C++ follower only (for distributed setup).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_control_ff_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'
    resources = f'{pkg_share}/resources'

    follower_ip_arg = DeclareLaunchArgument('follower_ip',
        default_value='169.254.186.92')
    rt_arg = DeclareLaunchArgument('rt', default_value='false')

    follower = Node(
        package='ur10e_teleop_control_ff_cpp',
        executable='follower_node',
        name='follower_real_node',
        output='screen',
        arguments=[
            '--robot-ip', LaunchConfiguration('follower_ip'),
            '--config', config,
            '--resources-dir', resources,
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return LaunchDescription([follower_ip_arg, rt_arg, follower])
