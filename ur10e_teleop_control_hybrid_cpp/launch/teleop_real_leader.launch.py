"""
teleop_real_leader.launch.py — C++ leader only (for distributed setup).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_teleop_control_hybrid_cpp')
    config = f'{pkg_share}/config/real_ur.yaml'
    resources = f'{pkg_share}/resources'

    robot_arg = DeclareLaunchArgument('robot', default_value='ur3e')
    leader_ip_arg = DeclareLaunchArgument('leader_ip',
        default_value='169.254.186.94')
    rt_arg = DeclareLaunchArgument('rt', default_value='false')

    leader = Node(
        package='ur10e_teleop_control_hybrid_cpp',
        executable='leader_node',
        name='leader_real_node',
        output='screen',
        arguments=[
            '--robot', LaunchConfiguration('robot'),
            '--robot-ip', LaunchConfiguration('leader_ip'),
            '--config', config,
            '--resources-dir', resources,
            '--rt-mode', LaunchConfiguration('rt'),
        ],
    )

    return LaunchDescription([robot_arg, leader_ip_arg, rt_arg, leader])
