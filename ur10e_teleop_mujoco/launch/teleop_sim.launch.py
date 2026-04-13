"""
teleop_sim.launch.py — Launch leader_sim_node + follower_sim_node (ROS2).

Usage:
  ros2 launch ur10e_teleop_mujoco teleop_sim.launch.py
  ros2 launch ur10e_teleop_mujoco teleop_sim.launch.py robot:=ur3e
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='ur10e',
        description='Leader robot type (ur10e or ur3e)')

    pkg_lib = get_package_share_directory('ur10e_teleop_mujoco').replace(
        '/share/', '/lib/')

    leader = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/leader_sim_node.py',
            '--robot', LaunchConfiguration('robot'),
        ],
        name='leader_sim_node',
        output='screen',
    )

    follower = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'), '-u',
            f'{pkg_lib}/follower_sim_node.py',
        ],
        name='follower_sim_node',
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        leader,
        follower,
    ])
