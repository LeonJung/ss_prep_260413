"""oa_fd.launch.py — single-PC bimanual enactic-style force-feedback bilateral.

Per-arm config: one self-contained yaml per arm
  config/oa_fd_leader_left.yaml   config/oa_fd_leader_right.yaml
  config/oa_fd_follower_left.yaml config/oa_fd_follower_right.yaml
Each carries that arm's CAN iface, gravity (urdf/vec/mirror/scale), friction,
Kp/Kd, home, torque_limit, freedrive shaping, joint_limits. The gravity URDF
defaults to the per-arm cali file (overridable via urdf_* args).

  ros2 launch oa_fd_cpp oa_fd.launch.py
  ros2 launch oa_fd_cpp oa_fd.launch.py arms:=left role:=both rt:=true rt_cpu:=2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    s = get_package_share_directory('oa_fd_cpp')

    args = [
        DeclareLaunchArgument('config_leader_left',    default_value=f'{s}/config/oa_fd_leader_left.yaml'),
        DeclareLaunchArgument('config_leader_right',   default_value=f'{s}/config/oa_fd_leader_right.yaml'),
        DeclareLaunchArgument('config_follower_left',  default_value=f'{s}/config/oa_fd_follower_left.yaml'),
        DeclareLaunchArgument('config_follower_right', default_value=f'{s}/config/oa_fd_follower_right.yaml'),
        DeclareLaunchArgument('urdf_leader_left',    default_value=f'{s}/urdf/openarmx_arm_cali_left_leader.urdf'),
        DeclareLaunchArgument('urdf_leader_right',   default_value=f'{s}/urdf/openarmx_arm_cali_right_leader.urdf'),
        DeclareLaunchArgument('urdf_follower_left',  default_value=f'{s}/urdf/openarmx_arm_cali_left_follower.urdf'),
        DeclareLaunchArgument('urdf_follower_right', default_value=f'{s}/urdf/openarmx_arm_cali_right_follower.urdf'),
        DeclareLaunchArgument('arms', default_value='both',
                              description='which pair(s) to drive: right | left | both'),
        DeclareLaunchArgument('role', default_value='both',
                              description='leader | follower | both (single role -> gravity-only)'),
        DeclareLaunchArgument('rt', default_value='false'),
        DeclareLaunchArgument('rt_priority', default_value='80'),
        DeclareLaunchArgument('rt_cpu', default_value='-1'),
    ]

    node = Node(
        package='oa_fd_cpp', executable='oa_fd_node', name='oa_fd_node', output='screen',
        arguments=[
            '--config-leader-left',    LaunchConfiguration('config_leader_left'),
            '--config-leader-right',   LaunchConfiguration('config_leader_right'),
            '--config-follower-left',  LaunchConfiguration('config_follower_left'),
            '--config-follower-right', LaunchConfiguration('config_follower_right'),
            '--urdf-leader-left',      LaunchConfiguration('urdf_leader_left'),
            '--urdf-leader-right',     LaunchConfiguration('urdf_leader_right'),
            '--urdf-follower-left',    LaunchConfiguration('urdf_follower_left'),
            '--urdf-follower-right',   LaunchConfiguration('urdf_follower_right'),
            '--arms', LaunchConfiguration('arms'),
            '--role', LaunchConfiguration('role'),
            '--rt-mode', LaunchConfiguration('rt'),
            '--rt-priority', LaunchConfiguration('rt_priority'),
            '--rt-cpu', LaunchConfiguration('rt_cpu'),
        ],
    )
    return LaunchDescription(args + [node])
