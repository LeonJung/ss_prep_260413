"""oa_fd.launch.py — single-PC bimanual enactic-style force-feedback bilateral.

  ros2 launch oa_fd_cpp oa_fd.launch.py
  ros2 launch oa_fd_cpp oa_fd.launch.py rt:=true rt_cpu:=2
  # calibrated leader model (handle tip), stock follower model (gripper tip):
  ros2 launch oa_fd_cpp oa_fd.launch.py arms:=left role:=leader \
      urdf_leader:=$HOME/git_ws/ss_prep_260413/oa_fd_cpp/urdf/openarmx_arm_cali_left_leader.urdf
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('oa_fd_cpp')
    default_config = f'{pkg_share}/config/oa_fd.yaml'
    default_urdf = f'{pkg_share}/urdf/openarmx_arm_v2com.urdf'  # enactic masses + COMs(links2-5); v2mass / openarmx_arm.urdf via urdf:=

    config_arg = DeclareLaunchArgument('config', default_value=default_config)
    urdf_arg = DeclareLaunchArgument('urdf', default_value=default_urdf)
    # per-role URDFs (leader = handle tip, follower = gripper tip). Empty ->
    # falls back to `urdf`. Point urdf_leader at the calibrated leader URDF.
    urdf_leader_arg = DeclareLaunchArgument('urdf_leader', default_value='')
    urdf_follower_arg = DeclareLaunchArgument('urdf_follower', default_value='')
    arms_arg = DeclareLaunchArgument(
        'arms', default_value='both',
        description='which pair(s) to drive: right | left | both')
    role_arg = DeclareLaunchArgument(
        'role', default_value='both',
        description='which side of each pair: leader | follower | both '
                    '(single role disables ACTIVE coupling -> gravity-only)')
    rt_arg = DeclareLaunchArgument('rt', default_value='false')
    rt_prio_arg = DeclareLaunchArgument('rt_priority', default_value='80')
    rt_cpu_arg = DeclareLaunchArgument('rt_cpu', default_value='-1')

    node = Node(
        package='oa_fd_cpp',
        executable='oa_fd_node',
        name='oa_fd_node',
        output='screen',
        arguments=[
            '--config', LaunchConfiguration('config'),
            '--urdf', LaunchConfiguration('urdf'),
            '--urdf-leader', LaunchConfiguration('urdf_leader'),
            '--urdf-follower', LaunchConfiguration('urdf_follower'),
            '--arms', LaunchConfiguration('arms'),
            '--role', LaunchConfiguration('role'),
            '--rt-mode', LaunchConfiguration('rt'),
            '--rt-priority', LaunchConfiguration('rt_priority'),
            '--rt-cpu', LaunchConfiguration('rt_cpu'),
        ],
    )

    return LaunchDescription([config_arg, urdf_arg, urdf_leader_arg,
                              urdf_follower_arg, arms_arg, role_arg, rt_arg,
                              rt_prio_arg, rt_cpu_arg, node])
