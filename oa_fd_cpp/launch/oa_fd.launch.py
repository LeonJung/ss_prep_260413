"""oa_fd.launch.py — single-PC bimanual enactic-style force-feedback bilateral.

  ros2 launch oa_fd_cpp oa_fd.launch.py
  ros2 launch oa_fd_cpp oa_fd.launch.py rt:=true rt_cpu:=2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('oa_fd_cpp')
    default_config = f'{pkg_share}/config/oa_fd.yaml'
    default_urdf = f'{pkg_share}/urdf/openarmx_arm_v2com.urdf'  # shared fallback
    # per-side calibrated gravity models (mass/COM differ slightly L vs R)
    default_urdf_left  = f'{pkg_share}/urdf/openarmx_arm_cali_left_leader.urdf'
    default_urdf_right = f'{pkg_share}/urdf/openarmx_arm_cali_right_leader.urdf'

    config_arg = DeclareLaunchArgument('config', default_value=default_config)
    urdf_arg = DeclareLaunchArgument('urdf', default_value=default_urdf)
    urdf_left_arg  = DeclareLaunchArgument('urdf_left',  default_value=default_urdf_left)
    urdf_right_arg = DeclareLaunchArgument('urdf_right', default_value=default_urdf_right)
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
            '--urdf-left', LaunchConfiguration('urdf_left'),
            '--urdf-right', LaunchConfiguration('urdf_right'),
            '--arms', LaunchConfiguration('arms'),
            '--role', LaunchConfiguration('role'),
            '--rt-mode', LaunchConfiguration('rt'),
            '--rt-priority', LaunchConfiguration('rt_priority'),
            '--rt-cpu', LaunchConfiguration('rt_cpu'),
        ],
    )

    return LaunchDescription([config_arg, urdf_arg, urdf_left_arg, urdf_right_arg,
                              arms_arg, role_arg, rt_arg,
                              rt_prio_arg, rt_cpu_arg, node])
