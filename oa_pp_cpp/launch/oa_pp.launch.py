"""oa_pp.launch.py — single-PC bimanual position-position bilateral.

  ros2 launch oa_pp_cpp oa_pp.launch.py
  ros2 launch oa_pp_cpp oa_pp.launch.py rt:=true rt_cpu:=2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('oa_pp_cpp')
    default_config = f'{pkg_share}/config/oa_pp.yaml'

    config_arg = DeclareLaunchArgument('config', default_value=default_config)
    rt_arg = DeclareLaunchArgument('rt', default_value='false')
    rt_prio_arg = DeclareLaunchArgument('rt_priority', default_value='80')
    rt_cpu_arg = DeclareLaunchArgument('rt_cpu', default_value='-1')

    node = Node(
        package='oa_pp_cpp',
        executable='oa_pp_node',
        name='oa_pp_node',
        output='screen',
        arguments=[
            '--config', LaunchConfiguration('config'),
            '--rt-mode', LaunchConfiguration('rt'),
            '--rt-priority', LaunchConfiguration('rt_priority'),
            '--rt-cpu', LaunchConfiguration('rt_cpu'),
        ],
    )

    return LaunchDescription([
        config_arg, rt_arg, rt_prio_arg, rt_cpu_arg, node,
    ])
