"""oa_viz.launch.py — RViz with BOTH robots' URDF models, driven by live joint states.

  leader robot  at x=0   (frame_prefix leader/)
  follower robot at x=1.5 (frame_prefix follower/)

Live pose comes from oa_fd_cpp's /oa/{leader,follower}_{left,right}/joint_state —
run oa_fd_node (e.g. FREEDRIVE) so states are published, then compare the RViz
model pose against the physical arms to find model/zero/sign mismatches.

  ros2 launch oa_viz oa_viz.launch.py                 # both robots
  ros2 launch oa_viz oa_viz.launch.py robots:=leader  # one robot only
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory('oa_viz')
    urdf_path = os.path.join(share, 'urdf', 'oa_bimanual.urdf')
    with open(urdf_path) as f:
        urdf = f.read()

    which = context.launch_configurations.get('robots', 'both')
    robots = ['leader', 'follower'] if which == 'both' else [which]
    x_offset = {'leader': 0.0, 'follower': 1.5}

    nodes = []
    for r in robots:
        prefix = f'{r}/'
        nodes += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=r, name='rsp', output='screen',
                 parameters=[{'robot_description': urdf,
                              'frame_prefix': prefix}]),
            Node(package='oa_viz', executable='joint_state_bridge',
                 namespace=r, name='bridge', output='screen',
                 parameters=[{'robot': r}]),
            # place the robot in the shared world frame
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'{r}_world_tf',
                 arguments=['--x', str(x_offset[r]), '--frame-id', 'world',
                            '--child-frame-id', f'{prefix}world']),
        ]

    rviz_cfg = os.path.join(share, 'rviz', 'oa_viz.rviz')
    nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                      arguments=['-d', rviz_cfg], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='both',
                              description='leader | follower | both'),
        OpaqueFunction(function=_setup),
    ])
