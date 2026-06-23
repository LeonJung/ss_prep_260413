#!/usr/bin/env python3
# Phase-1: follower gravity compensation (LEFT) for the A/B test.
# The official gravity_comp_node uses ABSOLUTE topics, so under the /follower
# namespace it can't reach the follower. This launch runs it with topic REMAPS
# so it feeds /follower/left_forward_effort_controller (left arm only).
#
# Prereq: follower bringup launched with enable_forward_effort:=true (so the
# follower's left_forward_effort_controller exists), URDF at /tmp/v10_bimanual.urdf.
#
#   # follower gravity comp ON:
#   ros2 launch openarmx_bilateral follower_gravity.launch.py
#   # OFF: just don't launch this.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        DeclareLaunchArgument('follower_ns', default_value='follower'),
        Node(
            package='openarmx_gravity_comp', executable='gravity_comp_node',
            name='follower_gravity_comp', output='screen',
            parameters=[{
                'urdf_path': LaunchConfiguration('urdf_path'),
                'g_scale': LaunchConfiguration('g_scale'),
                'enable_left': True,
                'enable_right': False,
            }],
            remappings=[
                ('/joint_states', '/follower/joint_states'),
                ('/left_forward_effort_controller/commands',
                 '/follower/left_forward_effort_controller/commands'),
            ],
        ),
    ])
