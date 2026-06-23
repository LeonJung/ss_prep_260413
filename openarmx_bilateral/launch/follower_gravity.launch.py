#!/usr/bin/env python3
# Phase-1: follower gravity compensation (LEFT), self-contained.
#
# The official gravity_comp_node uses ABSOLUTE topics (/joint_states,
# /left_forward_effort_controller/commands), so a follower bringup launched with
# enable_forward_effort:=true spawns a gravity node that publishes to ROOT (the
# LEADER's effort topic) and reads ROOT /joint_states (the LEADER's states) —
# i.e. it POLLUTES the leader and gives the follower NO gravity. So:
#   * launch the FOLLOWER bringup WITHOUT enable_forward_effort (no polluting node)
#   * this launch spawns the follower's effort controller AND a gravity_comp_node
#     REMAPPED onto /follower (left arm only).
#
# ON  = run this launch.   OFF = don't.
#   ros2 launch openarmx_bilateral follower_gravity.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        DeclareLaunchArgument('cm', default_value='/follower/controller_manager'),
        # 1) spawn the follower's left effort controller (so gravity torque applies)
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['left_forward_effort_controller', '-c',
                       LaunchConfiguration('cm')],
        ),
        # 2) gravity_comp_node REMAPPED onto /follower (left only), after the
        #    controller is up
        TimerAction(period=3.0, actions=[Node(
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
        )]),
    ])
