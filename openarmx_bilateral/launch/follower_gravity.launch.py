#!/usr/bin/env python3
# Follower gravity compensation, self-contained, per-side (LEFT or RIGHT).
#
# The official gravity_comp_node uses ABSOLUTE topics, so a follower bringup with
# enable_forward_effort:=true would pollute the LEADER. So we launch the follower
# bringup PLAIN and run a gravity_comp_node REMAPPED onto /follower here, plus spawn
# the follower's effort controller (the namespaced controllers yaml declares none,
# so we pass -t/-p with config/<side>_effort_controller.yaml).
#
#   ros2 launch openarmx_bilateral follower_gravity.launch.py            # left
#   ros2 launch openarmx_bilateral follower_gravity.launch.py side:=right

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    side = LaunchConfiguration('side').perform(context).lower()
    side = 'right' if side.startswith('r') else 'left'
    ctrl = f'{side}_forward_effort_controller'
    eff_params = os.path.join(get_package_share_directory('openarmx_bilateral'),
                              'config', f'{side}_effort_controller.yaml')
    effort_topic = LaunchConfiguration('effort_topic').perform(context) \
        or f'/follower/{ctrl}/commands'
    urdf = LaunchConfiguration('urdf_path')
    g_scale = LaunchConfiguration('g_scale')
    cm = LaunchConfiguration('cm')
    return [
        # spawn the follower's <side> effort controller (type + params explicit)
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[ctrl, '-c', cm,
                        '-t', 'forward_command_controller/ForwardCommandController',
                        '-p', eff_params, '--unload-on-kill']),
        # gravity_comp REMAPPED onto /follower (this side only), after controller up
        TimerAction(period=3.0, actions=[Node(
            package='openarmx_gravity_comp', executable='gravity_comp_node',
            name=f'follower_gravity_comp_{side}', output='screen',
            parameters=[{'urdf_path': urdf, 'g_scale': g_scale,
                         'enable_left': side == 'left',
                         'enable_right': side == 'right'}],
            remappings=[('/joint_states', '/follower/joint_states'),
                        (f'/{ctrl}/commands', effort_topic)])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left'),   # left | right
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        DeclareLaunchArgument('cm', default_value='/follower/controller_manager'),
        # '' => /follower/<side>_forward_effort_controller/commands (friction_id uses this);
        # bilateral.launch routes it to /follower/grav_only_<side>.
        DeclareLaunchArgument('effort_topic', default_value=''),
        OpaqueFunction(function=setup),
    ])
