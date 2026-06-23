#!/usr/bin/env python3
# openarmx_bilateral ONE-SHOT launch (LEFT arm). Run this single file and you get
# the whole teleop stack with the defaults we always use -- no arguments to type:
#   * relay  : leader-left -> follower-left (unilateral by default)
#   * leader : MIT kp=kd=0 (free / gravity-float) + gravity comp
#   * follower: effort controller + gravity comp (remapped onto /follower)
#
# You still bring up the two robots yourself, PLAIN (no enable_forward_effort) --
# this launch owns ALL gravity comp + effort controllers so nothing double-spawns:
#   [leader  bringup]  non-namespaced
#   [follower bringup] namespace:=follower
#   ros2 launch openarmx_bilateral bilateral.launch.py
#
# Phase 2+ (force feedback): ros2 launch ... bilateral.launch.py bilateral:=true leader_kp:=15
#
# Defaults baked in (override only if needed):
#   arm_side=left_arm  bilateral=false  leader_kp=0  leader_kd=0
#   urdf_path=/tmp/v10_bimanual.urdf  g_scale=1.05

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('openarmx_bilateral')
    eff_params = os.path.join(pkg, 'config', 'left_effort_controller.yaml')
    urdf = LaunchConfiguration('urdf_path')
    g_scale = LaunchConfiguration('g_scale')

    # --- relay + auto leader kp/kd (defaults 0/0) ---
    relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'relay.launch.py')),
        launch_arguments={
            'arm_side': LaunchConfiguration('arm_side'),
            'bilateral': LaunchConfiguration('bilateral'),
            'leader_kp': LaunchConfiguration('leader_kp'),
            'leader_kd': LaunchConfiguration('leader_kd'),
        }.items(),
    )

    # --- follower effort controller + gravity comp (already remapped onto /follower) ---
    follower_grav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'follower_gravity.launch.py')),
        launch_arguments={'urdf_path': urdf, 'g_scale': g_scale}.items(),
    )

    # --- leader effort controller (root CM) ---
    leader_eff = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['left_forward_effort_controller',
                   '-c', '/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', eff_params],
    )
    # --- leader gravity comp (root topics = leader, no remap) ---
    leader_grav = TimerAction(period=3.0, actions=[Node(
        package='openarmx_gravity_comp', executable='gravity_comp_node',
        name='leader_gravity_comp', output='screen',
        parameters=[{
            'urdf_path': urdf, 'g_scale': g_scale,
            'enable_left': True, 'enable_right': False,
        }],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('arm_side', default_value='left_arm'),
        DeclareLaunchArgument('bilateral', default_value='false'),
        DeclareLaunchArgument('leader_kp', default_value='0.0'),
        DeclareLaunchArgument('leader_kd', default_value='0.0'),
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        relay,
        leader_eff,
        leader_grav,
        follower_grav,
    ])
