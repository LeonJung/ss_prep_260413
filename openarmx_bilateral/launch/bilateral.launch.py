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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Phase 3 identified Coulomb friction (LEFT arm), per robot. tau_fric = Fc*tanh(0.1*k*w)
LEADER_FC   = [1.21, 0.63, 0.31, 0.39, 0.14, 0.17, 0.13]
LEADER_K    = [81.0, 51.0, 40.0, 52.0, 36.0, 20.0, 48.0]
FOLLOWER_FC = [1.04, 1.06, 0.35, 0.36, 0.16, 0.15, 0.12]
FOLLOWER_K  = [68.0, 60.0, 26.0, 87.0, 62.0, 57.0, 39.0]


def generate_launch_description():
    pkg = get_package_share_directory('openarmx_bilateral')
    eff_params = os.path.join(pkg, 'config', 'left_effort_controller.yaml')
    vel_params = os.path.join(pkg, 'config', 'left_velocity_controller.yaml')
    urdf = LaunchConfiguration('urdf_path')
    g_scale = LaunchConfiguration('g_scale')
    vel_ff = LaunchConfiguration('vel_ff')
    fric_en = ParameterValue(LaunchConfiguration('friction'), value_type=bool)
    fric_scale = ParameterValue(LaunchConfiguration('friction_scale'), value_type=float)

    # --- relay + auto leader/follower kp/kd (set all joints in one shot) ---
    relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'relay.launch.py')),
        launch_arguments={
            'arm_side': LaunchConfiguration('arm_side'),
            'bilateral': LaunchConfiguration('bilateral'),
            'vel_ff': vel_ff,
            'leader_kp': LaunchConfiguration('leader_kp'),
            'leader_kd': LaunchConfiguration('leader_kd'),
            'follower_kp': LaunchConfiguration('follower_kp'),
            'follower_kd': LaunchConfiguration('follower_kd'),
        }.items(),
    )

    # --- follower velocity controller (Phase 2): only when vel_ff:=true.
    #     Claims the VELOCITY interface only, so it runs alongside the position
    #     controller -> MIT command gets both {pos, vel}. ---
    follower_vel = Node(
        package='controller_manager', executable='spawner', output='screen',
        condition=IfCondition(vel_ff),
        arguments=['left_forward_velocity_controller',
                   '-c', '/follower/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', vel_params, '--unload-on-kill'],
    )

    # --- follower effort controller + gravity comp. Gravity is routed to
    #     /follower/grav_only; the follower friction_comp_node adds friction and
    #     publishes to the effort controller. (friction:=false => passthrough.) ---
    follower_grav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'follower_gravity.launch.py')),
        launch_arguments={'urdf_path': urdf, 'g_scale': g_scale,
                          'effort_topic': '/follower/grav_only'}.items(),
    )
    follower_fric = Node(
        package='openarmx_bilateral', executable='friction_comp_node',
        name='follower_friction_comp', output='screen',
        parameters=[{
            'grav_in': '/follower/grav_only',
            'states': '/follower/joint_states',
            'effort_out': '/follower/left_forward_effort_controller/commands',
            'fc': FOLLOWER_FC, 'k': FOLLOWER_K,
            'enable': fric_en, 'scale': fric_scale,
        }],
    )

    # --- leader effort controller (root CM) ---
    leader_eff = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['left_forward_effort_controller',
                   '-c', '/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', eff_params, '--unload-on-kill'],
    )
    # --- leader gravity comp (root topics = leader). Gravity -> /grav_only;
    #     leader friction_comp_node adds friction -> effort controller. ---
    leader_grav = TimerAction(period=3.0, actions=[Node(
        package='openarmx_gravity_comp', executable='gravity_comp_node',
        name='leader_gravity_comp', output='screen',
        parameters=[{
            'urdf_path': urdf, 'g_scale': g_scale,
            'enable_left': True, 'enable_right': False,
        }],
        remappings=[('/left_forward_effort_controller/commands', '/grav_only')],
    )])
    leader_fric = Node(
        package='openarmx_bilateral', executable='friction_comp_node',
        name='leader_friction_comp', output='screen',
        parameters=[{
            'grav_in': '/grav_only',
            'states': '/joint_states',
            'effort_out': '/left_forward_effort_controller/commands',
            'fc': LEADER_FC, 'k': LEADER_K,
            'enable': fric_en, 'scale': fric_scale,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_side', default_value='left_arm'),
        DeclareLaunchArgument('bilateral', default_value='false'),
        DeclareLaunchArgument('vel_ff', default_value='false'),  # Phase 2 velocity FF A/B
        DeclareLaunchArgument('friction', default_value='false'),  # Phase 3 friction comp A/B
        DeclareLaunchArgument('friction_scale', default_value='0.7'),
        DeclareLaunchArgument('leader_kp', default_value='0.0'),
        DeclareLaunchArgument('leader_kd', default_value='0.0'),
        DeclareLaunchArgument('follower_kp', default_value=''),  # '' => keep HW defaults
        DeclareLaunchArgument('follower_kd', default_value=''),
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        relay,
        leader_eff,
        leader_grav,
        leader_fric,
        follower_grav,
        follower_fric,
        follower_vel,
    ])
