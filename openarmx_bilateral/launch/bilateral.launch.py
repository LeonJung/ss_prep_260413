#!/usr/bin/env python3
# openarmx_bilateral ONE-SHOT launch. Run two PLAIN bringups (leader non-namespaced,
# follower namespace:=follower), then this. Picks LEFT, RIGHT, or BOTH arm pairs:
#
#   ros2 launch openarmx_bilateral bilateral.launch.py                 # left pair (default)
#   ros2 launch openarmx_bilateral bilateral.launch.py arm:=right      # right pair
#   ros2 launch openarmx_bilateral bilateral.launch.py arm:=both       # both pairs
#   ... bilateral:=true vel_ff:=true friction:=true leader_kp:=60 leader_kd:=0.5
#
# Per side it sets up: relay (leader<->follower) + leader/follower gravity comp +
# effort/velocity controllers + friction comp + auto kp/kd. RIGHT reuses LEFT's
# friction coefficients per joint (operator decision: right Jn fric == left Jn fric).
#
# NOTE (tech_debt.md): BOTH doubles CAN traffic; a single pair already caps ~89Hz on
# USB-CAN, so BOTH may run slower. couple_sign is +1 (HW-verified on LEFT pair);
# RIGHT may need -1 -> verify direction on HW, override with couple_sign:=-1.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Identified Coulomb friction (LEFT arm); reused for RIGHT per joint. tau=Fc*tanh(0.1*k*w)
LEADER_FC   = [1.21, 0.63, 0.31, 0.39, 0.14, 0.17, 0.13]
LEADER_K    = [81.0, 51.0, 40.0, 52.0, 36.0, 20.0, 48.0]
FOLLOWER_FC = [1.04, 1.06, 0.35, 0.36, 0.16, 0.15, 0.12]
FOLLOWER_K  = [68.0, 60.0, 26.0, 87.0, 62.0, 57.0, 39.0]


def build_side(side, pkg, lc):
    """All bilateral nodes for one arm side ('left' or 'right')."""
    eff = os.path.join(pkg, 'config', f'{side}_effort_controller.yaml')
    vel = os.path.join(pkg, 'config', f'{side}_velocity_controller.yaml')
    ec  = f'{side}_forward_effort_controller'
    vc  = f'{side}_forward_velocity_controller'
    jp  = f'openarmx_{side}_joint'
    grav_only_l = f'/grav_only_{side}'                  # leader  (root)
    grav_only_f = f'/follower/grav_only_{side}'          # follower
    eff_l = f'/{ec}/commands'
    eff_f = f'/follower/{ec}/commands'

    relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'relay.launch.py')),
        launch_arguments={
            'arm_side': f'{side}_arm',
            'bilateral': lc['bilateral'], 'vel_ff': lc['vel_ff'],
            'couple_sign': lc['couple_sign'],
            'leader_kp': lc['leader_kp'], 'leader_kd': lc['leader_kd'],
            'follower_kp': lc['follower_kp'], 'follower_kd': lc['follower_kd'],
        }.items())

    # --- follower: gravity -> grav_only_<side>, friction adds -> effort ctrl ---
    follower_grav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'follower_gravity.launch.py')),
        launch_arguments={'side': side, 'urdf_path': lc['urdf'], 'g_scale': lc['g_scale'],
                          'effort_topic': grav_only_f}.items())
    follower_fric = Node(
        package='openarmx_bilateral', executable='friction_comp_node',
        name=f'follower_friction_comp_{side}', output='screen',
        parameters=[{'grav_in': grav_only_f, 'states': '/follower/joint_states',
                     'effort_out': eff_f, 'joint_prefix': jp,
                     'fc': FOLLOWER_FC, 'k': FOLLOWER_K,
                     'enable': lc['fric'], 'scale': lc['fric_scale']}])

    # --- leader (root CM): effort ctrl + gravity -> grav_only_<side> + friction ---
    leader_eff = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[ec, '-c', '/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', eff, '--unload-on-kill'])
    leader_grav = TimerAction(period=3.0, actions=[Node(
        package='openarmx_gravity_comp', executable='gravity_comp_node',
        name=f'leader_gravity_comp_{side}', output='screen',
        parameters=[{'urdf_path': lc['urdf'], 'g_scale': lc['g_scale'],
                     'enable_left': side == 'left', 'enable_right': side == 'right'}],
        remappings=[(eff_l, grav_only_l)])])
    leader_fric = Node(
        package='openarmx_bilateral', executable='friction_comp_node',
        name=f'leader_friction_comp_{side}', output='screen',
        parameters=[{'grav_in': grav_only_l, 'states': '/joint_states',
                     'effort_out': eff_l, 'joint_prefix': jp,
                     'fc': LEADER_FC, 'k': LEADER_K,
                     'enable': lc['fric'], 'scale': lc['fric_scale']}])

    # --- follower velocity controller (Phase 2), only when vel_ff:=true ---
    follower_vel = Node(
        package='controller_manager', executable='spawner', output='screen',
        condition=IfCondition(lc['vel_ff']),
        arguments=[vc, '-c', '/follower/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', vel, '--unload-on-kill'])

    return [relay, leader_eff, leader_grav, leader_fric, follower_grav, follower_fric, follower_vel]


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('openarmx_bilateral')
    lc = {
        'bilateral': LaunchConfiguration('bilateral'),
        'vel_ff': LaunchConfiguration('vel_ff'),
        'couple_sign': LaunchConfiguration('couple_sign'),
        'leader_kp': LaunchConfiguration('leader_kp'), 'leader_kd': LaunchConfiguration('leader_kd'),
        'follower_kp': LaunchConfiguration('follower_kp'), 'follower_kd': LaunchConfiguration('follower_kd'),
        'urdf': LaunchConfiguration('urdf_path'), 'g_scale': LaunchConfiguration('g_scale'),
        'fric': ParameterValue(LaunchConfiguration('friction'), value_type=bool),
        'fric_scale': ParameterValue(LaunchConfiguration('friction_scale'), value_type=float),
    }
    arm = LaunchConfiguration('arm').perform(context).lower()
    sides = ['left', 'right'] if arm.startswith('b') else (['right'] if arm.startswith('r') else ['left'])
    actions = []
    for s in sides:
        actions += build_side(s, pkg, lc)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='left'),       # left | right | both
        DeclareLaunchArgument('bilateral', default_value='false'),
        DeclareLaunchArgument('vel_ff', default_value='false'),
        DeclareLaunchArgument('friction', default_value='false'),
        DeclareLaunchArgument('friction_scale', default_value='0.7'),
        DeclareLaunchArgument('couple_sign', default_value='1.0'),  # +1 verified on LEFT; verify RIGHT
        DeclareLaunchArgument('leader_kp', default_value='0.0'),
        DeclareLaunchArgument('leader_kd', default_value='0.0'),
        DeclareLaunchArgument('follower_kp', default_value=''),
        DeclareLaunchArgument('follower_kd', default_value=''),
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        OpaqueFunction(function=launch_setup),
    ])
