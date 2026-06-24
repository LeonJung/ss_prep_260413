#!/usr/bin/env python3
# Phase 3 step 1: LEADER friction identification data collection (LEFT).
#
# Same as friction_id.launch.py but for the LEADER, which is NON-namespaced
# (root controller_manager + root topics). Friction is a per-robot property, so
# leader and follower are identified separately (both get friction comp later).
#
# Setup: leader effort controller + gravity comp (root), leader velocity
# controller (drives the joints), leader kp:=0 (no position hold), friction_id_node.
#
# Run the LEADER bringup PLAIN first, put its LEFT arm in a mid-range pose, then:
#   ros2 launch openarmx_bilateral friction_id_leader.launch.py
# When it logs "friction ID DONE", Ctrl+C and send /tmp/friction_id_leader.csv.
# DO NOT hold/move the leader by hand during this.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('openarmx_bilateral')
    eff_params = os.path.join(pkg, 'config', 'left_effort_controller.yaml')
    vel_params = os.path.join(pkg, 'config', 'left_velocity_controller.yaml')
    urdf = LaunchConfiguration('urdf_path')
    g_scale = LaunchConfiguration('g_scale')
    lparam = '/openarmx_left_hardware_params'

    # friction-test-only per-joint ABSOLUTE limits [rad] (NOT control limits), LEFT
    left_lo = [-2.356, -2.356, -0.785, 0.0, -0.785, 0.0, -1.571]
    left_hi = [0.785, 0.0, 1.571, 1.745, 1.571, 0.785, 1.396]

    leader_eff = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['left_forward_effort_controller', '-c', '/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', eff_params, '--unload-on-kill'])
    leader_vel = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['left_forward_velocity_controller', '-c', '/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', vel_params, '--unload-on-kill'])
    leader_grav = TimerAction(period=3.0, actions=[Node(
        package='openarmx_gravity_comp', executable='gravity_comp_node',
        name='leader_gravity_comp', output='screen',
        parameters=[{'urdf_path': urdf, 'g_scale': g_scale,
                     'enable_left': True, 'enable_right': False}])])
    set_kp0 = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['bash', '-lc',
             'for j in $(seq 1 7); do ros2 param set %s kp_joint$j 0.0; done; '
             'echo "[friction_id] leader kp=0 set on %s"' % (lparam, lparam)],
        output='screen')])
    idnode = TimerAction(period=6.0, actions=[Node(
        package='openarmx_bilateral', executable='friction_id_node',
        name='openarmx_friction_id', output='screen',
        parameters=[{
            'csv': ParameterValue(LaunchConfiguration('csv'), value_type=str),
            'range': ParameterValue(LaunchConfiguration('range'), value_type=float),
            'dwell': ParameterValue(LaunchConfiguration('dwell'), value_type=float),
            'margin': ParameterValue(LaunchConfiguration('margin'), value_type=float),
            'vel_cmd': '/left_forward_velocity_controller/commands',
            'states': '/joint_states',
            'grav': '/left_forward_effort_controller/commands',
            'joint_lo': left_lo,
            'joint_hi': left_hi,
        }])])

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        DeclareLaunchArgument('csv', default_value='/tmp/friction_id_leader.csv'),
        DeclareLaunchArgument('range', default_value='0.45'),
        DeclareLaunchArgument('dwell', default_value='5.0'),
        DeclareLaunchArgument('margin', default_value='0.087'),
        leader_eff,
        leader_vel,
        leader_grav,
        set_kp0,
        idnode,
    ])
