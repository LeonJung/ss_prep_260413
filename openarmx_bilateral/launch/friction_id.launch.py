#!/usr/bin/env python3
# Phase 3 step 1: follower friction identification data collection (LEFT).
#
# Brings up everything needed to excite the follower joints at constant speeds:
#   * follower effort controller + gravity comp (via follower_gravity.launch)
#   * follower velocity controller (drives the joints)
#   * follower kp:=0  (no position hold -> joint free to be velocity-driven;
#                      kd kept at HW default so velocity is tracked)
#   * friction_id_node (shuttles one joint at a time through a speed list)
#
# You run the two PLAIN bringups first (leader optional/idle), put the follower
# LEFT arm in a roughly mid-range collision-free pose, then:
#   ros2 launch openarmx_bilateral friction_id.launch.py
# When it logs "friction ID DONE", Ctrl+C and send /tmp/friction_id.csv.
#
# DO NOT run the relay or move the leader during this.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('openarmx_bilateral')
    vel_params = os.path.join(pkg, 'config', 'left_velocity_controller.yaml')
    urdf = LaunchConfiguration('urdf_path')
    g_scale = LaunchConfiguration('g_scale')
    fparam = '/follower/openarmx_left_hardware_params'

    follower_grav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'follower_gravity.launch.py')),
        launch_arguments={'urdf_path': urdf, 'g_scale': g_scale}.items(),
    )
    follower_vel = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['left_forward_velocity_controller',
                   '-c', '/follower/controller_manager',
                   '-t', 'forward_command_controller/ForwardCommandController',
                   '-p', vel_params, '--unload-on-kill'],
    )
    # follower kp -> 0 (no position hold), all 7 joints in one shot
    set_kp0 = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['bash', '-lc',
             'for j in $(seq 1 7); do ros2 param set %s kp_joint$j 0.0; done; '
             'echo "[friction_id] follower kp=0 set on %s"' % (fparam, fparam)],
        output='screen')])
    # friction_id_node after controllers + kp=0 are in place
    idnode = TimerAction(period=6.0, actions=[Node(
        package='openarmx_bilateral', executable='friction_id_node',
        name='openarmx_friction_id', output='screen',
        parameters=[{
            'csv': LaunchConfiguration('csv'),
            'range': LaunchConfiguration('range'),
            'dwell': LaunchConfiguration('dwell'),
        }],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        DeclareLaunchArgument('csv', default_value='/tmp/friction_id.csv'),
        DeclareLaunchArgument('range', default_value='0.45'),   # shuttle half-range [rad]
        DeclareLaunchArgument('dwell', default_value='5.0'),    # s per speed level
        follower_grav,
        follower_vel,
        set_kp0,
        idnode,
    ])
