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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    # follower kp -> 0 (no position hold) and kd -> <kd> (so the velocity
    # controller has torque authority to drive even the high-friction shoulders),
    # all 7 joints in one shot. kd does NOT bias friction = effort - gravity.
    pre = ('for j in $(seq 1 7); do ros2 param set %s kp_joint$j 0.0; '
           'ros2 param set %s kd_joint$j ' % (fparam, fparam))
    post = '; done; echo "[friction_id] follower kp=0 kd set on %s"' % fparam
    set_gains = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['bash', '-lc', [TextSubstitution(text=pre),
                             LaunchConfiguration('kd'), TextSubstitution(text=post)]],
        output='screen')])
    # friction-test-only per-joint ABSOLUTE limits [rad] (NOT control limits).
    # From the operator's measured safe ranges. LEFT arm (this launch tests left):
    #   J1 [-135, 45]  J2 [-135, 0]  J3 [-45, 90]  J4 [0, 100]
    #   J5 [-45, 90]   J6 [0, 45]    J7 [-90, 80]   (deg)
    # RIGHT (for later): lo=[-0.785,0,-1.571,0,-1.571,-0.785,-1.396]
    #                    hi=[2.356,2.356,0.785,1.745,0.785,0,1.571]
    left_lo = [-2.356, -2.356, -0.785, 0.0, -0.785, 0.0, -1.571]
    left_hi = [0.785, 0.0, 1.571, 1.745, 1.571, 0.785, 1.396]

    # friction_id_node after controllers + kp=0 are in place
    idnode = TimerAction(period=6.0, actions=[Node(
        package='openarmx_bilateral', executable='friction_id_node',
        name='openarmx_friction_id', output='screen',
        parameters=[{
            'csv': ParameterValue(LaunchConfiguration('csv'), value_type=str),
            'range': ParameterValue(LaunchConfiguration('range'), value_type=float),
            'dwell': ParameterValue(LaunchConfiguration('dwell'), value_type=float),
            'margin': ParameterValue(LaunchConfiguration('margin'), value_type=float),
            'joint_lo': left_lo,
            'joint_hi': left_hi,
        }],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='1.05'),
        DeclareLaunchArgument('csv', default_value='/tmp/friction_id.csv'),
        DeclareLaunchArgument('range', default_value='0.45'),   # fallback half-range [rad]
        DeclareLaunchArgument('dwell', default_value='5.0'),    # s per speed level
        DeclareLaunchArgument('margin', default_value='0.087'), # cushion inside limits [rad] ~5deg
        DeclareLaunchArgument('kd', default_value='10.0'),      # velocity-loop authority during ID
        follower_grav,
        follower_vel,
        set_gains,
        idnode,
    ])
