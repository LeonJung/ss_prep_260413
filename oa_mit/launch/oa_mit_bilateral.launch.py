#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# oa_mit bilateral force-feedback teleop (built on the openarmx MIT layer).
#
# Mirrors openarmx_teleop_bimanual/teleop_bimanual_with_gravitycomp.launch.py
# (weightless gravity-comp leader on can0/can1 -> follower forward_position_
# controller) and ADDS the force-reflection coupling param `couple_kp`.
#
#   couple_kp = 0.0  -> identical to the proven Mode-2 weightless behavior.
#   couple_kp > 0    -> leader feels follower lag (force feedback). Start LOW
#                       (e.g. 5-10) and raise; too high re-introduces tremor.
#
# Prereqs (same as Mode 2):
#   1) /tmp/v10_bimanual.urdf exists:
#        xacro .../openarmx_description/urdf/robot/v10.urdf.xacro \
#              arm_type:=v10 bimanual:=true > /tmp/v10_bimanual.urdf
#   2) follower bringup running (can2/can3, forward_position_controller):
#        ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
#             right_can_interface:=can2 left_can_interface:=can3 \
#             control_mode:=mit robot_controller:=forward_position_controller
#
#   ros2 launch oa_mit oa_mit_bilateral.launch.py couple_kp:=8.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('leader_urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('control_rate_hz', default_value='300'),
        DeclareLaunchArgument('couple_kp', default_value='0.0',
                              description='force-reflection gain (0=Mode2 weightless; raise to add FF)'),
        DeclareLaunchArgument('couple_kd', default_value='0.0'),
        DeclareLaunchArgument('couple_tau_limit', default_value='8.0'),
        DeclareLaunchArgument('couple_lpf', default_value='1.0',
                              description='EMA on follower pos (1=off; 0.3-0.5 smooths 74Hz steps vs vibration)'),
        DeclareLaunchArgument('right_g_scale', default_value='0.9'),
        DeclareLaunchArgument('left_g_scale', default_value='0.8'),
        DeclareLaunchArgument('kp_hold', default_value='0.0'),
        DeclareLaunchArgument('kd_damp', default_value='0.0'),
        DeclareLaunchArgument('verbose', default_value='false'),
    ]

    common = dict(
        leader_urdf_path=LaunchConfiguration('leader_urdf_path'),
        control_rate_hz=LaunchConfiguration('control_rate_hz'),
        couple_kp=LaunchConfiguration('couple_kp'),
        couple_kd=LaunchConfiguration('couple_kd'),
        couple_tau_limit=LaunchConfiguration('couple_tau_limit'),
        couple_lpf=LaunchConfiguration('couple_lpf'),
        kp_hold=LaunchConfiguration('kp_hold'),
        kd_damp=LaunchConfiguration('kd_damp'),
        verbose=LaunchConfiguration('verbose'),
    )

    right = Node(
        package='oa_mit', executable='oa_mit_bilateral_node',
        name='oa_mit_bilateral_right', output='screen',
        parameters=[dict(common,
            arm_side='right_arm', leader_can='can0', follower_prefix='right',
            g_scale=LaunchConfiguration('right_g_scale'), gdir=[0.0, -9.81, 0.0])],
    )
    left = Node(
        package='oa_mit', executable='oa_mit_bilateral_node',
        name='oa_mit_bilateral_left', output='screen',
        parameters=[dict(common,
            arm_side='left_arm', leader_can='can1', follower_prefix='left',
            g_scale=LaunchConfiguration('left_g_scale'), gdir=[0.0, 9.81, 0.0])],
    )

    return LaunchDescription(args + [right, left])
