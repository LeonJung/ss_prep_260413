#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# oa_mit M1 — single-process dual-arm bilateral (one PC, both arms direct CAN,
# in-memory state exchange, NO ROS topic loop). Replicates the proven enactic /
# ur10e_teleop bilateral architecture on the openarmx MIT layer.
#
# Prereq: leader URDF at /tmp/v10_bimanual.urdf
#   xacro <ws>/src/openarmx_description/urdf/robot/v10.urdf.xacro \
#         arm_type:=v10 bimanual:=true > /tmp/v10_bimanual.urdf
# Do NOT run the follower bringup — THIS node owns both arms' CAN directly.
#
# Right pair: leader=can0, follower=can2.  Left pair: leader=can1, follower=can3.
#
# SAFETY: start with leader_kp/follower_kp = 0 (weightless parity), then raise
# follower_kp first (e.g. 30), then leader_kp tiny (e.g. 3) to VERIFY couple_sign
# (arms pull together = correct; push apart = set couple_sign:=1.0), then tune up.
#
#   ros2 launch oa_mit oa_mit_bilateral_dual.launch.py \
#        arm_side:=right_arm leader_can:=can0 follower_can:=can2 \
#        follower_kp:=30.0 leader_kp:=3.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('arm_side', default_value='right_arm'),
        DeclareLaunchArgument('leader_can', default_value='can0'),
        DeclareLaunchArgument('follower_can', default_value='can2'),
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('control_rate_hz', default_value='250'),
        DeclareLaunchArgument('g_scale', default_value='0.9'),
        DeclareLaunchArgument('gy_override', default_value='0.0'),  # 0=auto from arm_side
        DeclareLaunchArgument('leader_kp', default_value='0.0'),
        DeclareLaunchArgument('leader_kd', default_value='1.0'),
        DeclareLaunchArgument('follower_kp', default_value='0.0'),
        DeclareLaunchArgument('follower_kd', default_value='1.5'),
        DeclareLaunchArgument('couple_sign', default_value='-1.0'),
        DeclareLaunchArgument('verbose', default_value='true'),
    ]
    node = Node(
        package='oa_mit', executable='oa_mit_bilateral_dual_node',
        name='oa_mit_bilateral_dual', output='screen',
        parameters=[{
            'arm_side': LaunchConfiguration('arm_side'),
            'leader_can': LaunchConfiguration('leader_can'),
            'follower_can': LaunchConfiguration('follower_can'),
            'urdf_path': LaunchConfiguration('urdf_path'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'g_scale': LaunchConfiguration('g_scale'),
            'gy_override': LaunchConfiguration('gy_override'),
            'leader_kp': LaunchConfiguration('leader_kp'),
            'leader_kd': LaunchConfiguration('leader_kd'),
            'follower_kp': LaunchConfiguration('follower_kp'),
            'follower_kd': LaunchConfiguration('follower_kd'),
            'couple_sign': LaunchConfiguration('couple_sign'),
            'verbose': LaunchConfiguration('verbose'),
        }],
    )
    return LaunchDescription(args + [node])
