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
# Gains = a SCALE on a per-joint base profile (shoulder kp 50 / wrist kp 10,
# openarmx-proven shape so the low-inertia wrist isn't over-gained -> no 팡팡).
# SAFETY: start leader_gain=follower_gain=0 (weightless parity), then raise
# follower_gain (e.g. 0.6-1.0; 1.0 = openarmx follower) to get tracking & VERIFY
# couple_sign (track same direction = ok; opposite/runaway => couple_sign:=1.0),
# then add leader_gain (tiny, e.g. 0.1-0.3) for force feedback.
#
#   ros2 launch oa_mit oa_mit_bilateral_dual_threaded.launch.py \
#        arm_side:=right_arm follower_gain:=1.0 leader_gain:=0.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('arm_side', default_value='right_arm',
                              description='right_arm (can0/can2) or left_arm (can1/can3)'),
        DeclareLaunchArgument('leader_can', default_value='auto'),    # auto = from arm_side
        DeclareLaunchArgument('follower_can', default_value='auto'),
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('control_rate_hz', default_value='250'),
        DeclareLaunchArgument('g_scale', default_value='0.9'),
        DeclareLaunchArgument('gy_override', default_value='0.0'),  # 0=auto from arm_side
        DeclareLaunchArgument('leader_gain', default_value='0.0',
                              description='scale on per-joint kp profile (0=weightless; force-fb e.g. 0.3)'),
        DeclareLaunchArgument('follower_gain', default_value='0.0',
                              description='scale on per-joint kp profile (1.0=openarmx follower; tracking)'),
        DeclareLaunchArgument('leader_kd_scale', default_value='1.0',
                              description='extra damping on leader (kd only; raise vs vibration)'),
        DeclareLaunchArgument('follower_kd_scale', default_value='1.0',
                              description='extra damping on follower (kd only; raise vs vibration)'),
        DeclareLaunchArgument('vel_ff', default_value='false',
                              description='inject peer velocity (off=smooth, like openarmx follower)'),
        DeclareLaunchArgument('couple_sign', default_value='-1.0'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('enable_leader', default_value='true'),
        DeclareLaunchArgument('enable_follower', default_value='true'),
    ]
    node = Node(
        package='oa_mit', executable='oa_mit_bilateral_dual_threaded_node',
        name='oa_mit_bilateral_dual_threaded', output='screen',
        parameters=[{
            'arm_side': LaunchConfiguration('arm_side'),
            'leader_can': LaunchConfiguration('leader_can'),
            'follower_can': LaunchConfiguration('follower_can'),
            'urdf_path': LaunchConfiguration('urdf_path'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'g_scale': LaunchConfiguration('g_scale'),
            'gy_override': LaunchConfiguration('gy_override'),
            'leader_gain': LaunchConfiguration('leader_gain'),
            'follower_gain': LaunchConfiguration('follower_gain'),
            'leader_kd_scale': LaunchConfiguration('leader_kd_scale'),
            'follower_kd_scale': LaunchConfiguration('follower_kd_scale'),
            'vel_ff': LaunchConfiguration('vel_ff'),
            'couple_sign': LaunchConfiguration('couple_sign'),
            'verbose': LaunchConfiguration('verbose'),
            'enable_leader': LaunchConfiguration('enable_leader'),
            'enable_follower': LaunchConfiguration('enable_follower'),
        }],
    )
    return LaunchDescription(args + [node])
