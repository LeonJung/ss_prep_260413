#!/usr/bin/env python3
# oa_bilateral cross-relay (LEFT arm default).
#
#   bilateral:=false (default) -> UNILATERAL (step b): follower-left tracks
#                                  leader-left. Leader = gravity-float (run its
#                                  bringup with gravity_comp + kp lowered).
#   bilateral:=true            -> add follower->leader coupling (force feedback);
#                                  raise leader kp a little (soft) for the feel.
#
# Topics default to /leader and /follower namespaces + left_forward_position_
# controller. Override to match your two bringups. arm_side default = left_arm.
#
#   ros2 launch oa_bilateral relay.launch.py                 # unilateral left
#   ros2 launch oa_bilateral relay.launch.py bilateral:=true # bilateral left

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('arm_side', default_value='left_arm'),
        DeclareLaunchArgument('bilateral', default_value='false'),
        DeclareLaunchArgument('couple_sign', default_value='-1.0'),
        DeclareLaunchArgument('rate_hz', default_value='200'),
        DeclareLaunchArgument('leader_states', default_value='/leader/joint_states'),
        DeclareLaunchArgument('follower_states', default_value='/follower/joint_states'),
        DeclareLaunchArgument('leader_cmd',
            default_value='/leader/left_forward_position_controller/commands'),
        DeclareLaunchArgument('follower_cmd',
            default_value='/follower/left_forward_position_controller/commands'),
    ]
    node = Node(
        package='oa_bilateral', executable='relay_node',
        name='oa_bilateral_relay', output='screen',
        parameters=[{
            'arm_side': LaunchConfiguration('arm_side'),
            'bilateral': LaunchConfiguration('bilateral'),
            'couple_sign': LaunchConfiguration('couple_sign'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'leader_states': LaunchConfiguration('leader_states'),
            'follower_states': LaunchConfiguration('follower_states'),
            'leader_cmd': LaunchConfiguration('leader_cmd'),
            'follower_cmd': LaunchConfiguration('follower_cmd'),
        }],
    )
    return LaunchDescription(args + [node])
