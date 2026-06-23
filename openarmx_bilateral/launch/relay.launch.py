#!/usr/bin/env python3
# openarmx_bilateral cross-relay (LEFT arm default).
#
# Runs the relay AND auto-sets the leader arm's MIT gains (so you never type the
# `ros2 param set kp_joint..` loop):
#   - unilateral (default): leader_kp=leader_kd=0  -> leader free (gravity-float)
#   - bilateral (bilateral:=true): set leader_kp soft (e.g. 15) for force feedback
#
#   bilateral:=false (default) -> follower-left tracks leader-left (step b).
#   bilateral:=true            -> also follower->leader (force feedback).
#
# LEADER is expected NON-namespaced (so the official gravity_comp reaches it);
# FOLLOWER under /follower. Override topics if your bringup namespacing differs.
#
#   ros2 launch openarmx_bilateral relay.launch.py                  # unilateral left
#   ros2 launch openarmx_bilateral relay.launch.py bilateral:=true leader_kp:=15

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, TimerAction,
                            ExecuteProcess)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    side = LaunchConfiguration('arm_side').perform(context)
    sp = 'right' if side == 'right_arm' else 'left'

    relay = Node(
        package='openarmx_bilateral', executable='relay_node',
        name='openarmx_bilateral_relay', output='screen',
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
    actions = [relay]

    if LaunchConfiguration('set_leader_gains').perform(context).lower() == 'true':
        kp = LaunchConfiguration('leader_kp').perform(context)
        kd = LaunchConfiguration('leader_kd').perform(context)
        node = LaunchConfiguration('leader_param_node').perform(context)
        if not node:
            node = '/openarmx_%s_hardware_params' % sp
        n = int(LaunchConfiguration('n_joints').perform(context))
        cmd = ('for j in $(seq 1 %d); do '
               'ros2 param set %s kp_joint$j %s; '
               'ros2 param set %s kd_joint$j %s; done; '
               'echo "[openarmx_bilateral] leader gains set: kp=%s kd=%s on %s"'
               % (n, node, kp, node, kd, kp, kd, node))
        # delay so the leader HW param node is up (bring up the robots first)
        actions.append(TimerAction(period=3.0, actions=[
            ExecuteProcess(cmd=['bash', '-lc', cmd], output='screen')]))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('arm_side', default_value='left_arm'),
        DeclareLaunchArgument('bilateral', default_value='false'),
        DeclareLaunchArgument('couple_sign', default_value='-1.0'),
        DeclareLaunchArgument('rate_hz', default_value='200'),
        # leader gains auto-set (no manual param-set loop). unilateral: 0/0.
        DeclareLaunchArgument('set_leader_gains', default_value='true'),
        DeclareLaunchArgument('leader_kp', default_value='0.0'),   # bilateral: ~15 (soft)
        DeclareLaunchArgument('leader_kd', default_value='0.0'),
        DeclareLaunchArgument('leader_param_node', default_value=''),  # '' => /openarmx_<side>_hardware_params
        DeclareLaunchArgument('n_joints', default_value='7'),
        # leader NON-namespaced (gravity_comp reaches it); follower /follower
        DeclareLaunchArgument('leader_states', default_value='/joint_states'),
        DeclareLaunchArgument('follower_states', default_value='/follower/joint_states'),
        DeclareLaunchArgument('leader_cmd',
            default_value='/left_forward_position_controller/commands'),
        DeclareLaunchArgument('follower_cmd',
            default_value='/follower/left_forward_position_controller/commands'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
