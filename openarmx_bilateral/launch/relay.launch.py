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

    # topics: empty arg => derive from side (so arm_side=right_arm hits RIGHT topics).
    def topic(name, default):
        v = LaunchConfiguration(name).perform(context)
        return v if v else default
    leader_states  = topic('leader_states',  '/joint_states')
    follower_states = topic('follower_states', '/follower/joint_states')
    leader_cmd     = topic('leader_cmd',     '/%s_forward_position_controller/commands' % sp)
    follower_cmd   = topic('follower_cmd',   '/follower/%s_forward_position_controller/commands' % sp)

    relay = Node(
        package='openarmx_bilateral', executable='relay_node',
        name='openarmx_bilateral_relay_%s' % sp, output='screen',   # side-specific (both-arm safe)
        parameters=[{
            'arm_side': LaunchConfiguration('arm_side'),
            'bilateral': LaunchConfiguration('bilateral'),
            'vel_ff': LaunchConfiguration('vel_ff'),
            'couple_sign': LaunchConfiguration('couple_sign'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'leader_states': leader_states,
            'follower_states': follower_states,
            'leader_cmd': leader_cmd,
            'follower_cmd': follower_cmd,
        }],
    )
    actions = [relay]

    n = int(LaunchConfiguration('n_joints').perform(context))

    def gain_loop(label, node, kp, kd):
        # set kp_joint1..n and kd_joint1..n in ONE shot (no hand-tuning per joint)
        cmd = ('for j in $(seq 1 %d); do '
               'ros2 param set %s kp_joint$j %s; '
               'ros2 param set %s kd_joint$j %s; done; '
               'echo "[openarmx_bilateral] %s gains set: kp=%s kd=%s on %s"'
               % (n, node, kp, node, kd, label, kp, kd, node))
        # delay so the HW param node is up (bring up the robots first)
        return TimerAction(period=3.0, actions=[
            ExecuteProcess(cmd=['bash', '-lc', cmd], output='screen')])

    if LaunchConfiguration('set_leader_gains').perform(context).lower() == 'true':
        node = LaunchConfiguration('leader_param_node').perform(context) \
            or '/openarmx_%s_hardware_params' % sp
        actions.append(gain_loop('leader', node,
                                 LaunchConfiguration('leader_kp').perform(context),
                                 LaunchConfiguration('leader_kd').perform(context)))
        # leader gripper (joint8) gains — separate from the 7 arm joints
        gkp = LaunchConfiguration('leader_gripper_kp').perform(context)
        gkd = LaunchConfiguration('leader_gripper_kd').perform(context)
        if gkp != '' and gkd != '':
            gcmd = ('ros2 param set %s kp_joint8 %s; ros2 param set %s kd_joint8 %s; '
                    'echo "[openarmx_bilateral] leader gripper(j8) kp=%s kd=%s on %s"'
                    % (node, gkp, node, gkd, gkp, gkd, node))
            actions.append(TimerAction(period=3.0, actions=[
                ExecuteProcess(cmd=['bash', '-lc', gcmd], output='screen')]))

    # follower gains: only touched if BOTH kp and kd given (else keep HW defaults)
    fkp = LaunchConfiguration('follower_kp').perform(context)
    fkd = LaunchConfiguration('follower_kd').perform(context)
    if fkp != '' and fkd != '':
        node = LaunchConfiguration('follower_param_node').perform(context) \
            or '/follower/openarmx_%s_hardware_params' % sp
        actions.append(gain_loop('follower', node, fkp, fkd))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('arm_side', default_value='left_arm'),
        DeclareLaunchArgument('bilateral', default_value='false'),
        DeclareLaunchArgument('vel_ff', default_value='false'),     # Phase 2 velocity FF
        DeclareLaunchArgument('couple_sign', default_value='1.0'),  # +1 verified on HW
        DeclareLaunchArgument('rate_hz', default_value='200'),
        # leader gains auto-set (no manual param-set loop). unilateral: 0/0.
        DeclareLaunchArgument('set_leader_gains', default_value='true'),
        DeclareLaunchArgument('leader_kp', default_value='0.0'),   # bilateral: ~15 (soft)
        DeclareLaunchArgument('leader_kd', default_value='0.0'),
        DeclareLaunchArgument('leader_param_node', default_value=''),  # '' => /openarmx_<side>_hardware_params
        # leader gripper (joint8) gains (operator request: 5.0/0.5). '' => skip.
        DeclareLaunchArgument('leader_gripper_kp', default_value='5.0'),
        DeclareLaunchArgument('leader_gripper_kd', default_value='0.5'),
        # follower gains: '' = keep HW defaults; set BOTH to override all joints at once
        DeclareLaunchArgument('follower_kp', default_value=''),
        DeclareLaunchArgument('follower_kd', default_value=''),
        DeclareLaunchArgument('follower_param_node', default_value=''),  # '' => /follower/openarmx_<side>_hardware_params
        DeclareLaunchArgument('n_joints', default_value='7'),
        # topics: '' => derived from arm_side (leader /joint_states, follower
        # /follower/joint_states, cmd /<side>_forward_position_controller/commands).
        DeclareLaunchArgument('leader_states', default_value=''),
        DeclareLaunchArgument('follower_states', default_value=''),
        DeclareLaunchArgument('leader_cmd', default_value=''),
        DeclareLaunchArgument('follower_cmd', default_value=''),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
