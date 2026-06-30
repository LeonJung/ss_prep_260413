#!/usr/bin/env python3
# Phase 3 step 1: friction identification data collection (LEFT).
#
#   target:=follower (default) -> follower (/follower namespace)   [WORKING path]
#   target:=leader             -> leader  (root controller_manager + root topics)
#
# Same friction_id_node + per-joint limits for both; only the namespace/topics,
# controller_manager, HW param node and gravity-comp remap differ. Friction is a
# per-robot property, so run BOTH (separate CSVs).
#
# Run the relevant bringup PLAIN, put its LEFT arm in a mid-range pose, then:
#   ros2 launch openarmx_bilateral friction_id.launch.py                       # follower
#   ros2 launch openarmx_bilateral friction_id.launch.py target:=leader \
#        csv:=/tmp/friction_id_leader.csv                                      # leader
# When it logs "friction ID DONE", Ctrl+C and send the CSV.
# DO NOT run the relay or move the arm by hand during this.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# friction-test-only per-joint ABSOLUTE limits [rad] (NOT control limits), LEFT.
#   J1 [-135,45] J2 [-135,0] J3 [-45,90] J4 [0,100] J5 [-45,90] J6 [0,45] J7 [-90,80] (deg)
LEFT_LO = [-2.356, -2.356, -0.785, 0.0, -0.785, 0.0, -1.571]
LEFT_HI = [0.785, 0.0, 1.571, 1.745, 1.571, 0.785, 1.396]


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('openarmx_bilateral')
    eff_params = os.path.join(pkg, 'config', 'left_effort_controller.yaml')
    vel_params = os.path.join(pkg, 'config', 'left_velocity_controller.yaml')
    urdf = LaunchConfiguration('urdf_path')
    g_scale = LaunchConfiguration('g_scale')
    leader = LaunchConfiguration('target').perform(context).lower().startswith('lead')

    node_params = {
        'csv': ParameterValue(LaunchConfiguration('csv'), value_type=str),
        'range': ParameterValue(LaunchConfiguration('range'), value_type=float),
        'dwell': ParameterValue(LaunchConfiguration('dwell'), value_type=float),
        'margin': ParameterValue(LaunchConfiguration('margin'), value_type=float),
        # force the full schedule from the launch (overrides the node default, so a
        # stale binary whose default mis-compiled can't drop joints 1,2)
        'joints': [1, 2, 3, 4, 5, 6, 7],
        'n_joints': 7,
        'joint_lo': LEFT_LO,
        'joint_hi': LEFT_HI,
    }

    if not leader:
        # ---------- FOLLOWER (working path, unchanged) ----------
        cm = '/follower/controller_manager'
        pnode = '/follower/openarmx_left_hardware_params'
        grav_actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'follower_gravity.launch.py')),
            launch_arguments={'urdf_path': urdf, 'g_scale': g_scale}.items())]
        # node uses its default follower topics
    else:
        # ---------- LEADER (mirror; root CM + root topics) ----------
        cm = '/controller_manager'
        pnode = '/openarmx_left_hardware_params'
        grav_actions = [
            Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['left_forward_effort_controller', '-c', cm,
                            '-t', 'forward_command_controller/ForwardCommandController',
                            '-p', eff_params, '--unload-on-kill']),
            TimerAction(period=3.0, actions=[Node(
                package='openarmx_gravity_comp', executable='gravity_comp_node',
                name='leader_gravity_comp', output='screen',
                parameters=[{'urdf_path': urdf, 'g_scale': g_scale,
                             'enable_left': True, 'enable_right': False}])]),
        ]
        node_params.update({
            'vel_cmd': '/left_forward_velocity_controller/commands',
            'states': '/joint_states',
            'grav': '/left_forward_effort_controller/commands',
        })

    vel = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['left_forward_velocity_controller', '-c', cm,
                          '-t', 'forward_command_controller/ForwardCommandController',
                          '-p', vel_params, '--unload-on-kill'])
    # kp -> 0 (no position hold), all 7 joints in one shot
    set_kp0 = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['bash', '-lc',
             'for j in $(seq 1 7); do ros2 param set %s kp_joint$j 0.0; done; '
             'echo "[friction_id] %s kp=0 set on %s"'
             % (pnode, 'leader' if leader else 'follower', pnode)],
        output='screen')])
    idnode = TimerAction(period=6.0, actions=[Node(
        package='openarmx_bilateral', executable='friction_id_node',
        name='openarmx_friction_id', output='screen', parameters=[node_params])])

    return grav_actions + [vel, set_kp0, idnode]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target', default_value='follower'),  # follower | leader
        DeclareLaunchArgument('urdf_path', default_value='/tmp/v10_bimanual.urdf'),
        DeclareLaunchArgument('g_scale', default_value='0.97'),  # match bilateral
        DeclareLaunchArgument('csv', default_value='/tmp/friction_id.csv'),
        DeclareLaunchArgument('range', default_value='0.45'),   # fallback half-range [rad]
        DeclareLaunchArgument('dwell', default_value='5.0'),    # s per speed level
        DeclareLaunchArgument('margin', default_value='0.087'), # cushion inside limits [rad] ~5deg
        OpaqueFunction(function=launch_setup),
    ])
