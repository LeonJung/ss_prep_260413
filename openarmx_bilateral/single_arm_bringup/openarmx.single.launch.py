# Single-arm bringup for OpenArmX (left OR right only) — for isolating single-arm tests.
# Reuses the bimanual URDF (so LEFT/RIGHT joint & controller naming is preserved, keeping
# openarmx_bilateral compatible) but disables the OTHER arm's ros2_control hardware via the
# enable_left/enable_right xacro args, so only ONE arm's CAN is opened and only that side's
# controllers are spawned.
#
#   ros2 launch openarmx_bringup openarmx.single.launch.py arm_side:=right right_can_interface:=can0 \
#        control_mode:=mit robot_controller:=forward_position_controller
#   # follower single arm:  ... arm_prefix:=follower right_can_interface:=can2
#
# Only [P0-P4-equivalent] — NOT tested on hardware; verify controller spawn on the control PC.

import os
import xacro
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _ns(context, arm_prefix):
    p = context.perform_substitution(arm_prefix).strip('/')
    return p


def build(context: LaunchContext, *_, **__):
    side = LaunchConfiguration('arm_side').perform(context)          # left | right
    if side not in ('left', 'right'):
        raise ValueError(f"arm_side must be left|right, got {side}")
    ns = _ns(context, LaunchConfiguration('arm_prefix'))
    cm = f"/{ns}/controller_manager" if ns else "/controller_manager"

    desc_pkg = LaunchConfiguration('description_package').perform(context)
    desc_file = LaunchConfiguration('description_file').perform(context)
    control_mode = LaunchConfiguration('control_mode').perform(context)
    can_fd = LaunchConfiguration('can_fd').perform(context)
    left_can = LaunchConfiguration('left_can_interface').perform(context)
    right_can = LaunchConfiguration('right_can_interface').perform(context)
    robot_controller = LaunchConfiguration('robot_controller').perform(context)

    xacro_path = os.path.join(get_package_share_directory(desc_pkg), "urdf", "robot", desc_file)
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": LaunchConfiguration('arm_type').perform(context),
            "bimanual": "true",                       # keep left/right naming
            "use_fake_hardware": LaunchConfiguration('use_fake_hardware').perform(context),
            "ros2_control": "true",
            "can_fd": can_fd,
            "left_can_interface": left_can,
            "right_can_interface": right_can,
            "control_mode": control_mode,
            "node_namespace": ns,
            # THE single-arm switch: only the selected side's hardware is generated
            "enable_left": "true" if side == "left" else "false",
            "enable_right": "true" if side == "right" else "false",
        }
    ).toprettyxml(indent="  ")
    rd = {"robot_description": robot_description}

    controllers_file = os.path.join(
        get_package_share_directory('openarmx_bringup'), "config", "v10_controllers",
        "openarmx_v10_bimanual_controllers_namespaced.yaml" if ns
        else "openarmx_v10_bimanual_controllers.yaml")

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               name="robot_state_publisher", output="screen", namespace=ns, parameters=[rd])
    control_node = Node(package="controller_manager", executable="ros2_control_node",
                        output="both", namespace=ns, parameters=[rd, controllers_file])

    def spawner(names):
        return Node(package="controller_manager", executable="spawner", namespace=ns,
                    arguments=[*names, "-c", cm], output="screen")

    # only the selected side's controllers
    pos_name = (f"{side}_forward_position_controller"
                if robot_controller == "forward_position_controller"
                else f"{side}_joint_trajectory_controller")
    jsb = TimerAction(period=3.0, actions=[spawner(["joint_state_broadcaster"])])
    main_ctrl = TimerAction(period=4.0, actions=[spawner([pos_name])])
    # NOTE: effort/velocity controllers + gravity/friction are spawned by openarmx_bilateral's
    # bilateral.launch (-t/-p, --unload-on-kill), NOT here, to avoid 'already loaded' conflicts.
    return [rsp, control_node, jsb, main_ctrl]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('arm_side', default_value='right', description='left | right'),
        DeclareLaunchArgument('arm_prefix', default_value=''),        # '' leader, 'follower' follower
        DeclareLaunchArgument('description_package', default_value='openarmx_description'),
        DeclareLaunchArgument('description_file', default_value='v10.urdf.xacro'),
        DeclareLaunchArgument('arm_type', default_value='v10'),
        DeclareLaunchArgument('control_mode', default_value='mit'),
        DeclareLaunchArgument('robot_controller', default_value='forward_position_controller',
                              choices=['forward_position_controller', 'joint_trajectory_controller']),
        DeclareLaunchArgument('left_can_interface', default_value='can1'),
        DeclareLaunchArgument('right_can_interface', default_value='can0'),
        DeclareLaunchArgument('can_fd', default_value='false'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=build)])
