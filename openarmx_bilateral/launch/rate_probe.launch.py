#!/usr/bin/env python3
# Zero-arg software-rate probe (measurement #A/CM). Measures the ACTUAL publish rate +
# jitter of joint_states and controller command topics, and reads each
# controller_manager's configured update_rate — to confirm the ~89 Hz ceiling is the
# CM/broadcaster/DDS software layer (candump proved the CAN loop does 154-173 Hz).
#
# Run WHILE bringups + bilateral.launch are running, move the leader ~15-20 s, Ctrl+C,
# then send /tmp/rate_probe.csv.
#   ros2 launch openarmx_bilateral rate_probe.launch.py     # no args

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openarmx_bilateral',
            executable='rate_probe_node',
            name='rate_probe',
            output='screen',
        ),
    ])
