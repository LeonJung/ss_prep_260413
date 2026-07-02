#!/usr/bin/env python3
# Zero-arg latency capture (measurement #B). Logs TIMESTAMPED CAN frames on all 4
# channels while the control loop runs, so we can decompose per-cycle latency:
#   - CAN wire time (frame spacing at the bitrate) vs
#   - USB/host gaps (the big chunk if USB polling dominates).
#
# Run it as a 4th terminal WHILE bringups + bilateral.launch are running, then move
# the leader ~15-20 s and Ctrl+C. Send /tmp/latency_candump.log for offline analysis.
#   ros2 launch openarmx_bilateral latency_capture.launch.py     # no args
#
# (candump uses the kernel hardware timestamp = time the frame reached the host,
#  i.e. AFTER USB. Gaps between a command frame and its feedback frame therefore
#  include the USB round-trip + motor response.)

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='[latency_capture] logging can0..can3 -> /tmp/latency_candump.log '
                    '| move the leader ~15-20s, then Ctrl+C, then send the file.'),
        ExecuteProcess(
            cmd=['bash', '-lc',
                 'candump -ta can0 can1 can2 can3 > /tmp/latency_candump.log'],
            output='screen'),
    ])
