#!/usr/bin/env python3
"""joint_state_bridge — merge oa_fd_cpp per-arm states into robot_state_publisher input.

oa_fd_cpp publishes (names "joint1".."joint7"):
  /oa/leader_right/joint_state   /oa/leader_left/joint_state
  /oa/follower_right/joint_state /oa/follower_left/joint_state

robot_state_publisher (per robot, namespaced) needs one JointState with the
URDF joint names openarmx_{left,right}_joint1..7:
  /leader/joint_states           /follower/joint_states

Run one bridge per robot:  ros2 run oa_viz joint_state_bridge --ros-args -p robot:=leader
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NJ = 7
SIDES = ('left', 'right')


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        self.declare_parameter('robot', 'leader')      # leader | follower
        self.robot = self.get_parameter('robot').value
        if self.robot not in ('leader', 'follower'):
            raise ValueError(f"robot must be leader|follower, got {self.robot!r}")

        self.pos = {s: [0.0] * NJ for s in SIDES}
        self.vel = {s: [0.0] * NJ for s in SIDES}
        self.seen = {s: False for s in SIDES}

        for s in SIDES:
            self.create_subscription(
                JointState, f'/oa/{self.robot}_{s}/joint_state',
                lambda m, side=s: self._cb(side, m), 10)

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / 50.0, self._tick)   # 50 Hz is plenty for RViz
        self.get_logger().info(
            f'bridging /oa/{self.robot}_{{left,right}}/joint_state -> '
            f'{self.get_namespace().rstrip("/")}/joint_states')

    def _cb(self, side, msg):
        n = min(NJ, len(msg.position))
        self.pos[side][:n] = msg.position[:n]
        if len(msg.velocity) >= n:
            self.vel[side][:n] = msg.velocity[:n]
        self.seen[side] = True

    def _tick(self):
        if not any(self.seen.values()):
            return                      # nothing yet — publish only real states
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        for s in SIDES:
            out.name += [f'openarmx_{s}_joint{k}' for k in range(1, NJ + 1)]
            out.position += list(self.pos[s])
            out.velocity += list(self.vel[s])
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
