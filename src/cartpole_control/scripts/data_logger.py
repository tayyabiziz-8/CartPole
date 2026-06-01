#!/usr/bin/env python3
"""
Logs cart position, pendulum angle, and control force.
Run alongside lqr_controller to generate report plots.
Stop with Ctrl+C — saves to ~/cartpole_data.npz automatically.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')
        self.t0        = None
        self.times     = []
        self.x_vals    = []
        self.theta_vals = []
        self.u_vals    = []
        self.last_u    = 0.0

        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self.js_cb, 10)
        self.sub_u  = self.create_subscription(
            Float64, '/cart_force', self.u_cb, 10)

        self.get_logger().info('Data logger running — Ctrl+C to save')

    def js_cb(self, msg):
        try:
            xi  = msg.name.index('slider_joint')
            thi = msg.name.index('pole_joint')
        except ValueError:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now

        self.times.append(now - self.t0)
        self.x_vals.append(msg.position[xi])
        self.theta_vals.append(msg.position[thi])
        self.u_vals.append(self.last_u)

    def u_cb(self, msg):
        self.last_u = msg.data

    def save(self):
        path = '/tmp/cartpole_data.npz'
        np.savez(path,
                 t=np.array(self.times),
                 x=np.array(self.x_vals),
                 theta=np.array(self.theta_vals),
                 u=np.array(self.u_vals))
        self.get_logger().info(f'Saved {len(self.times)} samples → {path}')


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
