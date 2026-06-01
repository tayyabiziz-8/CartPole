#!/usr/bin/env python3
"""
Logs cart position, pendulum angle, velocities, and control force.
Uses gz.transport directly — same as lqr_controller.
Ctrl+C to save to /tmp/cartpole_data.npz
"""
import numpy as np
import signal
import sys
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.model_pb2 import Model
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.t0          = None
        self.times       = []
        self.x_vals      = []
        self.theta_vals  = []
        self.xdot_vals   = []
        self.tdot_vals   = []
        self.u_vals      = []
        self.last_u      = 0.0

        # Subscribe to force via ROS (published by lqr_controller)
        self.sub_u = self.create_subscription(
            Float64, '/cart_force', self.u_cb, 10)

        # Subscribe to joint states via gz.transport directly
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            Model,
            '/world/cartpole_world/model/cartpole/joint_state',
            self.gz_js_cb)

        self.get_logger().info('Data logger running — Ctrl+C to save')

    def gz_js_cb(self, msg):
        # Find slider and pole joints
        xi = thi = None
        for i, j in enumerate(msg.joint):
            if j.name == 'slider_joint': xi  = i
            if j.name == 'pole_joint':   thi = i
        if xi is None or thi is None:
            return

        now = time.time()
        if self.t0 is None:
            self.t0 = now

        self.times.append(now - self.t0)
        self.x_vals.append(msg.joint[xi].axis1.position)
        self.theta_vals.append(msg.joint[thi].axis1.position)
        self.xdot_vals.append(msg.joint[xi].axis1.velocity)
        self.tdot_vals.append(msg.joint[thi].axis1.velocity)
        self.u_vals.append(self.last_u)

    def u_cb(self, msg):
        self.last_u = msg.data

    def save(self):
        n = len(self.times)
        if n == 0:
            self.get_logger().warn('No data collected!')
            return
        path = '/tmp/cartpole_data.npz'
        np.savez(path,
                 t        = np.array(self.times),
                 x        = np.array(self.x_vals),
                 theta    = np.array(self.theta_vals),
                 xdot     = np.array(self.xdot_vals),
                 thetadot = np.array(self.tdot_vals),
                 u        = np.array(self.u_vals))
        self.get_logger().info(f'Saved {n} samples → {path}')


_node = None

def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = DataLogger()

    def shutdown(sig, frame):
        _node.save()
        _node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    try:
        rclpy.spin(_node)
    except Exception:
        _node.save()
        _node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
