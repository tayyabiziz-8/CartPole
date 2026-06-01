#!/usr/bin/env python3
"""
CartPole LQR Controller — CS424/CS524
Full state feedback using direct Gazebo measurements.
Gazebo provides [x, x_dot, theta, theta_dot] via joint_states.
Observer design is presented in the report for hardware implementation.
"""
import numpy as np
from scipy.linalg import solve_continuous_are
import threading, time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double

# ── System parameters ──────────────────────────────────────────
M = 1.0; m = 0.1; l = 0.5; g = 9.81; b = 0.1
I     = m * l**2
Delta = (M + m) * (I + m*l**2) - (m*l)**2

# ── State-space matrices ───────────────────────────────────────
A = np.array([
    [0, 1, 0, 0],
    [0, -(I+m*l**2)*b/Delta, -(m**2*g*l**2)/Delta, 0],
    [0, 0, 0, 1],
    [0, m*l*b/Delta, (M+m)*m*g*l/Delta, 0]
])
B = np.array([[0],[(I+m*l**2)/Delta],[0],[-m*l/Delta]])
C = np.array([[1, 0, 0, 0]])

# ── LQR — balanced weights ─────────────────────────────────────
Q = np.diag([10.0, 1.0, 500.0, 50.0])
R = np.array([[0.01]])
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

GZ_FORCE_TOPIC  = '/model/cartpole/joint/slider_joint/cmd_force'
ROS_STATE_TOPIC = '/world/cartpole_world/model/cartpole/joint_state'


class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        # Full state vector from Gazebo measurements
        self.state      = np.zeros((4, 1))
        self.got_state  = False
        self.lqr_active = False
        self.step_count = 0
        self.min_theta  = 999.0
        self.max_theta  = -999.0

        # gz transport for direct force publishing — no bridge latency
        self.gz_node    = GzNode()
        self.gz_pub     = self.gz_node.advertise(GZ_FORCE_TOPIC, Double)

        self.get_logger().info('=' * 55)
        self.get_logger().info('CartPole LQR — CS424/CS524')
        self.get_logger().info(f'K* = {K.flatten().round(3)}')
        self.get_logger().info(
            f'A-BK* poles: {np.linalg.eigvals(A-B@K).round(3)}')
        self.get_logger().info(
            f'Open-loop poles: {np.linalg.eigvals(A).round(3)}')
        self.get_logger().info('=' * 55)

        self.sub   = self.create_subscription(
            JointState, ROS_STATE_TOPIC, self.joint_state_cb, 10)
        self.pub_log = self.create_publisher(Float64, '/cart_force', 10)

        # Control loop at 500 Hz
        self.timer = self.create_timer(0.002, self.control_loop)

        # Perturbation thread
        t = threading.Thread(target=self.perturb, daemon=True)
        t.start()

    # ── State callback ─────────────────────────────────────────
    def joint_state_cb(self, msg: JointState):
        try:
            xi  = msg.name.index('slider_joint')
            thi = msg.name.index('pole_joint')
        except ValueError:
            return

        # Full state directly from Gazebo — positions AND velocities
        x         = msg.position[xi]
        x_dot     = msg.velocity[xi]
        theta     = msg.position[thi]
        theta_dot = msg.velocity[thi]

        self.state = np.array([[x], [x_dot], [theta], [theta_dot]])
        self.got_state = True

    # ── Perturbation ───────────────────────────────────────────
    def perturb(self):
        while not self.got_state:
            time.sleep(0.05)

        self.get_logger().info('Physics running — perturbing...')
        time.sleep(0.5)

        # Apply brief impulse directly via gz transport
        msg = Double()
        msg.data = 4.0
        t0 = time.time()
        while time.time() - t0 < 0.8:      # 0.8s real = 0.16s sim
            self.gz_pub.publish(msg)
            time.sleep(0.01)

        theta_deg = float(np.degrees(self.state[2, 0]))
        self.get_logger().info(
            f'Perturbation done. theta={theta_deg:.2f}°')

        # Zero force briefly
        msg.data = 0.0
        t0 = time.time()
        while time.time() - t0 < 0.3:
            self.gz_pub.publish(msg)
            time.sleep(0.01)

        theta_deg = float(np.degrees(self.state[2, 0]))
        self.get_logger().info(
            f'LQR active! theta={theta_deg:.2f}° '
            f'theta_dot={float(self.state[3,0]):.3f} rad/s')
        self.lqr_active = True

    # ── Control loop ───────────────────────────────────────────
    def control_loop(self):
        if not self.lqr_active or not self.got_state:
            return

        self.step_count += 1
        dt = 0.002

        # u = -K* x  (full state feedback)
        u     = float((-K @ self.state)[0, 0])
        u_sat = float(np.clip(u, -50.0, 50.0))

        # Publish via gz transport directly
        gz_msg      = Double()
        gz_msg.data = u_sat
        self.gz_pub.publish(gz_msg)

        # Log for data collection
        log_msg      = Float64()
        log_msg.data = u_sat
        self.pub_log.publish(log_msg)

        theta     = float(self.state[2, 0])
        theta_dot = float(self.state[3, 0])
        x         = float(self.state[0, 0])

        self.min_theta = min(self.min_theta, theta)
        self.max_theta = max(self.max_theta, theta)

        if self.step_count % 500 == 0:
            self.get_logger().info(
                f't={self.step_count*dt:.1f}s | '
                f'x={x:.3f}m | '
                f'θ={theta:.4f}rad ({np.degrees(theta):.2f}°) | '
                f'θ̇={theta_dot:.3f}rad/s | '
                f'u={u_sat:.2f}N | '
                f'θ∈[{np.degrees(self.min_theta):.1f}°,'
                f'{np.degrees(self.max_theta):.1f}°]')


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
