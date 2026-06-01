#!/usr/bin/env python3
"""
CartPole LQR + Luenberger Observer Controller
CS424/CS524 — Robotics and Control

State vector:  x = [x, x_dot, theta, theta_dot]
Output:        y = x  (cart position only)
Control:       u = -K* @ x_hat  (uses estimated state)

All matrices derived analytically — see project report.
"""

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import place_poles
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rcl_interfaces.msg import ParameterDescriptor


# ═══════════════════════════════════════════════════════════════
#  SYSTEM PARAMETERS  (must match URDF and math derivation)
# ═══════════════════════════════════════════════════════════════
M = 1.0       # cart mass        [kg]
m = 0.1       # pendulum mass    [kg]
l = 0.5       # pendulum length to CoM  [m]
g = 9.81      # gravity          [m/s²]
b = 0.1       # cart damping     [N·s/m]
I = m * l**2  # moment of inertia [kg·m²]  (point mass approx)

# Denominator  Δ = (M+m)·I − (ml)²
Delta = (M + m) * (I + m*l**2) - (m*l)**2

# ═══════════════════════════════════════════════════════════════
#  STATE-SPACE MATRICES  A, B, C
# ═══════════════════════════════════════════════════════════════
A = np.array([
    [0,  1,                          0,   0],
    [0, -(I + m*l**2)*b / Delta,    -(m**2 * g * l**2) / Delta,   0],
    [0,  0,                          0,   1],
    [0,  m*l*b / Delta,              (M + m)*m*g*l / Delta,        0]
])

B = np.array([
    [0],
    [(I + m*l**2) / Delta],
    [0],
    [-m*l / Delta]
])

C = np.array([[1, 0, 0, 0]])   # only cart position measured

# ═══════════════════════════════════════════════════════════════
#  LQR GAIN  K*  via Algebraic Riccati Equation
# ═══════════════════════════════════════════════════════════════
#  Penalize angle 10× more than position
Q = np.diag([1.0, 1.0, 10.0, 10.0])
R = np.array([[1.0]])

P   = solve_continuous_are(A, B, Q, R)
K   = np.linalg.inv(R) @ B.T @ P      # (1×4)

# ═══════════════════════════════════════════════════════════════
#  LUENBERGER OBSERVER GAIN  L  via pole placement
#  Observer poles ~4× faster than controller poles
# ═══════════════════════════════════════════════════════════════
obs_poles = np.array([-10+10j, -10-10j, -20+20j, -20-20j])
result    = place_poles(A.T, C.T, obs_poles)
L         = result.gain_matrix.T    # (4×1)


class LQRController(Node):

    def __init__(self):
        super().__init__('lqr_controller')

        # ── state estimate (observer) ──────────────────────────
        self.x_hat = np.zeros((4, 1))   # [x, x_dot, theta, theta_dot]

        # ── last measurement & time ────────────────────────────
        self.last_y   = None
        self.last_time = None

        # ── print gains on startup ─────────────────────────────
        self.get_logger().info('═' * 55)
        self.get_logger().info('CartPole LQR + Observer — CS424/CS524')
        self.get_logger().info('═' * 55)
        self.get_logger().info(f'Δ      = {Delta:.6f}')
        self.get_logger().info(f'K*     = {K.flatten().round(4)}')
        self.get_logger().info(f'L      = {L.flatten().round(4)}')
        self.get_logger().info(f'A-BK* eigenvalues: '
            f'{np.linalg.eigvals(A - B @ K).round(3)}')
        self.get_logger().info(f'A-LC  eigenvalues: '
            f'{np.linalg.eigvals(A - L @ C).round(3)}')
        self.get_logger().info('═' * 55)

        # ── ROS interfaces ─────────────────────────────────────
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )
        self.pub = self.create_publisher(Float64, '/cart_force', 10)

        # ── control loop at 200 Hz ─────────────────────────────
        self.timer = self.create_timer(0.005, self.control_loop)
        self.get_logger().info('Controller running at 200 Hz')

    # ──────────────────────────────────────────────────────────
    def joint_state_cb(self, msg: JointState):
        """Parse joint states — find slider and pole joints."""
        try:
            xi  = msg.name.index('slider_joint')
            thi = msg.name.index('pole_joint')
        except ValueError:
            return

        x_meas     = msg.position[xi]
        theta_meas = msg.position[thi]

        # y = cart position only (as per C matrix)
        self.last_y    = np.array([[x_meas]])
        self.last_time = self.get_clock().now()

    # ──────────────────────────────────────────────────────────
    def control_loop(self):
        """
        Observer update + LQR control at 200 Hz.

        Observer:  x̂_dot = A·x̂ + B·u + L·(y − C·x̂)
        Control:   u     = −K*·x̂
        """
        if self.last_y is None:
            return

        dt = 0.005   # 200 Hz fixed step

        y     = self.last_y       # (1×1) measurement
        x_hat = self.x_hat        # (4×1) current estimate

        # ── compute control from current estimate ──────────────
        u = -K @ x_hat            # (1×1)

        # ── clamp force to actuator limit ──────────────────────
        u_sat = np.clip(u, -50.0, 50.0)

        # ── Luenberger observer step (Euler integration) ───────
        innovation  = y - C @ x_hat              # (1×1) output error
        x_hat_dot   = A @ x_hat + B @ u_sat + L @ innovation
        self.x_hat  = x_hat + dt * x_hat_dot     # (4×1)

        # ── publish force ──────────────────────────────────────
        msg       = Float64()
        msg.data  = float(u_sat[0, 0])
        self.pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    def get_state_estimate(self):
        return self.x_hat.flatten()


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
