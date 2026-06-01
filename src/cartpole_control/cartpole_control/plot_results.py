#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

matplotlib.rcParams.update({
    'font.family': 'serif', 'font.size': 11,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 150
})

data     = np.load('/tmp/cartpole_data.npz')
t        = data['t']
x        = data['x']
theta    = np.degrees(data['theta'])
thetadot = np.degrees(data['thetadot'])
u        = data['u']

fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
fig.suptitle('CartPole LQR — Closed-Loop Response\nCS424/CS524',
             fontsize=13, fontweight='bold')

axes[0].plot(t, x, color='steelblue', lw=1.5, label='Cart position $x$ (m)')
axes[0].axhline(0, color='red', ls='--', lw=0.8, label='Reference')
axes[0].set_ylabel('Position (m)')
axes[0].legend(fontsize=9)
axes[0].set_title('Cart Position Tracking')

axes[1].plot(t, theta, color='darkorange', lw=1.5,
             label='Pendulum angle $\\theta$ (°)')
axes[1].axhline(0, color='red', ls='--', lw=0.8)
axes[1].set_ylabel('Angle (°)')
axes[1].legend(fontsize=9)
axes[1].set_title('Pendulum Angle Stabilization')

axes[2].plot(t, u, color='forestgreen', lw=1.2, label='Control force $u$ (N)')
axes[2].axhline(0, color='gray', ls='--', lw=0.8)
axes[2].set_ylabel('Force (N)')
axes[2].set_xlabel('Time (s)')
axes[2].legend(fontsize=9)
axes[2].set_title('Control Effort')

# Compute performance metrics
try:
    lqr_start = next(i for i,v in enumerate(u) if abs(v) > 0.01)
    t_lqr = t[lqr_start:]
    th_lqr = theta[lqr_start:]
    settle_idx = next((i for i in range(len(th_lqr))
                      if all(abs(th_lqr[i:i+50]) < 2.0)), None)
    if settle_idx:
        t_settle = t_lqr[settle_idx] - t_lqr[0]
        axes[1].axvline(t_lqr[0] + t_settle, color='blue',
                        ls=':', lw=1, label=f'Settle: {t_settle:.1f}s')
        axes[1].legend(fontsize=9)
        print(f'Settling time: {t_settle:.2f}s')
    print(f'Max |theta|: {max(abs(theta)):.2f}°')
    print(f'Max |u|: {max(abs(u)):.2f}N')
    print(f'Final theta: {theta[-1]:.4f}°')
except Exception as e:
    print(f'Metrics: {e}')

plt.tight_layout()
out = '/tmp/cartpole_results.png'
plt.savefig(out, bbox_inches='tight')
print(f'Plot saved → {out}')
