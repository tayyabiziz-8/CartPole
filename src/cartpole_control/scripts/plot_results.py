#!/usr/bin/env python3
"""
Run after data_logger.py has saved cartpole_data.npz
Generates 3 report-quality plots.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams.update({
    'font.family': 'serif',
    'font.size': 11,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'figure.dpi': 150
})

data  = np.load('/tmp/cartpole_data.npz')
t     = data['t']
x     = data['x']
theta = np.degrees(data['theta'])   # convert to degrees for readability
u     = data['u']

fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
fig.suptitle('CartPole LQR+Observer — Closed-Loop Response', fontsize=13)

axes[0].plot(t, x,     color='steelblue',   linewidth=1.5, label='Cart position $x$ (m)')
axes[0].axhline(0, color='red', linestyle='--', linewidth=0.8, label='Reference $x_{ref}=0$')
axes[0].set_ylabel('Position (m)')
axes[0].legend(loc='upper right', fontsize=9)
axes[0].set_title('Cart Position Tracking')

axes[1].plot(t, theta, color='darkorange', linewidth=1.5, label='Pendulum angle $\\theta$ (deg)')
axes[1].axhline(0, color='red', linestyle='--', linewidth=0.8, label='Upright $\\theta=0$')
axes[1].set_ylabel('Angle (degrees)')
axes[1].legend(loc='upper right', fontsize=9)
axes[1].set_title('Pendulum Angle Stabilization')

axes[2].plot(t, u,     color='forestgreen', linewidth=1.2, label='Control force $u$ (N)')
axes[2].axhline(0, color='gray', linestyle='--', linewidth=0.8)
axes[2].set_ylabel('Force (N)')
axes[2].set_xlabel('Time (s)')
axes[2].legend(loc='upper right', fontsize=9)
axes[2].set_title('Control Effort')

plt.tight_layout()
outpath = '/tmp/cartpole_results.png'
plt.savefig(outpath, bbox_inches='tight')
print(f'Saved → {outpath}')
plt.show()
