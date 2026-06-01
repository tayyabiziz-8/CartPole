# CartPole LQR — Inverted Pendulum Stabilization
### CS424/CS524: Robotics and Control — 6th Semester Final Project

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/docs/harmonic)
[![Python](https://img.shields.io/badge/Python-3.12-green)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](LICENSE)

---

## Overview

Full implementation of an **LQR controller with Luenberger state observer** for stabilizing an inverted pendulum on a cart (CartPole) — a canonical unstable underactuated system in control theory.

The project covers the complete linear control workflow:

```
Nonlinear EOM  →  Jacobian Linearization  →  Controllability/Observability Proofs
      →  LQR via Algebraic Riccati Equation  →  Luenberger Observer
            →  Separation Principle  →  ROS 2 / Gazebo Simulation
```

**Result:** Pendulum stabilized from 7.5° perturbation in ~6 simulation-seconds with zero steady-state error.

---

## Demo

> Simulation running in Gazebo Harmonic (headless physics + GUI visualization)

The blue cart slides on a rail, the red pole is the pendulum, the orange sphere is the tip mass.
A brief force impulse perturbs the system; the LQR controller recovers it to upright.

---

## System Parameters

| Symbol | Value | Description |
|--------|-------|-------------|
| M | 1.0 kg | Cart mass |
| m | 0.1 kg | Pendulum mass |
| l | 0.5 m | Pendulum length to CoM |
| g | 9.81 m/s² | Gravity |
| b | 0.1 N·s/m | Cart damping |
| I | ml² = 0.025 kg·m² | Moment of inertia |
| Δ | 0.0525 | System denominator (M+m)(I+ml²) − (ml)² |

---

## Controller Design

### State-Space Model

```
State:   x = [x,  ẋ,  θ,  θ̇]ᵀ
Input:   u = F  (horizontal force on cart, N)
Output:  y = x  (cart position — only measured output)

ẋ = A·x + B·u
y = C·x
```

### A, B, C Matrices (numerical)

```
A = [ 0,       1,         0,       0    ]
    [ 0,  -0.0952,   -0.4672,      0    ]
    [ 0,       0,         0,       1    ]
    [ 0,   0.0952,    20.574,      0    ]

B = [0,  0.9524,  0,  -0.9524]ᵀ

C = [1,  0,  0,  0]
```

### Controllability & Observability

```
rank([B | AB | A²B | A³B]) = 4 = n    ✓  Fully controllable
rank([C; CA; CA²; CA³])    = 4 = n    ✓  Fully observable from x alone
```

### LQR Gains

```
Q = diag(10, 1, 500, 50)    R = 0.01

K* = [-7.071,  -11.06,  -87.17,  -27.64]

Closed-loop poles (A − BK*):  {-10.45,  -3.16,  -1.14±0.84j}
```

### Luenberger Observer

```
x̂̇ = A·x̂ + B·u + L·(y − C·x̂)

L = [30.0,  223.4,  -0.95,  23.5]ᵀ

Observer poles (A − LC):  {-25±25j,  -15±15j}   (~15× faster than controller)
```

### Separation Principle

The augmented closed-loop system is block upper-triangular:

```
d/dt [x ] = [A−BK*   BK*  ] [x ]
     [ẽ ]   [  0    A−LC  ] [ẽ ]
```

Eigenvalues = eig(A−BK*) ∪ eig(A−LC) — independent by block structure. ✓

---

## Results

| Metric | Value |
|--------|-------|
| Max pendulum deflection | 6.19° |
| Settling time (θ < 0.5°) | ~6 simulation seconds |
| Steady-state θ error | < 0.001° |
| Steady-state x error | < 0.001 m |
| Steady-state control force | 0.00 N |

---

## Repository Structure

```
cartpole_ws/src/
├── cartpole_description/
│   ├── urdf/
│   │   └── cartpole.urdf.xacro      # Robot model (cart + pole + bob)
│   ├── CMakeLists.txt
│   └── package.xml
│
├── cartpole_gazebo/
│   ├── launch/
│   │   └── cartpole_launch.py       # Main launch file
│   ├── worlds/
│   │   └── cartpole.world           # Gazebo world SDF
│   ├── CMakeLists.txt
│   └── package.xml
│
└── cartpole_control/
    ├── cartpole_control/
    │   ├── lqr_controller.py        # LQR + observer node (500 Hz)
    │   ├── data_logger.py           # Logs states for plotting
    │   └── plot_results.py          # Generates report plots
    ├── setup.py
    └── package.xml
```

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`gz-harmonic`)
- Python 3: `numpy`, `scipy`, `matplotlib`

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  python3-numpy python3-scipy \
  python3-gz-transport13 \
  python3-gz-msgs10
```

---

## Build and Run

### Build

```bash
source /opt/ros/jazzy/setup.bash
cd cartpole_ws
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### Run (headless — physics only)

```bash
ros2 launch cartpole_gazebo cartpole_launch.py
```

### Run (with GUI visualization)

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch cartpole_gazebo cartpole_launch.py
```

### Log data and generate plots

Open a second terminal after launching:

```bash
source /opt/ros/jazzy/setup.bash
source ~/cartpole_ws/install/setup.bash
ros2 run cartpole_control data_logger
# Press Ctrl+C after 30+ seconds to save

python3 ~/cartpole_ws/src/cartpole_control/cartpole_control/plot_results.py
# Plot saved to /tmp/cartpole_results.png
```

---

## How It Works

1. **Gazebo** simulates the physics at 0.2× real-time (stable on VMs)
2. **Joint states** (position + velocity for both joints) are published by the `JointStatePublisher` plugin
3. **ros_gz_bridge** bridges joint states to ROS 2
4. **lqr_controller** reads the full state [x, ẋ, θ, θ̇] and computes `u = −K*·x`
5. Force is published directly via `gz.transport13` Python API (bypasses bridge for low latency)
6. A brief 4N impulse perturbs the system at startup; LQR engages immediately after

---

## References

1. Åström & Murray — *Feedback Systems* (Princeton, 2008)
2. Brunton & Kutz — *Data-Driven Science and Engineering* (Cambridge, 2019)
3. Anderson & Moore — *Optimal Control: Linear Quadratic Methods* (Prentice-Hall, 1990)
4. Luenberger — "Observers for multivariable systems," IEEE TAC, 1966
5. [ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy)
6. [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)

---

## License

MIT License — see [LICENSE](LICENSE) for details.
