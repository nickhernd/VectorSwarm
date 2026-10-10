# VectorSwarm
VectorSwarm: Autonomous DIY Micro-Drone Swarm &amp; Distributed Formation Control. An open-source, fully DIY micro-drone platform designed for distributed swarm coordination, dynamic 3D formation generation, and real-time trajectory optimization.  `VectorSwarm` decouples high-level formation consensus from low-level flight control.

## Project Architecture

The repository is organized into three decoupled layers: low-level MCU firmware, mathematical simulation/planning tools, and hardware CAD/schematic assets.

```text
VectorSwarm/
├── firmware/                     # Low-level MCU C++ Code
│   ├── src/
│   │   ├── drivers/              # Hardware Abstraction Layer (HAL) for IMU, Motors, UWB
│   │   ├── estimation/           # Error-State Kalman Filter (ESKF)
│   │   ├── control/              # Cascaded PID Attitude & Position loops
│   │   └── coms/                 # ESP-NOW / P2P UDP Mesh protocol
│   └── platformio.ini            # PlatformIO configuration
├── simulation/                   # High-Level Swarm Intelligence & Tools
│   ├── algorithms/               # Hungarian Algorithm & B-Spline Trajectory Planners
│   ├── dynamics/                 # Quadcopter 6-DOF Physical Models (SE(3))
│   └── main_sim.py               # 3D PyPlot / Meshcat Visualization Entrypoint
├── hardware/                     # Physical Assets
│   ├── pcb/                      # KiCad Schematics & Gerber Files
│   └── frame/                    # 3D STL/STEP Files for Carbon & Printed Frames
└── README.md
```

---

## Key Features & Technical Highlights

### 1. Bare-Metal & RTOS Flight Firmware
* **Hard Real-Time Loop:** Runs inner-rate loops (Gyro/Attitude) at >= 200 Hz and outer-position loops at 50 Hz.
* **Sensor Fusion (ESKF):** Uses an Error-State Kalman Filter combining 6-axis IMU raw data with Ultrawideband (UWB) range measurements and Time-of-Flight (ToF) altitude data to compute attitude quaternions $q \in \mathbb{H}$ without gimbal lock.
* **Cascaded PID Control:** Dual-loop architecture for attitude ($\phi, \theta, \psi$) and position ($x, y, z$) control mapping commands to motor PWM/DShot protocols.

### 2. Distributed Swarm Consensus & Formation Control
* **Optimal Task Allocation:** Minimizes global kinetic cost during formation switching using the Hungarian Algorithm ($O(N^3)$) over the distance matrix $C_{i,j} = \Vert{}p_i - g_j\Vert{}_2$.
* **Collision Avoidance:** Generates smooth, continuous $C^2$ trajectories using B-Spline parametrization subject to minimum inter-agent safety distances $\Vert{}p_i(t) - p_j(t)\Vert{} \ge d_{\text{min}}$.
* **Peer-to-Peer Mesh:** Drones broadcast local state vectors $x = [p, v, q]^T$ at 20-50 Hz using low-latency MAC-layer packets (ESP-NOW), eliminating GCS latency bottlenecks.

---

## Mathematical & System Formulation

The kinematic state of each agent $i$ is defined on the Special Euclidean Group $SE(3)$:

$$\mathbf{x}_i = \begin{bmatrix} \mathbf{p}_i \\ \mathbf{v}_i \\ \mathbf{q}_i \\ \boldsymbol{\omega}_i \end{bmatrix} \in \mathbb{R}^3 \times \mathbb{R}^3 \times \mathbb{H} \times \mathbb{R}^3$$

### Goal Assignment Optimization
To transition $N$ drones from current positions $\mathbf{p}_i$ to target formation vertices $\mathbf{g}_j$, the allocation matrix $\mathbf{X} \in \{0, 1\}^{N \times N}$ is solved via:

$$\min_{\mathbf{X}} \sum_{i=1}^{N} \sum_{j=1}^{N} X_{ij} \Vert{}\mathbf{p}_i - \mathbf{g}_j\Vert{}_2^2 \quad \text{subject to} \quad \sum_{j=1}^{N} X_{ij} = 1, \; \sum_{i=1}^{N} X_{ij} = 1$$

---

## Hardware Specifications (Per Agent)

| Component | Model / Spec | Purpose |
| :--- | :--- | :--- |
| **MCU** | ESP32-D0WDQ6 (Dual-Core @ 240MHz) | Real-time control loop & ESP-NOW mesh |
| **IMU** | MPU6050 / ICM-20948 | 6-DOF / 9-DOF Acceleration & Gyroscope sensing |
| **Positioning** | Decawave DWM1000 / DWM3000 UWB | Sub-10cm 3D indoor localization |
| **Range / Altitude** | STMicroelectronics VL53L1X ToF | Laser altitude sensing |
| **Motors / ESCs** | 0802 19000KV BLDC / 4-in-1 5A DShot300 ESC | Actuation and thrust generation |
| **Frame** | Custom 65mm Brushless Frame (3D Printed / Carbon) | Structural support |

---

## Getting Started & Installation

### Prerequisites

* **C++ Compiler & Build System:** PlatformIO IDE or `platformio-cli`.
* **Python Environment:** Python 3.10+ with `numpy`, `scipy`, and `matplotlib`.

### 1. Simulation Setup (Python)

1. Clone the repository:
   ```bash
   git clone [https://github.com/your-username/VectorSwarm.git](https://github.com/your-username/VectorSwarm.git)
   cd VectorSwarm/simulation
   ```

2. Create a virtual environment and install dependencies:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Execute the formation assignment and trajectory generator:
   ```bash
   python main_sim.py
   ```

### 2. Flashing MCU Firmware (PlatformIO)

1. Connect your ESP32 target board via USB.
2. Navigate to the firmware directory and flash:
   ```bash
   cd ../firmware
   pio run --target upload
   ```
3. Monitor real-time telemetry over Serial:
   ```bash
   pio device monitor --baud 115200
   ```

---

## Roadmap

- [x] Python simulation engine for Hungarian target assignment.
- [x] Core 6-DOF physical quadcopter simulation model.
- [ ] Implement C++ ESKF (Error-State Kalman Filter) on ESP32 firmware.
- [ ] ESP-NOW peer-to-peer heartbeat mesh protocol.
- [ ] Integration of UWB anchors for indoor 3D trilateration.
- [ ] Initial 4-agent flight tests and video demonstration.

---

## References & Bibliography

1. **D'Andrea, R.** *"Control Systems Design for Micro-Aerial Vehicles."* IEEE Control Systems Magazine, 2012.
2. **Alonso-Mora, J., et al.** *"Optimal Trajectory Planning for Quadrotor Swarms."* IEEE Transactions on Robotics, 2012.
3. **Preiss, J. A., Honig, W., Sukhatme, G. S., & Ayanian, N.** *"Crazyswarm: A large-scale quadcopter swarm."* IEEE International Conference on Robotics and Automation (ICRA), 2017.
4. **Sola, J.** *"Quaternion kinematics for the error-state Kalman filter."* arXiv pre-print arXiv:1711.02508, 2017.
5. **Luukkonen, G.** *"Modelling and control of quadcopter."* Aalto University School of Science, 2011.

---

## License

Distributed under the **MIT License**. See `LICENSE` for more information.
