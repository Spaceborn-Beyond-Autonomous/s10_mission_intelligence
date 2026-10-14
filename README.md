# S10 Mission Intelligence & Autonomous Drone Simulator

Autonomous UAV Mission Intelligence pipeline running on **ROS 2 Jazzy** with **Gazebo Sim** physics and **RViz2** telemetry.

## Features

- **Mission Intent Parser & DAG Builder**: Converts declarative YAML mission specifications into structured Directed Acyclic Graphs (DAG).
- **Multi-Objective Route Optimizer**: Optimizes flight paths balancing fuel consumption, mission time, and environmental risk constraints (2-opt algorithm).
- **Deterministic Safety & Risk Engine**: Evaluates geofencing limits, altitude envelopes, obstacle proximity, and dynamic risk scoring.
- **Hierarchical Mission FSM & Replanner**: Manages 8 discrete operational states (`IDLE` → `PLANNING` → `VALIDATING` → `ARMED` → `EXECUTING` → `REPLANNING` → `COMPLETED`) with real-time dynamic obstacle replanning at 2 Hz.
- **Photorealistic 3D Quadrotor Drone Model**: High-fidelity carbon-fiber airframe, forward camera gimbal, aerodynamic propellers (CW/CCW), and standard aviation navigation LEDs.
- **Dual Telemetry Display**: Real-time physical simulation in Gazebo Sim and live 3D drone mesh tracking with trajectory trail in RViz2.

---

## Repository Structure

```
s10_mission_intelligence/
├── README.md                                 # Overview and getting started
├── HOW_TO_RUN.md                             # Step-by-step launch & verification guide
├── S10_DOCUMENTATION.md                      # Comprehensive architectural documentation
├── S10_Mission_Intelligence_..._Plan.pdf     # 5-Day plan specifications
├── s10_gazebo_sim/                           # ROS 2 simulation package
│   ├── CMakeLists.txt                        # ROS 2 build script
│   ├── package.xml                           # Package manifest & dependencies
│   ├── config/                               # Mission intents & YAML parameters
│   ├── launch/                               # ROS 2 launch files
│   ├── models/                               # 3D arena models and realistic s10_drone SDF
│   ├── rviz/                                 # Pre-configured RViz2 display setup
│   ├── src/                                  # Mission intelligence nodes & drone controller
│   └── worlds/                               # Gazebo Sim 3D simulation environment
├── include/                                  # C/C++ interface headers
├── src/                                      # Modular interface stubs
└── config/                                   # High-level mission configs
```

---

## Quick Start

### 1. Build the Workspace
```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build the package from your colcon workspace
colcon build --packages-select s10_gazebo_sim --symlink-install
source install/setup.bash
```

### 2. Launch Simulation
```bash
# Starts Gazebo Sim, RViz2, S10 Mission Intelligence orchestrator, and Drone Controller
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py
```

### 3. Headless Mode (No GUI)
```bash
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py headless:=true
```

---

## Documentation

- [HOW_TO_RUN.md](HOW_TO_RUN.md): Complete guide on how the simulation runs, topic monitoring commands, and troubleshooting.
- [S10_DOCUMENTATION.md](S10_DOCUMENTATION.md): Full technical breakdown of all 7 pipeline modules, state machine transitions, and algorithms.
