# S10 Mission Intelligence Simulator — Integrated Release

This repository integrates the S10 member work into one ROS 2 Jazzy package with Gazebo Sim and RViz2 visualization, plus the native C mission-intent/planning modules contributed in the other archives.

## What is included

- `s10_gazebo_sim/`: runnable ROS 2 simulation package (mission parser, mission graph, waypoint generation, fuel/time/risk scoring, safety validation, FSM, replanning, drone controller, Gazebo world/model, RViz markers).
- `include/` and `src/`: native C mission-intent, graph, planning, optimization, risk, validation, FSM and adapter/output contracts from the member submissions.
- `tests/`: native C regression scenarios from planning/optimization work.
- `s10_gazebo_sim/tests/`: pure-Python pipeline regression tests that run without ROS.
- `HOW_TO_RUN.md`: detailed setup and runtime verification.
- `S10_DOCUMENTATION.md`: architecture and module documentation.

The ROS/Gazebo Python pipeline is the executable visualization path. The native C modules are retained as a separate modular core; not every native adapter/output module is wired into the ROS node.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim (Harmonic recommended for Jazzy)
- `ros_gz_sim`, `ros_gz_bridge`, `rviz2`
- `python3-yaml`, `colcon`, `build-essential`

## Build and launch

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y python3-yaml python3-colcon-common-extensions ros-jazzy-ros-gz ros-jazzy-rviz2
mkdir -p ~/s10_ws/src
cp -a s10_gazebo_sim ~/s10_ws/src/
cd ~/s10_ws
colcon build --packages-select s10_gazebo_sim --symlink-install
source install/setup.bash
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py
```

For headless simulation:

```bash
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py headless:=true
```

To choose another mission file:

```bash
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py mission_file:=/absolute/path/to/mission.yaml
```

## Safety behavior

A mission is dispatched only after graph validation and safety validation pass. If safety validation fails, the FSM enters `ABORTED` and the plan is not sent to the drone controller. This is a simulation project; validate dynamics, controller tuning, and all safety assumptions before any hardware use.

## Run checks without ROS

```bash
python3 -m unittest discover -s s10_gazebo_sim/tests -v
./scripts/test_native.sh
```

The native tests cover the natural-language intent parser and graph builder. The planning scenarios that load YAML require `libyaml` development headers/libraries. See `HOW_TO_RUN.md` for commands to inspect topics and troubleshoot Gazebo/RViz.
