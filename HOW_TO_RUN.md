# S10 Integrated Project — Build, Run and Verify

Target environment: **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Sim Harmonic**.

## 1. Install dependencies

```bash
sudo apt update
sudo apt install -y python3-yaml python3-colcon-common-extensions ros-jazzy-ros-gz ros-jazzy-rviz2
source /opt/ros/jazzy/setup.bash
```

If `ros-jazzy-ros-gz` is not available in your configured apt sources, install the ROS-Gazebo integration package recommended for your Jazzy installation.

## 2. Create a colcon workspace and build

From the extracted project folder:

```bash
mkdir -p ~/s10_ws/src
cp -a s10_mission_intelligence ~/s10_ws/src/
cd ~/s10_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select s10_gazebo_sim --symlink-install
source install/setup.bash
```

## 3. Start Gazebo + S10 + RViz

```bash
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py
```

Gazebo loads the S10 arena and quadrotor, the mission node parses `config/sample_mission.yaml`, validates the graph, generates and scores waypoints, validates the plan, and publishes it to the controller. RViz displays route waypoints, route line, risk zones, the drone mesh, and its trajectory.

For headless mode:

```bash
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py headless:=true
```

For a different mission:

```bash
ros2 launch s10_gazebo_sim s10_mission_sim.launch.py mission_file:=/absolute/path/to/mission.yaml
```

## 4. Monitor live topics

Open another terminal and source both environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/s10_ws/install/setup.bash
```

Then use:

```bash
ros2 topic list | grep -E 'mission_intelligence_node|drone_controller|s10_drone'
ros2 topic echo /mission_intelligence_node/fsm_state
ros2 topic echo /mission_intelligence_node/status
ros2 topic echo /mission_intelligence_node/mission_plan --once
ros2 topic echo /mission_intelligence_node/mission_graph --once
ros2 topic echo /mission_intelligence_node/risk_report --once
ros2 topic echo /mission_intelligence_node/fsm_event
ros2 topic echo /drone_controller/navigation_status
ros2 topic echo /s10_drone/odometry
```

Expected successful FSM lifecycle: `IDLE → PLANNING → VALIDATING → ARMED → EXECUTING → COMPLETED`. A safety validation failure ends in `ABORTED`; unsafe plans are not dispatched.

## 5. Run automated checks

The pure Python tests do not require ROS:

```bash
python3 -m unittest discover -s s10_gazebo_sim/tests -v
```

Native C intent/graph tests:

```bash
./scripts/test_native.sh
```

The advanced C planning scenario tests in `tests/` use YAML configuration and may require `libyaml-dev` (plus linking against libyaml). On Ubuntu, install it with `sudo apt install libyaml-dev`.

## 6. Troubleshooting

- **`Package 's10_gazebo_sim' not found`:** source `/opt/ros/jazzy/setup.bash`, build the workspace, then source `~/s10_ws/install/setup.bash`.
- **`No executable found`:** rebuild with `--symlink-install` and confirm scripts are executable in the installed package.
- **Model mesh missing:** the launch file adds the package's `models/` directory to `GZ_SIM_RESOURCE_PATH`; run from the built package environment rather than launching the SDF manually.
- **No mission starts:** inspect `mission_file` in launch output and verify the YAML file path. The default sample mission is installed under the package share directory.
- **RViz has no markers:** check `/mission_intelligence_node/waypoint_markers`, `/mission_intelligence_node/risk_zone_markers`, and the RViz Fixed Frame (`world`).
- **Drone remains stationary:** inspect `/s10_drone/odometry` and `/s10_drone/cmd_vel`, and confirm the Gazebo multicopter plugins load without errors. Gazebo plugin availability can differ by installation.
- **GUI is unavailable:** use `headless:=true` to test node execution and topics separately.
