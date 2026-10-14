#!/usr/bin/env python3
"""
S10 Mission Intelligence — Gazebo Sim Launch File
==================================================
Launches:
    1. Gazebo Sim with the S10 world and drone model
    2. ros_gz_bridge for cmd_vel, odometry, and IMU
    3. mission_intelligence_node  (S10 orchestrator)
    4. drone_controller           (velocity waypoint tracker)

Usage:
    ros2 launch s10_gazebo_sim s10_mission_sim.launch.py
    ros2 launch s10_gazebo_sim s10_mission_sim.launch.py headless:=true
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("s10_gazebo_sim")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    # ── Paths ───────────────────────────────────────────────────────
    world_file = os.path.join(pkg_share, "worlds", "s10_mission_world.sdf")
    model_dir  = os.path.join(pkg_share, "models")
    params_file = os.path.join(pkg_share, "config", "mission_params.yaml")
    mission_file = os.path.join(pkg_share, "config", "sample_mission.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "s10_mission.rviz")

    # ── Launch arguments ────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo in headless mode (no GUI)"
    )
    mission_arg = DeclareLaunchArgument(
        "mission_file", default_value=mission_file,
        description="Path to the mission intent YAML"
    )

    # ── Environment ─────────────────────────────────────────────────
    current_gz_res = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    full_model_path = f"{model_dir}:{current_gz_res}" if current_gz_res else model_dir

    set_model_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=full_model_path,
    )
    set_gz_ip = SetEnvironmentVariable(
        name="GZ_IP",
        value="127.0.0.1",
    )

    # ── Gazebo Sim ──────────────────────────────────────────────────
    gz_args = PythonExpression([
        "'-r -s ' + '", world_file, "' if '", LaunchConfiguration("headless"), "' == 'true' else '-r ' + '", world_file, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args,
        }.items(),
    )

    # ── Spawn drone ─────────────────────────────────────────────────
    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "s10_drone",
            "-file", os.path.join(model_dir, "s10_drone", "model.sdf"),
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
        output="screen",
    )

    # ── ROS ↔ Gazebo bridge ─────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/s10_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/s10_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/s10_drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    # ── S10 Mission Intelligence Node ───────────────────────────────
    mission_node = Node(
        package="s10_gazebo_sim",
        executable="mission_intelligence_node.py",
        name="mission_intelligence_node",
        output="screen",
        parameters=[
            params_file,
            {"mission_file": LaunchConfiguration("mission_file")},
        ],
    )

    # ── Drone Controller Node ───────────────────────────────────────
    controller = Node(
        package="s10_gazebo_sim",
        executable="drone_controller.py",
        name="drone_controller",
        output="screen",
        parameters=[{
            "cruise_speed": 5.0,
            "acceptance_radius": 2.0,
            "max_speed": 8.0,
        }],
    )

    # ── RViz2 ───────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    return LaunchDescription([
        headless_arg,
        mission_arg,
        set_model_path,
        set_gz_ip,
        gz_sim,
        spawn_drone,
        bridge,
        mission_node,
        controller,
        rviz,
    ])
