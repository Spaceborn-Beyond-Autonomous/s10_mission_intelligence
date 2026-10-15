#!/usr/bin/env python3
"""
S10 Drone Controller
====================
ROS 2 node that commands the Gazebo drone via Twist velocity messages.
It receives a waypoint sequence, flies to each one in order, and
publishes status feedback.

Subscribed topics
    /s10_drone/odometry          (nav_msgs/Odometry)   — from Gazebo bridge
    ~/target_waypoints           (std_msgs/String)      — JSON waypoint list

Published topics
    /s10_drone/cmd_vel           (geometry_msgs/Twist)  — velocity commands
    ~/navigation_status          (std_msgs/String)      — JSON status
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker


class DroneController(Node):
    """Proportional velocity controller for waypoint following."""

    def __init__(self):
        super().__init__("drone_controller")

        # Parameters
        self.declare_parameter("cruise_speed", 5.0)
        self.declare_parameter("acceptance_radius", 2.0)
        self.declare_parameter("max_speed", 8.0)

        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.accept_r = self.get_parameter("acceptance_radius").value
        self.max_speed = self.get_parameter("max_speed").value

        # State
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self._loop_count = 0
        self.waypoints = []
        self.wp_index = 0
        self.mission_active = False
        self.loiter_until = 0.0
        self.armed = False
        self.odom_received = False
        self.landing_active = False
        self._last_odom_wall_time = time.monotonic()

        # Visualizations
        self.marker_pub = self.create_publisher(Marker, "/drone_controller/drone_marker", 10)
        self.path_pub = self.create_publisher(Path, "/drone_controller/flight_path", 10)
        self.flight_path = Path()
        self.flight_path.header.frame_id = "world"
        self._path_count = 0

        # Pubs / Subs
        self.cmd_pub = self.create_publisher(Twist, "/s10_drone/cmd_vel", 10)
        self.enable_pub = self.create_publisher(Bool, "/s10_drone/enable", 10)
        self.status_pub = self.create_publisher(String, "~/navigation_status", 10)

        self.create_subscription(Odometry, "/s10_drone/odometry", self._odom_cb, 10)
        self.create_subscription(String, "~/target_waypoints", self._wp_cb, 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self._control_loop)
        # The Gazebo MulticopterVelocityControl system is gated by an enable
        # topic on some Harmonic builds. Keep it asserted so late-spawned models
        # do not miss a one-shot enable message.
        self.create_timer(0.5, self._publish_enable)

        self.get_logger().info("DroneController ready — waiting for Gazebo odometry and mission waypoints")

    # ── Callbacks ──────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.odom_received = True
        self._last_odom_wall_time = time.monotonic()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Publish 3D mesh marker & trajectory for RViz visualization
        self._publish_drone_visuals(msg)

    def _publish_drone_visuals(self, msg: Odometry):
        now = self.get_clock().now().to_msg()

        # 1. 3D Drone Mesh Marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = now
        marker.ns = "s10_drone"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "package://s10_gazebo_sim/models/s10_drone/meshes/x3.dae"
        marker.mesh_use_embedded_materials = True
        marker.pose = msg.pose.pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        self.marker_pub.publish(marker)

        # 2. Flight Trajectory Path (sampled at ~4 Hz)
        self._path_count += 1
        if self._path_count % 5 == 0:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"
            pose_stamped.header.stamp = now
            pose_stamped.pose = msg.pose.pose
            self.flight_path.poses.append(pose_stamped)
            if len(self.flight_path.poses) > 500:
                self.flight_path.poses.pop(0)
            self.flight_path.header.frame_id = "world"
            self.flight_path.header.stamp = now
            self.path_pub.publish(self.flight_path)

    def _wp_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                self.waypoints = data
            elif isinstance(data, dict) and "waypoints" in data:
                self.waypoints = data["waypoints"]
            else:
                self.get_logger().warn("Unexpected waypoint payload format")
                return
            self.wp_index = 0
            self.mission_active = True
            self.landing_active = False
            self.armed = True
            self.get_logger().info(
                f"Received {len(self.waypoints)} waypoints — starting navigation"
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Bad waypoint JSON: {e}")

    # ── Control Loop ───────────────────────────────────────────────

    def _publish_enable(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)

    def _control_loop(self):
        twist = Twist()

        # Never send navigation commands based on the initial placeholder pose.
        if not self.odom_received:
            self.cmd_pub.publish(twist)
            return

        if self.landing_active:
            if self.z > 0.45:
                twist.linear.z = -0.45
                self.cmd_pub.publish(twist)
            else:
                self.landing_active = False
                self.mission_active = False
                self.cmd_pub.publish(Twist())
                self._publish_status("COMPLETED", "All waypoints reached and landing complete")
                self.get_logger().info("Mission completed — drone landed at the launch pad.")
            return

        if not self.mission_active or not self.waypoints:
            self.cmd_pub.publish(twist)
            return

        if self.wp_index >= len(self.waypoints):
            # Continue descending until odometry confirms the vehicle is near
            # the ground. Do not report COMPLETED after a single descent tick.
            self.landing_active = True
            self._publish_status("LANDING", "Mission route complete; descending to launch pad")
            twist.linear.z = -0.45 if self.z > 0.45 else 0.0
            self.cmd_pub.publish(twist)
            if self.z <= 0.45:
                self.landing_active = False
                self.mission_active = False
                self._publish_status("COMPLETED", "All waypoints reached and landing complete")
            return

        wp = self.waypoints[self.wp_index]
        tx, ty, tz = float(wp["x"]), float(wp["y"]), float(wp["z"])

        # Loiter handling
        now = time.time()
        if self.loiter_until > now:
            remaining = self.loiter_until - now
            self._publish_status("LOITERING", f"at {wp['id']} ({remaining:.1f}s remaining)")
            self._loop_count += 1
            if self._loop_count % 20 == 0:
                self.get_logger().info(f"Loitering at {wp['id']} ({remaining:.1f}s remaining)")
            self.cmd_pub.publish(twist)
            return

        # Distance to target
        dx = tx - self.x
        dy = ty - self.y
        dz = tz - self.z
        dist_xy = math.sqrt(dx * dx + dy * dy)
        dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist_3d < self.accept_r:
            # Arrived
            loiter = wp.get("loiter", 0)
            if loiter > 0 and self.loiter_until < 1.0:
                self.loiter_until = now + loiter
                self._publish_status("ARRIVED", f"loitering {loiter}s at {wp['id']}")
                self.get_logger().info(f"✓ Arrived at {wp['id']} — loitering for {loiter}s")
                self.cmd_pub.publish(twist)
                return

            self.loiter_until = 0.0
            self._publish_status("WAYPOINT_REACHED", wp["id"])
            self.get_logger().info(f"✓ Reached waypoint {wp['id']}")
            self.wp_index += 1
            self.cmd_pub.publish(twist)
            return

        # Proportional velocity command (clamped)
        speed = min(self.cruise_speed, dist_3d * 0.8)
        speed = min(speed, self.max_speed)

        if dist_3d > 0.01:
            world_vx = dx / dist_3d * speed
            world_vy = dy / dist_3d * speed
            world_vz = dz / dist_3d * speed

            # Transform horizontal velocity from world to drone body frame
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            twist.linear.x = cos_yaw * world_vx + sin_yaw * world_vy
            twist.linear.y = -sin_yaw * world_vx + cos_yaw * world_vy
            twist.linear.z = world_vz

            # Yaw steering towards waypoint
            if dist_xy > 1.0:
                desired_yaw = math.atan2(dy, dx)
                yaw_diff = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))
                twist.angular.z = max(-1.0, min(1.0, 1.2 * yaw_diff))
            else:
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        self._publish_status(
            "NAVIGATING",
            f"→ {wp['id']}  dist={dist_3d:.1f}m"
        )

        self._loop_count += 1
        if self._loop_count % 20 == 0:
            self.get_logger().info(
                f"Navigating → {wp['id']} | pos: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) | dist: {dist_3d:.2f}m"
            )

    # ── Helpers ────────────────────────────────────────────────────

    def _publish_status(self, state: str, detail: str):
        msg = String()
        wp_id = ""
        if self.waypoints and self.wp_index < len(self.waypoints):
            wp_id = self.waypoints[self.wp_index].get("id", "")
        if state == "WAYPOINT_REACHED":
            wp_id = detail

        msg.data = json.dumps({
            "state": state,
            "detail": detail,
            "wp_id": wp_id,
            "wp_index": self.wp_index,
            "total_wps": len(self.waypoints),
            "position": {"x": round(self.x, 2), "y": round(self.y, 2), "z": round(self.z, 2)},
            "timestamp": time.time(),
        })
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
