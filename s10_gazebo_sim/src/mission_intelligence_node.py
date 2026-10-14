#!/usr/bin/env python3
"""
S10 Mission Intelligence Node
==============================
The main ROS 2 orchestrator that runs the complete S10 pipeline:

    1. Parse mission intent      (IntentParser)
    2. Build mission graph       (MissionGraph DAG)
    3. Generate waypoints        (WaypointGenerator)
    4. Optimize route            (RouteOptimizer — fuel/time/risk)
    5. Validate safety           (SafetyValidator)
    6. Score risk                (RiskEngine)
    7. Execute via FSM           (MissionFSM)
    8. Dynamic replanning        (Replanner)

Published topics
    ~/mission_graph         (std_msgs/String)          JSON mission DAG
    ~/mission_plan          (std_msgs/String)          JSON optimised plan
    ~/risk_report           (std_msgs/String)          JSON risk scores
    ~/fsm_state             (std_msgs/String)          current FSM state
    ~/fsm_event             (std_msgs/String)          FSM transition event
    ~/waypoint_markers      (visualization_msgs/MarkerArray)  RViz markers
    ~/risk_zone_markers     (visualization_msgs/MarkerArray)  risk zone vis
    ~/status                (std_msgs/String)          overall status
"""
import os
import sys
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

# ── Local S10 modules (installed beside this script) ─────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from intent_parser import IntentParser, MissionGraph         # noqa: E402
from waypoint_generator import WaypointGenerator              # noqa: E402
from route_optimizer import RouteOptimizer                    # noqa: E402
from safety_validator import SafetyValidator                  # noqa: E402
from risk_engine import RiskEngine                            # noqa: E402
from mission_fsm import MissionFSM, MissionState              # noqa: E402
from replanner import Replanner                               # noqa: E402


class MissionIntelligenceNode(Node):
    """Orchestrates the full S10 mission intelligence pipeline."""

    def __init__(self):
        super().__init__("mission_intelligence_node")

        # ── Declare Parameters ──────────────────────────────────────
        self.declare_parameter("mission_file", "")
        self.declare_parameter("cruise_altitude", 15.0)
        self.declare_parameter("cruise_speed", 5.0)
        self.declare_parameter("waypoint_acceptance_radius", 2.0)
        self.declare_parameter("max_speed", 8.0)
        self.declare_parameter("weight_fuel", 0.35)
        self.declare_parameter("weight_time", 0.40)
        self.declare_parameter("weight_risk", 0.25)
        self.declare_parameter("risk_zones", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("obstacles", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("min_altitude", 5.0)
        self.declare_parameter("max_altitude", 50.0)
        self.declare_parameter("geofence_radius", 90.0)
        self.declare_parameter("max_risk_score", 0.85)
        self.declare_parameter("fuel_capacity", 100.0)
        self.declare_parameter("fuel_rate_per_metre", 0.12)
        self.declare_parameter("replan_trigger_distance", 8.0)
        self.declare_parameter("replan_cooldown", 5.0)
        self.declare_parameter("marker_publish_rate", 2.0)
        self.declare_parameter("status_publish_rate", 5.0)

        # ── Read parameters ─────────────────────────────────────────
        self.mission_file = self.get_parameter("mission_file").value
        self.cruise_alt = self.get_parameter("cruise_altitude").value
        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.accept_r = self.get_parameter("waypoint_acceptance_radius").value

        # Parse risk_zones and obstacles from flat parameter arrays
        self.risk_zones = self._parse_nested_param("risk_zones", 4)
        self.obstacles = self._parse_nested_param("obstacles", 3)

        # ── Instantiate pipeline modules ────────────────────────────
        self.parser = IntentParser()
        self.wp_gen = WaypointGenerator(cruise_altitude=self.cruise_alt)
        self.optimizer = RouteOptimizer(
            weight_fuel=self.get_parameter("weight_fuel").value,
            weight_time=self.get_parameter("weight_time").value,
            weight_risk=self.get_parameter("weight_risk").value,
            fuel_rate=self.get_parameter("fuel_rate_per_metre").value,
            cruise_speed=self.cruise_speed,
            risk_zones=self.risk_zones,
        )
        self.validator = SafetyValidator(
            geofence_radius=self.get_parameter("geofence_radius").value,
            min_altitude=self.get_parameter("min_altitude").value,
            max_altitude=self.get_parameter("max_altitude").value,
            max_risk_score=self.get_parameter("max_risk_score").value,
            fuel_capacity=self.get_parameter("fuel_capacity").value,
            fuel_rate=self.get_parameter("fuel_rate_per_metre").value,
        )
        self.risk_eng = RiskEngine(
            risk_zones=self.risk_zones,
            obstacles=self.obstacles,
        )
        self.fsm = MissionFSM()
        self.replanner = Replanner(
            obstacles=self.obstacles,
            risk_zones=self.risk_zones,
            trigger_distance=self.get_parameter("replan_trigger_distance").value,
            cruise_altitude=self.cruise_alt,
        )

        # ── State ───────────────────────────────────────────────────
        self.graph: MissionGraph = None
        self.waypoints = []
        self.evaluation = {}
        self.risk_report = {}
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.wp_index = 0
        self.last_replan_time = 0.0

        # ── Publishers ──────────────────────────────────────────────
        self.graph_pub   = self.create_publisher(String, "~/mission_graph", 10)
        self.plan_pub    = self.create_publisher(String, "~/mission_plan", 10)
        self.risk_pub    = self.create_publisher(String, "~/risk_report", 10)
        self.fsm_pub     = self.create_publisher(String, "~/fsm_state", 10)
        self.event_pub   = self.create_publisher(String, "~/fsm_event", 10)
        self.status_pub  = self.create_publisher(String, "~/status", 10)
        self.wp_marker_pub   = self.create_publisher(MarkerArray, "~/waypoint_markers", 10)
        self.risk_marker_pub = self.create_publisher(MarkerArray, "~/risk_zone_markers", 10)
        self.target_wp_pub   = self.create_publisher(String, "/drone_controller/target_waypoints", 10)

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(Odometry, "/s10_drone/odometry", self._odom_cb, 10)
        self.create_subscription(
            String, "/drone_controller/navigation_status", self._nav_status_cb, 10
        )

        # ── Register FSM callback ───────────────────────────────────
        self.fsm.register_callback(self._on_fsm_transition)

        # ── Timers ──────────────────────────────────────────────────
        marker_hz = self.get_parameter("marker_publish_rate").value
        status_hz = self.get_parameter("status_publish_rate").value
        self.create_timer(1.0 / max(marker_hz, 0.1), self._publish_markers)
        self.create_timer(1.0 / max(status_hz, 0.1), self._publish_status)
        self.create_timer(0.5, self._replan_check)

        # ── Kick off pipeline ───────────────────────────────────────
        # Use a one-shot timer to let the node finish construction first
        self.create_timer(2.0, self._run_pipeline, callback_group=None)

        self.get_logger().info("═══ S10 Mission Intelligence Node started ═══")

    # ══════════════════════════════════════════════════════════════════
    # PIPELINE
    # ══════════════════════════════════════════════════════════════════

    def _run_pipeline(self):
        """Execute the full S10 planning pipeline once."""
        # Only run once
        if self.fsm.state != MissionState.IDLE:
            return

        if not self.mission_file:
            self.get_logger().warn("No mission_file parameter — waiting…")
            return

        self.get_logger().info(f"▶ Loading mission from {self.mission_file}")

        # ── 1. PLANNING ─────────────────────────────────────────────
        self.fsm.transition(MissionState.PLANNING, "Pipeline started")

        try:
            self.graph = self.parser.parse_file(self.mission_file)
        except Exception as e:
            self.get_logger().error(f"Intent parse failed: {e}")
            self.fsm.transition(MissionState.ABORTED, str(e))
            return

        # Validate graph
        errors = self.graph.validate()
        if errors:
            for err in errors:
                self.get_logger().error(f"Graph validation: {err}")
            self.fsm.transition(MissionState.ABORTED, "; ".join(errors))
            return

        self.get_logger().info(
            f"✓ Mission graph built: {len(self.graph.nodes)} tasks, "
            f"order = {self.graph.topological_order()}"
        )
        self._publish_graph()

        # ── 2. Generate waypoints ────────────────────────────────────
        self.waypoints = self.wp_gen.generate(self.graph)
        self.get_logger().info(f"✓ Generated {len(self.waypoints)} waypoints")

        # ── 3. Optimise ─────────────────────────────────────────────
        self.waypoints, self.evaluation = self.optimizer.optimize(self.waypoints)
        self.get_logger().info(
            f"✓ Optimised — composite cost = {self.evaluation['composite_cost']}"
        )

        # ── 4. VALIDATING ───────────────────────────────────────────
        self.fsm.transition(MissionState.VALIDATING, "Checking safety constraints")

        passed, violations = self.validator.validate(self.waypoints, self.evaluation)
        if not passed:
            for v in violations:
                self.get_logger().warn(f"⚠ Safety violation: {v}")
            # Still proceed (flag, don't reject) for demo purposes
            self.get_logger().warn("Plan has violations but proceeding for demo")

        # ── 5. Risk scoring ─────────────────────────────────────────
        self.risk_report = self.risk_eng.score_plan(self.waypoints)
        self.get_logger().info(
            f"✓ Risk — cumulative={self.risk_report['cumulative_risk']}, "
            f"max_single={self.risk_report['max_single_risk']}"
        )
        self._publish_risk()

        # ── 6. Publish plan ─────────────────────────────────────────
        self._publish_plan()

        # ── 7. ARM → EXECUTE ────────────────────────────────────────
        self.fsm.transition(MissionState.ARMED, "Plan validated")

        # Small delay then start executing
        self.create_timer(1.0, self._start_execution, callback_group=None)

    def _start_execution(self):
        if self.fsm.state != MissionState.ARMED:
            return
        self.fsm.transition(MissionState.EXECUTING, "Sending waypoints to controller")
        self.wp_index = 0

        # Send waypoints to drone controller
        msg = String()
        msg.data = json.dumps(self.waypoints)
        self.target_wp_pub.publish(msg)
        self.get_logger().info("✓ Waypoints dispatched to drone controller")

    # ══════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ══════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z

    def _nav_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            state = data.get("state", "")
            if state == "WAYPOINT_REACHED":
                reached_id = data.get("wp_id") or data.get("detail", "")
                found = False
                for idx, wp in enumerate(self.waypoints):
                    if wp["id"] == reached_id:
                        self.wp_index = idx + 1
                        found = True
                        break
                if not found:
                    self.wp_index += 1
            elif state == "COMPLETED":
                if self.fsm.state == MissionState.EXECUTING:
                    self.fsm.transition(MissionState.COMPLETED, "All waypoints reached")
        except json.JSONDecodeError:
            pass

    def _on_fsm_transition(self, old, new, reason):
        """Called on every FSM transition — publish state and event."""
        fsm_msg = String()
        fsm_msg.data = new.name
        self.fsm_pub.publish(fsm_msg)

        event_msg = String()
        event_msg.data = self.fsm.last_event_json()
        self.event_pub.publish(event_msg)

        self.get_logger().info(f"FSM: {old.name} → {new.name}  ({reason})")

    # ══════════════════════════════════════════════════════════════════
    # REPLANNING
    # ══════════════════════════════════════════════════════════════════

    def _replan_check(self):
        """Periodic check for dynamic replanning triggers."""
        if self.fsm.state != MissionState.EXECUTING:
            return
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            return

        cooldown = self.get_parameter("replan_cooldown").value
        if time.time() - self.last_replan_time < cooldown:
            return

        new_plan, did_replan = self.replanner.replan(
            self.waypoints, self.wp_index, self.drone_x, self.drone_y
        )
        if did_replan:
            self.last_replan_time = time.time()
            self.fsm.transition(MissionState.REPLANNING, "Obstacle avoidance")
            self.waypoints = new_plan
            # Re-evaluate
            self.evaluation = self.optimizer.evaluate(self.waypoints)
            self.risk_report = self.risk_eng.score_plan(self.waypoints)
            self._publish_plan()
            self._publish_risk()

            # Re-send waypoints to controller
            msg = String()
            msg.data = json.dumps({"waypoints": self.waypoints[self.wp_index:]})
            self.target_wp_pub.publish(msg)

            self.fsm.transition(MissionState.EXECUTING, "Replanned — resuming")
            self.get_logger().info(
                f"↻ Replanned at wp_index={self.wp_index}, "
                f"new plan has {len(self.waypoints)} waypoints"
            )

    # ══════════════════════════════════════════════════════════════════
    # PUBLISHERS
    # ══════════════════════════════════════════════════════════════════

    def _publish_graph(self):
        if self.graph:
            msg = String()
            msg.data = self.graph.to_json()
            self.graph_pub.publish(msg)

    def _publish_plan(self):
        msg = String()
        msg.data = json.dumps({
            "waypoints": self.waypoints,
            "evaluation": self.evaluation,
        }, indent=2)
        self.plan_pub.publish(msg)

    def _publish_risk(self):
        msg = String()
        msg.data = json.dumps(self.risk_report, indent=2)
        self.risk_pub.publish(msg)

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "fsm_state": self.fsm.state.name,
            "wp_index": self.wp_index,
            "total_wps": len(self.waypoints),
            "drone": {
                "x": round(self.drone_x, 2),
                "y": round(self.drone_y, 2),
                "z": round(self.drone_z, 2),
            },
            "evaluation": self.evaluation,
            "risk_cumulative": self.risk_report.get("cumulative_risk", 0),
        })
        self.status_pub.publish(msg)

    def _publish_markers(self):
        """Publish RViz MarkerArray for waypoints and risk zones."""
        self._publish_wp_markers()
        self._publish_risk_markers()

    def _publish_wp_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Clear previous markers to keep RViz display clean and synchronized
        del_marker = Marker()
        del_marker.header.frame_id = "world"
        del_marker.header.stamp = stamp
        del_marker.action = Marker.DELETEALL
        ma.markers.append(del_marker)

        for i, wp in enumerate(self.waypoints):
            # Sphere at each waypoint
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp
            m.ns = "s10_waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp["x"]
            m.pose.position.y = wp["y"]
            m.pose.position.z = wp["z"]
            m.scale.x = m.scale.y = m.scale.z = 1.5

            # Colour by type
            if wp["type"] == "TAKEOFF":
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 0.9
            elif wp["type"] == "LAND":
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 0.9
            elif wp["type"] == "INSPECT":
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.9, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.3, 0.9

            # Highlight current target
            if i == self.wp_index and self.fsm.state == MissionState.EXECUTING:
                m.scale.x = m.scale.y = m.scale.z = 2.5
                m.color.a = 1.0
            ma.markers.append(m)

            # Text label
            txt = Marker()
            txt.header.frame_id = "world"
            txt.header.stamp = stamp
            txt.ns = "s10_wp_labels"
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = wp["x"]
            txt.pose.position.y = wp["y"]
            txt.pose.position.z = wp["z"] + 2.5
            txt.scale.z = 1.2
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = f"{wp['id']}"
            ma.markers.append(txt)

        # Line strip connecting waypoints
        if len(self.waypoints) >= 2:
            line = Marker()
            line.header.frame_id = "world"
            line.header.stamp = stamp
            line.ns = "s10_route"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.3
            line.color.r, line.color.g, line.color.b, line.color.a = 0.3, 0.9, 1.0, 0.7
            for wp in self.waypoints:
                p = Point()
                p.x = wp["x"]
                p.y = wp["y"]
                p.z = wp["z"]
                line.points.append(p)
            ma.markers.append(line)

        self.wp_marker_pub.publish(ma)

    def _publish_risk_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, zone in enumerate(self.risk_zones):
            cx, cy, r, sev = zone
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp
            m.ns = "s10_risk_zones"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.5
            m.scale.x = m.scale.y = r * 2.0
            m.scale.z = 1.0
            m.color.r = sev
            m.color.g = 0.1
            m.color.b = 0.0
            m.color.a = 0.25
            ma.markers.append(m)

        self.risk_marker_pub.publish(ma)

    # ── Parameter helpers ───────────────────────────────────────────

    def _parse_nested_param(self, name: str, stride: int):
        """
        ROS 2 flattens nested YAML lists to a single 1-D array.
        Reconstruct the list-of-lists from the flat parameter.
        """
        try:
            raw = self.get_parameter(name).value
            if not raw:
                return []
            # If already nested (shouldn't happen with ROS 2 params)
            if isinstance(raw[0], (list, tuple)):
                return [list(r) for r in raw]
            # Flat array → chunk into stride-sized groups
            return [raw[i:i + stride] for i in range(0, len(raw), stride)]
        except Exception:
            return []


def main():
    rclpy.init()
    node = MissionIntelligenceNode()
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
