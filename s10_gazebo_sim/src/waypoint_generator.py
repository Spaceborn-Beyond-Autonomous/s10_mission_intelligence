#!/usr/bin/env python3
"""
S10 Waypoint Generator
======================
Converts the mission graph's topological task order into an ordered sequence of
3-D waypoints (x, y, z) suitable for the drone controller.

Published topics
    ~/waypoints   (nav_msgs/Path)
"""
import math
from typing import List, Tuple

# Type alias
Waypoint = Tuple[float, float, float]   # (x, y, z)


class WaypointGenerator:
    """Generate a waypoint sequence from a mission graph."""

    def __init__(self, cruise_altitude: float = 15.0):
        self.cruise_altitude = cruise_altitude

    def generate(self, graph) -> List[dict]:
        """
        Walk the graph in topological order and extract waypoints.

        Returns a list of dicts:
            { "id": task_id, "type": task_type, "x": …, "y": …, "z": …,
              "loiter": seconds_or_0 }
        """
        waypoints: List[dict] = []
        order = graph.topological_order()

        for task_id in order:
            node = graph.nodes[task_id]
            target = node.params.get("target")

            if node.task_type == "TAKEOFF":
                alt = node.params.get("target_altitude", self.cruise_altitude)
                waypoints.append({
                    "id": task_id,
                    "type": node.task_type,
                    "x": 0.0,
                    "y": 0.0,
                    "z": alt,
                    "loiter": 0,
                })

            elif node.task_type in ("NAVIGATE", "INSPECT"):
                if target and len(target) >= 3:
                    x, y, z = float(target[0]), float(target[1]), float(target[2])
                elif target and len(target) == 2:
                    x, y = float(target[0]), float(target[1])
                    z = self.cruise_altitude
                else:
                    continue  # skip nodes without valid targets
                loiter = int(node.params.get("loiter_seconds", 0))
                waypoints.append({
                    "id": task_id,
                    "type": node.task_type,
                    "x": x,
                    "y": y,
                    "z": z,
                    "loiter": loiter,
                })

            elif node.task_type == "LAND":
                if target and len(target) >= 2:
                    x, y = float(target[0]), float(target[1])
                else:
                    x, y = 0.0, 0.0
                waypoints.append({
                    "id": task_id,
                    "type": node.task_type,
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "loiter": 0,
                })

        return waypoints

    # ── helpers ────────────────────────────────────────────────────────

    @staticmethod
    def total_distance(waypoints: List[dict]) -> float:
        """Sum of Euclidean distances between consecutive waypoints."""
        dist = 0.0
        for i in range(1, len(waypoints)):
            a, b = waypoints[i - 1], waypoints[i]
            dist += math.sqrt(
                (b["x"] - a["x"]) ** 2 +
                (b["y"] - a["y"]) ** 2 +
                (b["z"] - a["z"]) ** 2
            )
        return dist

    @staticmethod
    def estimated_time(waypoints: List[dict], speed: float = 5.0) -> float:
        """Flight time estimate in seconds (distance / speed + loiter)."""
        dist = WaypointGenerator.total_distance(waypoints)
        loiter = sum(wp.get("loiter", 0) for wp in waypoints)
        return dist / max(speed, 0.1) + loiter
