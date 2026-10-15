#!/usr/bin/env python3
"""
S10 Replanner
=============
Monitors the drone's live position against obstacles and risk zones.
When a trigger condition is met (e.g., drone too close to an obstacle),
the replanner generates an avoidance waypoint and splices it into the
active plan.

Trigger conditions
    • Obstacle proximity  < replan_trigger_distance
    • Risk zone entry     (severity > threshold)
"""
import math
from typing import List, Dict, Optional, Tuple


class Replanner:
    """Dynamic mission replanner for obstacle avoidance and risk mitigation."""

    def __init__(
        self,
        obstacles: List[List[float]] = None,
        risk_zones: List[List[float]] = None,
        trigger_distance: float = 8.0,
        cruise_altitude: float = 15.0,
    ):
        self.obstacles = obstacles or []
        self.risk_zones = risk_zones or []
        self.trigger_distance = trigger_distance
        self.cruise_alt = cruise_altitude

    def check_replan_needed(
        self,
        drone_x: float,
        drone_y: float,
        next_wp: Dict,
    ) -> Tuple[bool, str]:
        """
        Check whether a replan is needed based on the segment from
        (drone_x, drone_y) toward next_wp.

        Returns (needs_replan: bool, reason: str).
        """
        # Never replan an avoidance waypoint into another avoidance waypoint
        if next_wp.get("id", "").startswith("avoid_"):
            return False, "Already executing avoidance waypoint"

        # Do not replan takeoff or landing
        if next_wp.get("type") in ("TAKEOFF", "LAND"):
            return False, "Takeoff and Land waypoints are not replanned"

        x1, y1 = drone_x, drone_y
        x2, y2 = float(next_wp["x"]), float(next_wp["y"])
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 0.25:
            return False, ""

        # Check line segment against obstacles (perpendicular clearance)
        for obs in self.obstacles:
            cx, cy, hw = obs[0], obs[1], obs[2]
            t = max(0.0, min(1.0, ((cx - x1) * dx + (cy - y1) * dy) / l2))
            px = x1 + t * dx
            py = y1 + t * dy
            d = math.sqrt((cx - px) ** 2 + (cy - py) ** 2)
            clearance = d - hw
            if clearance < self.trigger_distance:
                return True, f"Obstacle at ({cx}, {cy}) clearance={clearance:.1f}m < {self.trigger_distance}m"

        # Check against high-severity risk zones
        for zone in self.risk_zones:
            cx, cy, r, sev = zone[0], zone[1], zone[2], zone[3]
            if sev < 0.6:
                continue
            mx = (x1 + x2) / 2.0
            my = (y1 + y2) / 2.0
            d = math.sqrt((mx - cx) ** 2 + (my - cy) ** 2)
            if d < r * 0.5:
                return True, f"Deep inside risk zone ({cx}, {cy}) severity={sev}"

        return False, ""

    def generate_avoidance_waypoint(
        self,
        drone_x: float,
        drone_y: float,
        next_wp: Dict,
    ) -> Optional[Dict]:
        """
        Create a single avoidance waypoint that routes the drone around
        the nearest obstacle threatening the current flight segment.
        """
        x1, y1 = drone_x, drone_y
        x2, y2 = float(next_wp["x"]), float(next_wp["y"])
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 0.25:
            return None

        closest_obs = None
        min_clearance = float("inf")
        proj_point = (x1, y1)

        for obs in self.obstacles:
            cx, cy, hw = obs[0], obs[1], obs[2]
            t = max(0.0, min(1.0, ((cx - x1) * dx + (cy - y1) * dy) / l2))
            px = x1 + t * dx
            py = y1 + t * dy
            d = math.sqrt((cx - px) ** 2 + (cy - py) ** 2)
            clearance = d - hw
            if clearance < min_clearance:
                min_clearance = clearance
                closest_obs = obs
                proj_point = (px, py)

        if closest_obs is None or min_clearance >= self.trigger_distance:
            return None

        cx, cy, hw = closest_obs[0], closest_obs[1], closest_obs[2]
        seg_len = math.sqrt(l2)
        # Unit perpendicular vector
        perp_x = -dy / seg_len
        perp_y = dx / seg_len

        # Offset by obstacle half-width plus 8m safety buffer
        buffer = hw + 8.0
        opt_a = (proj_point[0] + perp_x * buffer, proj_point[1] + perp_y * buffer)
        opt_b = (proj_point[0] - perp_x * buffer, proj_point[1] - perp_y * buffer)

        da = math.sqrt((opt_a[0] - cx) ** 2 + (opt_a[1] - cy) ** 2)
        db = math.sqrt((opt_b[0] - cx) ** 2 + (opt_b[1] - cy) ** 2)

        chosen = opt_a if da > db else opt_b

        return {
            "id": f"avoid_{next_wp['id']}",
            "type": "NAVIGATE",
            "x": round(chosen[0], 2),
            "y": round(chosen[1], 2),
            "z": self.cruise_alt,
            "loiter": 0,
        }

    def replan(
        self,
        waypoints: List[Dict],
        current_index: int,
        drone_x: float,
        drone_y: float,
    ) -> Tuple[List[Dict], bool]:
        """
        Check the next waypoint; if threatened, splice in an avoidance
        waypoint and return the updated plan.

        Returns (updated_waypoints, did_replan).
        """
        if current_index >= len(waypoints):
            return waypoints, False

        next_wp = waypoints[current_index]

        # Never replan an avoidance waypoint
        if next_wp.get("id", "").startswith("avoid_"):
            return waypoints, False

        # Never duplicate avoidance if already inserted right before this target
        if current_index > 0 and waypoints[current_index - 1].get("id", "") == f"avoid_{next_wp['id']}":
            return waypoints, False

        needs, reason = self.check_replan_needed(drone_x, drone_y, next_wp)
        if not needs:
            return waypoints, False

        avoid_wp = self.generate_avoidance_waypoint(drone_x, drone_y, next_wp)
        if avoid_wp is None:
            return waypoints, False

        # Splice the avoidance waypoint before the current target
        new_plan = list(waypoints)
        new_plan.insert(current_index, avoid_wp)
        return new_plan, True
