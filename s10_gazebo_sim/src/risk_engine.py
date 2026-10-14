#!/usr/bin/env python3
"""
S10 Risk Engine
===============
Compute a per-waypoint and cumulative risk score based on proximity to
known risk zones and obstacle structures.

Risk zones are defined as (cx, cy, radius, severity) tuples.
Obstacles are (cx, cy, half_width) columns.
"""
import math
from typing import List, Dict


class RiskEngine:
    """Score risk for individual waypoints and full plans."""

    def __init__(
        self,
        risk_zones: List[List[float]] = None,
        obstacles: List[List[float]] = None,
    ):
        self.risk_zones = risk_zones or []
        self.obstacles = obstacles or []

    def score_waypoint(self, x: float, y: float) -> float:
        """Return risk ∈ [0, 1] for a single (x, y) position."""
        risk = 0.0

        # Risk zones
        for zone in self.risk_zones:
            cx, cy, r, severity = zone[0], zone[1], zone[2], zone[3]
            d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            if d < r:
                risk += severity * (1.0 - d / r)

        # Obstacle proximity (inversely proportional within 2× half-width)
        for obs in self.obstacles:
            cx, cy, hw = obs[0], obs[1], obs[2]
            d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            danger_radius = hw * 2.0
            if d < danger_radius:
                risk += 0.3 * (1.0 - d / danger_radius)

        return min(risk, 1.0)

    def score_plan(self, waypoints: List[Dict]) -> Dict:
        """Return per-waypoint scores and a cumulative risk score."""
        per_wp: List[Dict] = []
        total = 0.0

        for wp in waypoints:
            s = self.score_waypoint(wp["x"], wp["y"])
            per_wp.append({"id": wp["id"], "risk": round(s, 4)})
            total += s

        # Normalise cumulative to [0, 1]
        n = max(len(waypoints), 1)
        cumulative = min(total / n, 1.0)

        return {
            "per_waypoint": per_wp,
            "cumulative_risk": round(cumulative, 4),
            "max_single_risk": round(max((w["risk"] for w in per_wp), default=0.0), 4),
        }
