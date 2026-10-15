#!/usr/bin/env python3
"""
S10 Safety Validator
====================
Validates a waypoint plan against configurable safety constraints:
    • Geofence (max distance from origin)
    • Altitude bounds
    • Cumulative risk threshold
    • Fuel capacity

Returns PASS / FAIL with a list of violations.
"""
import math
from typing import List, Dict, Tuple


class SafetyValidator:
    """Check a mission plan against safety constraints."""

    def __init__(
        self,
        geofence_radius: float = 90.0,
        min_altitude: float = 5.0,
        max_altitude: float = 50.0,
        max_risk_score: float = 0.85,
        fuel_capacity: float = 100.0,
        fuel_rate: float = 0.12,
    ):
        self.geofence_radius = geofence_radius
        self.min_alt = min_altitude
        self.max_alt = max_altitude
        self.max_risk = max_risk_score
        self.fuel_cap = fuel_capacity
        self.fuel_rate = fuel_rate

    def validate(
        self,
        waypoints: List[Dict],
        evaluation: Dict,
    ) -> Tuple[bool, List[str]]:
        """
        Validate the plan.

        Parameters
            waypoints   — ordered list of waypoint dicts
            evaluation  — output of RouteOptimizer.evaluate()

        Returns
            (passed: bool, violations: list[str])
        """
        violations: List[str] = []

        # ── Geofence ───────────────────────────────────────────────
        for wp in waypoints:
            d = math.sqrt(wp["x"] ** 2 + wp["y"] ** 2)
            if d > self.geofence_radius:
                violations.append(
                    f"Waypoint '{wp['id']}' at ({wp['x']:.1f}, {wp['y']:.1f}) "
                    f"is {d:.1f} m from origin — exceeds geofence {self.geofence_radius} m"
                )

        # ── Altitude ───────────────────────────────────────────────
        for wp in waypoints:
            z = wp["z"]
            if wp["type"] == "LAND":
                continue  # landing at z=0 is fine
            if z < self.min_alt:
                violations.append(
                    f"Waypoint '{wp['id']}' altitude {z:.1f} m < minimum {self.min_alt} m"
                )
            if z > self.max_alt:
                violations.append(
                    f"Waypoint '{wp['id']}' altitude {z:.1f} m > maximum {self.max_alt} m"
                )

        # ── Risk ───────────────────────────────────────────────────
        risk = evaluation.get("risk_raw", 0.0)
        if risk > self.max_risk:
            violations.append(
                f"Cumulative risk {risk:.4f} exceeds threshold {self.max_risk}"
            )

        # ── Fuel ───────────────────────────────────────────────────
        fuel = evaluation.get("fuel_raw", 0.0)
        if fuel > self.fuel_cap:
            violations.append(
                f"Estimated fuel usage {fuel:.1f} exceeds capacity {self.fuel_cap}"
            )

        passed = len(violations) == 0
        return passed, violations
