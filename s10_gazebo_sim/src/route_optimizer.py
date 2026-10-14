#!/usr/bin/env python3
"""
S10 Route Optimizer
===================
Multi-objective route optimizer that scores candidate waypoint plans on
three objectives — fuel, time, and risk — then picks the Pareto-best plan.

For the MVP the optimiser operates on a single candidate and can also
attempt a local 2-opt improvement before returning the result.
"""
import math
from typing import List, Dict, Tuple

Waypoint = Dict


class RouteOptimizer:
    """Evaluate and improve a waypoint plan using fuel / time / risk weights."""

    def __init__(
        self,
        weight_fuel: float = 0.35,
        weight_time: float = 0.40,
        weight_risk: float = 0.25,
        fuel_rate: float = 0.12,       # fuel units per metre
        cruise_speed: float = 5.0,     # m/s
        risk_zones: List[List[float]] = None,
    ):
        self.w_fuel = weight_fuel
        self.w_time = weight_time
        self.w_risk = weight_risk
        self.fuel_rate = fuel_rate
        self.cruise_speed = cruise_speed
        self.risk_zones = risk_zones or []

    # ── public API ──────────────────────────────────────────────────

    def evaluate(self, waypoints: List[Waypoint]) -> Dict:
        """Return objective scores and composite cost for a plan."""
        fuel = self._fuel_cost(waypoints)
        time = self._time_cost(waypoints)
        risk = self._risk_cost(waypoints)

        # Normalise each objective to [0, 1] range using soft caps
        fuel_norm = min(fuel / 100.0, 1.0)
        time_norm = min(time / 600.0, 1.0)
        risk_norm = min(risk, 1.0)

        composite = (
            self.w_fuel * fuel_norm +
            self.w_time * time_norm +
            self.w_risk * risk_norm
        )

        return {
            "fuel_raw": round(fuel, 2),
            "time_raw": round(time, 2),
            "risk_raw": round(risk, 4),
            "fuel_norm": round(fuel_norm, 4),
            "time_norm": round(time_norm, 4),
            "risk_norm": round(risk_norm, 4),
            "composite_cost": round(composite, 4),
        }

    def optimize(self, waypoints: List[Waypoint]) -> Tuple[List[Waypoint], Dict]:
        """
        Attempt a 2-opt local search on the NAVIGATE segments (preserving
        TAKEOFF at the start and LAND at the end).  Returns the best plan
        and its evaluation.
        """
        if len(waypoints) <= 4:
            return waypoints, self.evaluate(waypoints)

        # Only re-order the middle (navigate/inspect) waypoints
        head = [waypoints[0]]                      # TAKEOFF
        tail = [waypoints[-1]]                      # LAND
        middle = waypoints[1:-1]

        best = list(middle)
        best_cost = self.evaluate(head + best + tail)["composite_cost"]

        improved = True
        while improved:
            improved = False
            for i in range(len(best) - 1):
                for j in range(i + 1, len(best)):
                    candidate = list(best)
                    candidate[i:j + 1] = reversed(candidate[i:j + 1])
                    c = self.evaluate(head + candidate + tail)["composite_cost"]
                    if c < best_cost - 1e-6:
                        best = candidate
                        best_cost = c
                        improved = True

        optimized = head + best + tail
        return optimized, self.evaluate(optimized)

    # ── internal cost models ────────────────────────────────────────

    def _fuel_cost(self, wps: List[Waypoint]) -> float:
        dist = 0.0
        for i in range(1, len(wps)):
            dist += self._dist(wps[i - 1], wps[i])
        return dist * self.fuel_rate

    def _time_cost(self, wps: List[Waypoint]) -> float:
        dist = 0.0
        for i in range(1, len(wps)):
            dist += self._dist(wps[i - 1], wps[i])
        loiter = sum(wp.get("loiter", 0) for wp in wps)
        return dist / max(self.cruise_speed, 0.1) + loiter

    def _risk_cost(self, wps: List[Waypoint]) -> float:
        """Cumulative risk: for each segment, sample the midpoint and
        check proximity to risk zones."""
        total = 0.0
        for i in range(1, len(wps)):
            mx = (wps[i - 1]["x"] + wps[i]["x"]) / 2.0
            my = (wps[i - 1]["y"] + wps[i]["y"]) / 2.0
            seg_len = self._dist(wps[i - 1], wps[i])
            for zone in self.risk_zones:
                cx, cy, r, severity = zone[0], zone[1], zone[2], zone[3]
                d = math.sqrt((mx - cx) ** 2 + (my - cy) ** 2)
                if d < r:
                    # linearly scale: at centre → full severity; at edge → 0
                    total += severity * (1.0 - d / r) * (seg_len / 50.0)
        return min(total, 1.0)

    @staticmethod
    def _dist(a: Waypoint, b: Waypoint) -> float:
        return math.sqrt(
            (b["x"] - a["x"]) ** 2 +
            (b["y"] - a["y"]) ** 2 +
            (b["z"] - a["z"]) ** 2
        )
