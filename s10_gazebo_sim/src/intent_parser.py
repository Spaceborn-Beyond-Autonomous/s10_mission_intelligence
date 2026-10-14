#!/usr/bin/env python3
"""
S10 Intent Parser
=================
Converts a YAML mission-intent file into a structured mission graph (DAG).
Each task becomes a node; edges encode dependencies.

Published topics
    ~/mission_graph   (std_msgs/String)   — JSON-serialised mission graph
"""
import json
import yaml
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional


# ── Data classes ────────────────────────────────────────────────────────────

@dataclass
class TaskNode:
    """Single node in the mission graph."""
    id: str
    task_type: str                      # TAKEOFF | NAVIGATE | INSPECT | LAND
    description: str = ""
    depends_on: List[str] = field(default_factory=list)
    params: Dict = field(default_factory=dict)
    status: str = "PENDING"             # PENDING → ACTIVE → DONE | SKIPPED

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class MissionGraph:
    """Directed acyclic graph of mission tasks."""
    mission_id: str
    name: str
    description: str = ""
    priority: str = "NORMAL"
    time_limit: float = 600.0
    nodes: Dict[str, TaskNode] = field(default_factory=dict)
    objectives: List[Dict] = field(default_factory=list)
    constraints: Dict = field(default_factory=dict)

    # ── graph helpers ───────────────────────────────────────────────

    def topological_order(self) -> List[str]:
        """Return task IDs in dependency-respecting order (Kahn's algorithm)."""
        in_deg = {nid: 0 for nid in self.nodes}
        adj: Dict[str, List[str]] = {nid: [] for nid in self.nodes}
        for nid, node in self.nodes.items():
            for dep in node.depends_on:
                if dep in adj:
                    adj[dep].append(nid)
                    in_deg[nid] += 1

        queue = [nid for nid, d in in_deg.items() if d == 0]
        order: List[str] = []
        while queue:
            curr = queue.pop(0)
            order.append(curr)
            for child in adj[curr]:
                in_deg[child] -= 1
                if in_deg[child] == 0:
                    queue.append(child)
        return order

    def validate(self) -> List[str]:
        """Return a list of validation errors (empty = valid)."""
        errors: List[str] = []
        order = self.topological_order()
        if len(order) != len(self.nodes):
            errors.append("Mission graph contains a cycle")
        for nid, node in self.nodes.items():
            for dep in node.depends_on:
                if dep not in self.nodes:
                    errors.append(f"Task '{nid}' depends on unknown task '{dep}'")
        if not any(n.task_type == "TAKEOFF" for n in self.nodes.values()):
            errors.append("Mission graph has no TAKEOFF task")
        if not any(n.task_type == "LAND" for n in self.nodes.values()):
            errors.append("Mission graph has no LAND task")
        return errors

    def to_dict(self) -> dict:
        return {
            "mission_id": self.mission_id,
            "name": self.name,
            "description": self.description,
            "priority": self.priority,
            "time_limit": self.time_limit,
            "nodes": {nid: n.to_dict() for nid, n in self.nodes.items()},
            "order": self.topological_order(),
            "objectives": self.objectives,
            "constraints": self.constraints,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)


# ── Parser ──────────────────────────────────────────────────────────────────

class IntentParser:
    """Parse a YAML mission-intent document into a MissionGraph."""

    @staticmethod
    def parse_file(path: str) -> MissionGraph:
        with open(path, "r") as fh:
            raw = yaml.safe_load(fh)
        return IntentParser.parse_dict(raw)

    @staticmethod
    def parse_dict(raw: dict) -> MissionGraph:
        m = raw.get("mission", raw)
        graph = MissionGraph(
            mission_id=m.get("id", "UNKNOWN"),
            name=m.get("name", "Unnamed Mission"),
            description=m.get("description", ""),
            priority=m.get("priority", "NORMAL"),
            time_limit=float(m.get("time_limit_seconds", 600)),
            objectives=m.get("objectives", []),
            constraints=m.get("constraints", {}),
        )
        for t in m.get("tasks", []):
            node = TaskNode(
                id=t["id"],
                task_type=t.get("type", "NAVIGATE"),
                description=t.get("description", ""),
                depends_on=t.get("depends_on", []),
                params=t.get("params", {}),
            )
            graph.nodes[node.id] = node
        return graph
