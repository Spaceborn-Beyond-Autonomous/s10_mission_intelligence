"""Regression tests for the S10 mission-intelligence pipeline (no ROS required)."""
import importlib.util
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
CONFIG = ROOT / "config"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, SRC / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


intent_parser = load_module("intent_parser")
waypoint_generator = load_module("waypoint_generator")
route_optimizer = load_module("route_optimizer")
safety_validator = load_module("safety_validator")
risk_engine = load_module("risk_engine")
mission_fsm = load_module("mission_fsm")


class MissionPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.graph = intent_parser.IntentParser.parse_file(str(CONFIG / "sample_mission.yaml"))
        cls.waypoints = waypoint_generator.WaypointGenerator().generate(cls.graph)
        cls.zones = [[30.0, 30.0, 15.0, 0.8], [-30.0, -25.0, 12.0, 0.6], [-40.0, 40.0, 10.0, 0.4]]
        cls.obstacles = [[20.0, 15.0, 5.0], [-25.0, 30.0, 6.0], [35.0, -20.0, 6.0],
                         [-15.0, -35.0, 5.5], [50.0, 50.0, 7.0], [-50.0, -10.0, 7.0]]

    def test_sample_graph_is_valid(self):
        self.assertEqual(self.graph.validate(), [])
        self.assertEqual(len(self.graph.nodes), 10)

    def test_waypoint_generation_keeps_task_order(self):
        self.assertEqual(len(self.waypoints), 10)
        self.assertEqual(self.waypoints[0]["type"], "TAKEOFF")
        self.assertEqual(self.waypoints[-1]["type"], "LAND")
        self.assertEqual([w["id"] for w in self.waypoints], self.graph.topological_order())

    def test_optimizer_does_not_break_semantic_order(self):
        optimizer = route_optimizer.RouteOptimizer(risk_zones=self.zones)
        optimized, evaluation = optimizer.optimize(self.waypoints)
        self.assertEqual([w["id"] for w in optimized], [w["id"] for w in self.waypoints])
        self.assertGreaterEqual(evaluation["composite_cost"], 0.0)
        self.assertIn("fuel_raw", evaluation)
        self.assertIn("time_raw", evaluation)
        self.assertIn("risk_raw", evaluation)

    def test_sample_plan_passes_safety_validation(self):
        optimizer = route_optimizer.RouteOptimizer(risk_zones=self.zones)
        plan, evaluation = optimizer.optimize(self.waypoints)
        passed, violations = safety_validator.SafetyValidator().validate(plan, evaluation)
        self.assertTrue(passed, msg="; ".join(violations))

    def test_risk_scores_are_bounded(self):
        engine = risk_engine.RiskEngine(risk_zones=self.zones, obstacles=self.obstacles)
        report = engine.score_plan(self.waypoints)
        self.assertGreaterEqual(report["cumulative_risk"], 0.0)
        self.assertLessEqual(report["cumulative_risk"], 1.0)
        self.assertGreaterEqual(report["max_single_risk"], 0.0)
        self.assertLessEqual(report["max_single_risk"], 1.0)

    def test_fsm_rejects_invalid_transition(self):
        fsm = mission_fsm.MissionFSM()
        self.assertFalse(fsm.transition(mission_fsm.MissionState.COMPLETED, "skip planning"))
        self.assertEqual(fsm.state, mission_fsm.MissionState.IDLE)
        self.assertTrue(fsm.transition(mission_fsm.MissionState.PLANNING, "start"))
        self.assertTrue(fsm.transition(mission_fsm.MissionState.VALIDATING, "graph ready"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
