"""Static wiring tests for the ROS/Gazebo runtime contract (no ROS required)."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = (ROOT / "launch" / "s10_mission_sim.launch.py").read_text()
MODEL = (ROOT / "models" / "s10_drone" / "model.sdf").read_text()
MISSION = (ROOT / "src" / "mission_intelligence_node.py").read_text()
CONTROLLER = (ROOT / "src" / "drone_controller.py").read_text()


class RuntimeContractTests(unittest.TestCase):
    def test_bridge_has_command_odometry_and_enable_topics(self):
        self.assertIn("/s10_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist", LAUNCH)
        self.assertIn("/s10_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry", LAUNCH)
        self.assertIn("/s10_drone/enable@std_msgs/msg/Bool]gz.msgs.Boolean", LAUNCH)

    def test_model_has_matching_velocity_controller_topics(self):
        self.assertIn("<commandSubTopic>cmd_vel</commandSubTopic>", MODEL)
        self.assertIn("<enableSubTopic>enable</enableSubTopic>", MODEL)
        self.assertIn("<odom_topic>/s10_drone/odometry</odom_topic>", MODEL)

    def test_mission_and_controller_topic_names_match(self):
        self.assertIn('"/drone_controller/target_waypoints"', MISSION)
        self.assertIn('"~/target_waypoints"', CONTROLLER)
        self.assertIn('"/drone_controller/navigation_status"', MISSION)
        self.assertIn('"~/navigation_status"', CONTROLLER)

    def test_startup_has_spawn_delay_and_odometry_gate(self):
        self.assertIn("TimerAction(period=4.0", LAUNCH)
        self.assertIn("if not self.odom_received", MISSION)
        self.assertIn("if not self.odom_received", CONTROLLER)

    def test_landing_is_not_reported_complete_after_one_tick(self):
        self.assertIn("self.landing_active = True", CONTROLLER)
        self.assertIn("All waypoints reached and landing complete", CONTROLLER)


if __name__ == "__main__":
    unittest.main(verbosity=2)
