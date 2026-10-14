cmake_minimum_required(VERSION 3.8)
project(s10_gazebo_sim)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Install Python scripts as executables
# ---------------------------------------------------------------------------
install(PROGRAMS
  src/mission_intelligence_node.py
  src/intent_parser.py
  src/waypoint_generator.py
  src/route_optimizer.py
  src/safety_validator.py
  src/risk_engine.py
  src/mission_fsm.py
  src/replanner.py
  src/drone_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Install launch, world, model, config, and rviz assets
# ---------------------------------------------------------------------------
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY worlds/   DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/   DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/     DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
