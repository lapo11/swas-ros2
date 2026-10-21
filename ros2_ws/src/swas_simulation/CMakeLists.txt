cmake_minimum_required(VERSION 3.8)
project(swas_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/bxu_battery_lift_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
