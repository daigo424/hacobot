cmake_minimum_required(VERSION 3.8)
project(status_overlay)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/status_overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
