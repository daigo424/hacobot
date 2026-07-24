cmake_minimum_required(VERSION 3.8)
project(create_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/world_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
