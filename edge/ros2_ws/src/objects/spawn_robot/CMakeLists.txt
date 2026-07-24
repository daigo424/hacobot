cmake_minimum_required(VERSION 3.8)
project(spawn_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/entity_reaper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
