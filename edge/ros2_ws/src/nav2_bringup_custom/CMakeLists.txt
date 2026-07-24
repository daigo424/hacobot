cmake_minimum_required(VERSION 3.8)
project(nav2_bringup_custom)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/activate_slam_toolbox.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
