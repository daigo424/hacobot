cmake_minimum_required(VERSION 3.8)
project(stall_recovery)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(safety_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(safety_metrics REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/stall_recovery_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  safety_msgs
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  slam_toolbox
  safety_metrics
)

add_executable(stall_recovery_node src/main.cpp)
target_link_libraries(stall_recovery_node ${PROJECT_NAME}_lib)
ament_target_dependencies(stall_recovery_node rclcpp)

install(TARGETS
  stall_recovery_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lib
  DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stall_recovery test/test_stall_recovery.cpp)
  target_link_libraries(test_stall_recovery ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_stall_recovery
    rclcpp
    rclcpp_lifecycle
    safety_msgs
    lifecycle_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
  )
endif()

ament_package()
