cmake_minimum_required(VERSION 3.8)
project(nav2_heartbeat_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(safety_metrics REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/nav2_heartbeat_adapter_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  lifecycle_msgs
  safety_metrics
)

add_executable(nav2_heartbeat_adapter_node src/main.cpp)
target_link_libraries(nav2_heartbeat_adapter_node ${PROJECT_NAME}_lib)
ament_target_dependencies(nav2_heartbeat_adapter_node rclcpp)

install(TARGETS
  nav2_heartbeat_adapter_node
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

ament_package()
