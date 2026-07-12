cmake_minimum_required(VERSION 3.8)
project(watchdog)

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
find_package(std_msgs REQUIRED)
find_package(safety_metrics REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/watchdog_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  safety_msgs
  lifecycle_msgs
  std_msgs
  safety_metrics
)

add_executable(watchdog_node src/main.cpp)
target_link_libraries(watchdog_node ${PROJECT_NAME}_lib)
ament_target_dependencies(watchdog_node rclcpp)

install(TARGETS
  watchdog_node
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
  find_package(sensor_msgs REQUIRED)
  ament_add_gtest(test_watchdog test/test_watchdog.cpp)
  target_link_libraries(test_watchdog ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_watchdog
    rclcpp
    rclcpp_lifecycle
    safety_msgs
    lifecycle_msgs
    sensor_msgs
    std_msgs
  )
endif()

ament_package()
