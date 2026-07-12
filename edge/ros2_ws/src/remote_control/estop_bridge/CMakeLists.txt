cmake_minimum_required(VERSION 3.8)
project(estop_bridge)

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

# librdkafkaはROSパッケージではなくシステムライブラリ(Dockerfileでapt installしたlibrdkafka-dev)
# のため、ament_cmakeのfind_package()ではなくpkg-configで探す。
find_package(PkgConfig REQUIRED)
pkg_check_modules(RDKAFKA REQUIRED IMPORTED_TARGET rdkafka++)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/estop_bridge_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  safety_msgs
  lifecycle_msgs
  std_msgs
  safety_metrics
)
target_link_libraries(${PROJECT_NAME}_lib PkgConfig::RDKAFKA)

add_executable(estop_bridge_node src/main.cpp)
target_link_libraries(estop_bridge_node ${PROJECT_NAME}_lib)
ament_target_dependencies(estop_bridge_node rclcpp)

install(TARGETS
  estop_bridge_node
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
  ament_add_gtest(test_estop_bridge test/test_estop_bridge.cpp)
  target_link_libraries(test_estop_bridge ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_estop_bridge
    rclcpp
    rclcpp_lifecycle
    safety_msgs
    lifecycle_msgs
    std_msgs
  )
endif()

ament_package()
