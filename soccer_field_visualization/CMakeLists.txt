cmake_minimum_required(VERSION 3.8)
project(soccer_field_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(soccer_field_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Build soccer_field_visualization_node
add_executable(soccer_field_visualization_node
  src/soccer_field_visualization_node.cpp
  src/marker_generation.cpp)

ament_target_dependencies(soccer_field_visualization_node
  rclcpp
  soccer_field_msgs
  visualization_msgs)

# Install
install(TARGETS
  soccer_field_visualization_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
