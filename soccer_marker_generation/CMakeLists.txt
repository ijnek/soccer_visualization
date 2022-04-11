cmake_minimum_required(VERSION 3.8)
project(soccer_marker_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(soccer_object_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(soccer_vision_3d_msgs REQUIRED)

# Build ball_to_marker
add_executable(ball_to_marker src/ball_to_marker.cpp)
ament_target_dependencies(ball_to_marker
  rclcpp
  visualization_msgs
  soccer_object_msgs)

# Build goalpost_array_to_marker_array
add_executable(goalpost_array_to_marker_array src/goalpost_array_to_marker_array.cpp)
ament_target_dependencies(goalpost_array_to_marker_array
  rclcpp
  visualization_msgs
  soccer_object_msgs)

# Build field_line_array_to_marker_array
add_executable(field_line_array_to_marker_array src/field_line_array_to_marker_array.cpp)
ament_target_dependencies(field_line_array_to_marker_array
  rclcpp
  visualization_msgs
  soccer_object_msgs)

# Build robot_array_to_marker_array
add_executable(robot_array_to_marker_array src/robot_array_to_marker_array.cpp)
ament_target_dependencies(robot_array_to_marker_array
  rclcpp
  visualization_msgs
  soccer_object_msgs)

# Build flag_array_to_marker_array
add_executable(flag_array_to_marker_array src/flag_array_to_marker_array.cpp)
ament_target_dependencies(flag_array_to_marker_array
  rclcpp
  visualization_msgs
  soccer_object_msgs)

# Build ball_array_to_marker_array
add_library(ball_array_to_marker_array_node src/ball_array_to_marker_array.cpp)
ament_target_dependencies(ball_array_to_marker_array_node
  rclcpp
  visualization_msgs
  soccer_vision_3d_msgs
  rclcpp_components)
rclcpp_components_register_node(ball_array_to_marker_array_node
  PLUGIN "BallArray"
  EXECUTABLE ball_array_to_marker_array)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Install
install(TARGETS
  ball_to_marker
  goalpost_array_to_marker_array
  field_line_array_to_marker_array
  robot_array_to_marker_array
  flag_array_to_marker_array
  DESTINATION lib/${PROJECT_NAME})

foreach(dir launch meshes)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir}/)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
