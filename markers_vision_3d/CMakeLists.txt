cmake_minimum_required(VERSION 3.8)
project(markers_vision_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(soccer_vision_3d_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp_components
  soccer_vision_3d_msgs
  visualization_msgs)

# Build ball_node
add_library(ball_node SHARED
  src/ball.cpp)
target_include_directories(ball_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(ball_node
  PLUGIN "markers_vision_3d::Ball"
  EXECUTABLE ball)

ament_target_dependencies(ball_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Build marking_array_node
add_library(marking_array_node SHARED
  src/marking_array.cpp)
target_include_directories(marking_array_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(marking_array_node
  PLUGIN "markers_vision_3d::MarkingArray"
  EXECUTABLE marking_array)

ament_target_dependencies(marking_array_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ball_node marking_array_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
