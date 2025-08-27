cmake_minimum_required(VERSION 3.8)
project(x500_offboard_control)

# Default to C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add library
add_library(${PROJECT_NAME}_lib
  src/x500_offboard_control.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Add executable
add_executable(${PROJECT_NAME}_node
  src/x500_offboard_control_node.cpp
)

# Link library to executable
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Link dependencies to executable
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

# Install targets
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files (if any)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Export dependencies
ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()