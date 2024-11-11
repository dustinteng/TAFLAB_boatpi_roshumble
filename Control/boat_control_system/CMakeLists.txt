cmake_minimum_required(VERSION 3.8)
project(boat_control_system)

# Compiler settings for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Include only if necessary

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/boat_control_system
)

# Create a library for shared code
add_library(coordinate_calculations_lib src/Coordinate_Calculations.cpp)
target_include_directories(coordinate_calculations_lib PUBLIC include)
ament_target_dependencies(coordinate_calculations_lib rclcpp std_msgs sensor_msgs geometry_msgs)

# Add executables and link the library
add_executable(coordinate_calculations_node src/coordinate_calculations_node.cpp)
target_link_libraries(coordinate_calculations_node coordinate_calculations_lib)
ament_target_dependencies(coordinate_calculations_node rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(waypoint_queue_node src/waypoint_queue_node.cpp src/Waypoint_Queue.cpp)
target_link_libraries(waypoint_queue_node coordinate_calculations_lib)
ament_target_dependencies(waypoint_queue_node rclcpp std_msgs sensor_msgs geometry_msgs)

# Install targets
install(TARGETS coordinate_calculations_node waypoint_queue_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
