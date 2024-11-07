cmake_minimum_required(VERSION 3.8)
project(boat_control_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  include
)


# Add executables
add_executable(coordinate_calculations_node src/coordinate_calculations_node.cpp src/Coordinate_Calculations.cpp)
ament_target_dependencies(coordinate_calculations_node rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(waypoint_queue_node src/waypoint_queue_node.cpp src/Waypoint_Queue.cpp)
ament_target_dependencies(waypoint_queue_node rclcpp std_msgs sensor_msgs geometry_msgs)

# Install targets
install(TARGETS coordinate_calculations_node waypoint_queue_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()