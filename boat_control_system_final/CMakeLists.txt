cmake_minimum_required(VERSION 3.8)
project(boat_control_system_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Adding include files
include_directories(include/boat_control_system_final)

# Add the executable and specify the source file
add_executable(WaypointQueueNode src/WaypointQueueNode.cpp)  # Replace with your actual source file

# Link the executable with its dependencies
ament_target_dependencies(WaypointQueueNode rclcpp)
ament_target_dependencies(WaypointQueueNode rclcpp std_msgs)
ament_target_dependencies(WaypointQueueNode rclcpp sensor_msgs)


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

# Install the executable
install(TARGETS
  WaypointQueueNode
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
