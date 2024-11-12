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

# Add the libraries for the source files
add_library(WaypointQueue src/WaypointQueue.cpp)
ament_target_dependencies(WaypointQueue rclcpp sensor_msgs)

add_library(CoordinateCalculations src/Coordinate_Calculations.cpp)
ament_target_dependencies(CoordinateCalculations rclcpp sensor_msgs std_msgs)

# Ensure that CoordinateCalculationsNode links to both libraries
add_library(CoordinateCalculationsNodeLib src/CoordinateCalculationsNode.cpp)
ament_target_dependencies(CoordinateCalculationsNodeLib rclcpp sensor_msgs std_msgs)
target_link_libraries(CoordinateCalculationsNodeLib CoordinateCalculations WaypointQueue)

# Add the executable for the CoordinateCalculationsNode
add_executable(CoordinateCalculationsNodeExec src/CoordinateCalculationsNode.cpp)
target_link_libraries(CoordinateCalculationsNodeExec CoordinateCalculationsNodeLib CoordinateCalculations WaypointQueue)

# Add the executable for the WaypointQueueNode
add_executable(WaypointQueueNode src/WaypointQueueNode.cpp)
ament_target_dependencies(WaypointQueueNode rclcpp sensor_msgs std_msgs)
target_link_libraries(WaypointQueueNode WaypointQueue)

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

# Install the targets
install(TARGETS
  WaypointQueueNode
  CoordinateCalculationsNodeExec
  CoordinateCalculationsNodeLib
  CoordinateCalculations
  WaypointQueue
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
