cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(boat_control_system_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(taflab_msgs REQUIRED)

# Include directories
include_directories(
  include/boat_control_system_final
  ${taflab_msgs_INCLUDE_DIRS}  # Ensure taflab_msgs headers are included
)

# Add libraries for source files
add_library(WaypointQueue src/WaypointQueue.cpp)
ament_target_dependencies(WaypointQueue rclcpp sensor_msgs)

add_library(CoordinateCalculations src/Coordinate_Calculations.cpp)
ament_target_dependencies(CoordinateCalculations rclcpp sensor_msgs std_msgs)

add_library(RudderControlHelper src/RudderControlHelper.cpp)
ament_target_dependencies(RudderControlHelper rclcpp sensor_msgs std_msgs)

# Create a library for CoordinateCalculationsNode without a main function
add_library(CoordinateCalculationsNodeLib src/CoordinateCalculationsNode.cpp)
ament_target_dependencies(CoordinateCalculationsNodeLib rclcpp sensor_msgs std_msgs taflab_msgs)
target_link_libraries(CoordinateCalculationsNodeLib CoordinateCalculations WaypointQueue)

# Create a library for RudderServoControlNode without a main function
add_library(RudderServoControlNodeLib src/RudderServoControlNode.cpp)
ament_target_dependencies(RudderServoControlNodeLib rclcpp sensor_msgs std_msgs taflab_msgs)
target_include_directories(RudderServoControlNodeLib PUBLIC ${taflab_msgs_INCLUDE_DIRS})
target_link_libraries(RudderServoControlNodeLib RudderControlHelper WaypointQueue)

# Add the executable for CoordinateCalculationsNode with the main function
add_executable(coordinate_calculations_node src/CoordinateCalculationsNode.cpp)
target_link_libraries(coordinate_calculations_node CoordinateCalculationsNodeLib)

# Add the executable for RudderServoControlNode with the main function
add_executable(rudder_servo_control_node src/RudderServoControlNode.cpp)
target_link_libraries(rudder_servo_control_node RudderServoControlNodeLib)

# Add a library for SailServoControlNode
add_library(SailServoControlNodeLib src/SailServoControlNode.cpp)
ament_target_dependencies(SailServoControlNodeLib rclcpp sensor_msgs std_msgs taflab_msgs)

# Add the executable for sail servo control
add_executable(sail_servo_control_node src/SailServoControlNode.cpp)
target_link_libraries(sail_servo_control_node SailServoControlNodeLib)

# Add the executable for WaypointQueueNode
add_executable(waypoint_queue_node src/WaypointQueueNode.cpp)
ament_target_dependencies(waypoint_queue_node rclcpp sensor_msgs std_msgs)
target_link_libraries(waypoint_queue_node WaypointQueue)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the targets
install(TARGETS
  waypoint_queue_node
  coordinate_calculations_node
  rudder_servo_control_node
  sail_servo_control_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
