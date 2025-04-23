cmake_minimum_required(VERSION 3.8)
project(boat_control_system_final)

# C++17 + warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(taflab_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# include headers
include_directories(
  include/${PROJECT_NAME}
  ${taflab_msgs_INCLUDE_DIRS}
)

# ——— Libraries ———
add_library(WaypointQueue        src/WaypointQueue.cpp)
ament_target_dependencies(WaypointQueue        rclcpp sensor_msgs)

add_library(CoordinateCalculations src/Coordinate_Calculations.cpp)
ament_target_dependencies(CoordinateCalculations rclcpp sensor_msgs std_msgs)

add_library(RudderControlHelper   src/RudderControlHelper.cpp)
ament_target_dependencies(RudderControlHelper   rclcpp sensor_msgs std_msgs)

# Node‐lib variants (no main)
add_library(CoordinateCalculationsNodeLib src/CoordinateCalculationsNode.cpp)
ament_target_dependencies(
  CoordinateCalculationsNodeLib
  rclcpp sensor_msgs std_msgs taflab_msgs
)
target_link_libraries(
  CoordinateCalculationsNodeLib
  CoordinateCalculations
  WaypointQueue
)

add_library(RudderServoControlNodeLib src/RudderServoControlNode.cpp)
ament_target_dependencies(
  RudderServoControlNodeLib
  rclcpp sensor_msgs std_msgs taflab_msgs
)
target_include_directories(
  RudderServoControlNodeLib PUBLIC ${taflab_msgs_INCLUDE_DIRS}
)
target_link_libraries(
  RudderServoControlNodeLib
  RudderControlHelper
  WaypointQueue
)

add_library(SailServoControlNodeLib src/SailServoControlNode.cpp)
ament_target_dependencies(
  SailServoControlNodeLib
  rclcpp sensor_msgs std_msgs taflab_msgs ament_index_cpp
)
target_link_libraries(
  SailServoControlNodeLib
  ament_index_cpp::ament_index_cpp
)

# ——— Executables ———
add_executable(coordinate_calculations_node src/CoordinateCalculationsNode.cpp)
ament_target_dependencies(
  coordinate_calculations_node
  rclcpp sensor_msgs std_msgs taflab_msgs
)
target_link_libraries(coordinate_calculations_node CoordinateCalculationsNodeLib)

add_executable(rudder_servo_control_node src/RudderServoControlNode.cpp)
ament_target_dependencies(
  rudder_servo_control_node
  rclcpp sensor_msgs std_msgs taflab_msgs
)
target_link_libraries(rudder_servo_control_node RudderServoControlNodeLib)

add_executable(sail_servo_control_node src/SailServoControlNode.cpp)
ament_target_dependencies(
  sail_servo_control_node
  rclcpp sensor_msgs std_msgs taflab_msgs
)
target_link_libraries(sail_servo_control_node SailServoControlNodeLib)

add_executable(waypoint_queue_node src/WaypointQueueNode.cpp)
ament_target_dependencies(waypoint_queue_node rclcpp sensor_msgs std_msgs)
target_link_libraries(waypoint_queue_node WaypointQueue)

# ——— Linting (optional) ———
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ——— Installation ———

## C++ nodes into lib/<pkg>/
install(
  TARGETS
    coordinate_calculations_node
    rudder_servo_control_node
    sail_servo_control_node
    waypoint_queue_node
  RUNTIME  DESTINATION lib/${PROJECT_NAME}
  LIBRARY  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE  DESTINATION lib/${PROJECT_NAME}
)

## Headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

## Data files (CSV)
install(
  FILES SailAngleData.csv
  DESTINATION share/${PROJECT_NAME}
)

# ament exports
ament_package()