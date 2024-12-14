cmake_minimum_required(VERSION 3.8)
project(taflab_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlData.msg"
  "msg/CalibrationData.msg"  # Add more .msg files here if necessary
  DEPENDENCIES std_msgs  # Add dependencies used in your messages
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
