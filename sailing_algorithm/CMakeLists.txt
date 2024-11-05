cmake_minimum_required(VERSION 3.8)
project(sailing_algorithm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(boat_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sailing_controller src/sailing_controller.cpp)
target_include_directories(sailing_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sailing_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sailing_controller
  "rclcpp"
  "boat_msgs"
  "sensor_msgs"
)

install(TARGETS sailing_controller
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
