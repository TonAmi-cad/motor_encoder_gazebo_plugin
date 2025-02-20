cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(link_msgs REQUIRED)

# Create turret control plugin
add_library(turret_control_plugin SHARED
  src/turret_control_plugin.cpp
  src/turret_parameters.cpp
  src/action_server.cpp
  src/trajectory_calculator.cpp
  src/angle_utils.cpp
)

target_include_directories(turret_control_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against dependencies
ament_target_dependencies(turret_control_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "geometry_msgs"
  "link_msgs"
)

# Install plugin library
install(TARGETS turret_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description
install(FILES
  gazebo_ros_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
