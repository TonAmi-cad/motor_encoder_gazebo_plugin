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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TurretControl.action"
  DEPENDENCIES builtin_interfaces
)

# Get the include directories for generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

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

# Link against the generated interfaces
target_link_libraries(turret_control_plugin "${cpp_typesupport_target}")

ament_target_dependencies(turret_control_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "geometry_msgs"
)

# Install plugin library
install(TARGETS turret_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
