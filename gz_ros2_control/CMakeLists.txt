cmake_minimum_required(VERSION 3.5)
project(gz_ros2_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  controller_manager
  generate_parameter_library
  gz_sim_vendor
  gz-sim
  gz_plugin_vendor
  gz-plugin
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(gz_ros2_control_parameters
  src/gz_ros2_control.yaml
)

#########

add_library(${PROJECT_NAME}-system SHARED
  src/gz_ros2_control_plugin.cpp
)

target_include_directories(${PROJECT_NAME}-system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME}-system
  gz-sim::gz-sim
  gz-plugin::register
  gz_ros2_control_parameters
)
ament_target_dependencies(${PROJECT_NAME}-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
)

#########

add_library(gz_hardware_plugins SHARED
  src/gz_system.cpp
  src/gz_system_pid.cpp
)
ament_target_dependencies(gz_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
)

target_include_directories(gz_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(gz_hardware_plugins
  gz-sim::gz-sim
  gz_ros2_control_parameters
)

## Install
install(TARGETS
  gz_hardware_plugins
  gz_ros2_control_parameters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  include/
  DESTINATION include
)

#########

add_library(ign_ros2_control-system SHARED
  src/gz_ros2_control_plugin.cpp
)

target_include_directories(ign_ros2_control-system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(ign_ros2_control-system
  gz-sim::gz-sim
  gz-plugin::register
  gz_ros2_control_parameters
)
ament_target_dependencies(ign_ros2_control-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add GTest dependency
  find_package(ament_cmake_gtest REQUIRED)

  # Add the PID helper test
  ament_add_gtest(
    test_pid_helper
    test/test_pid_helper.cpp
  )
  target_include_directories(test_pid_helper PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  ament_target_dependencies(test_pid_helper
    rclcpp_lifecycle
    hardware_interface
    rclcpp
  )
  target_link_libraries(test_pid_helper
    gz_hardware_plugins
    gz-sim::gz-sim
  )

  # Add the GZ system test
  ament_add_gtest(
    test_gz_system
    test/test_gz_system.cpp
  )
  target_include_directories(test_gz_system PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  ament_target_dependencies(test_gz_system
    rclcpp_lifecycle
    hardware_interface
    rclcpp
  )
  target_link_libraries(test_gz_system
    gz_hardware_plugins
    gz-sim::gz-sim
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}-system gz_hardware_plugins)
ament_export_libraries(ign_ros2_control-system gz_hardware_plugins)

# Install directories
install(TARGETS ${PROJECT_NAME}-system
  EXPORT gz_ros2_control
  DESTINATION lib
)
install(TARGETS ign_ros2_control-system
  EXPORT gz_ros2_control
  DESTINATION lib
)

pluginlib_export_plugin_description_file(gz_ros2_control gz_hardware_plugins.xml)

# Setup the project
ament_package()
