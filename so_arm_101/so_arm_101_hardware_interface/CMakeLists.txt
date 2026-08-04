cmake_minimum_required(VERSION 3.16)
project(so_arm_101_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(so101_system SHARED
  src/so101_system.cpp
)
target_compile_features(so101_system PUBLIC cxx_std_17)
target_include_directories(so101_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(so101_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface so_arm_101_hardware_interface_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS so101_system
  EXPORT export_so101_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES so_arm_101_hardware_interface_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_so101_system HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs std_msgs)
ament_package()
