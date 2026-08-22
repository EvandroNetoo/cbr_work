cmake_minimum_required(VERSION 3.16)
project(base_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(mariola_system SHARED src/mariola_system.cpp)
target_compile_features(mariola_system PUBLIC cxx_std_17)
target_include_directories(mariola_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mariola_system
  interfaces hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs)

pluginlib_export_plugin_description_file(
  hardware_interface base_hardware_interface_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mariola_system EXPORT export_mariola_system
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES base_hardware_interface_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mariola_system test/test_mariola_system.cpp)
  target_link_libraries(test_mariola_system mariola_system)
  ament_target_dependencies(test_mariola_system
    interfaces hardware_interface rclcpp rclcpp_lifecycle sensor_msgs)
endif()

ament_export_targets(export_mariola_system HAS_LIBRARY_TARGET)
ament_export_dependencies(
  interfaces hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs)
ament_package()
