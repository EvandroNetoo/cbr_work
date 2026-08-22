cmake_minimum_required(VERSION 3.16)
project(robot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_composed_urdf test/test_composed_urdf.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()
ament_package()
