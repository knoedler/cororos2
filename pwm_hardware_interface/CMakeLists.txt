cmake_minimum_required(VERSION 3.8)
project(pwm_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(pwm_hardware_interface SHARED
  src/pwm_hardware_interface.cpp)
target_compile_features(pwm_hardware_interface PUBLIC cxx_std_17)
target_include_directories(pwm_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pwm_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface pwm_hardware_interface.xml)

install(TARGETS
  pwm_hardware_interface
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(FILES pwm_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(pwm_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
