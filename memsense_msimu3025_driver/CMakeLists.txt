cmake_minimum_required(VERSION 3.8)
project(memsense_msimu3025_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(msimu3025_driver src/msimu3025_driver.cpp)
target_compile_features(msimu3025_driver PUBLIC cxx_std_17)
ament_target_dependencies(msimu3025_driver geometry_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS msimu3025_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
