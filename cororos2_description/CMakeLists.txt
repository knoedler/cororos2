cmake_minimum_required(VERSION 3.8)
project(cororos2_description)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_allie_urdf_xacro test/allie_test_urdf_xacro.py)
  ament_add_pytest_test(test_cornelius_urdf_xacro test/cornelius_test_urdf_xacro.py)
  ament_add_pytest_test(test_joe_urdf_xacro test/joe_test_urdf_xacro.py)
endif()

ament_package()
