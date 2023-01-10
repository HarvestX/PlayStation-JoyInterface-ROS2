cmake_minimum_required(VERSION 3.8)
project(p9n_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Display Node ======================================================
set(TARGET teleop_twist_joy_node)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${MY_LIB_NAME}
  PLUGIN "${PROJECT_NAME}::TeleopTwistJoyNode"
  EXECUTABLE ${TARGET}_exec)

# Testing ===========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
