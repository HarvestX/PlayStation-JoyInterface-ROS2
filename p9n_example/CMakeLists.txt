cmake_minimum_required(VERSION 3.8)
project(p9n_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Display Node ======================================================
set(TARGET display_node)
ament_auto_add_library(
  ${TARGET}
    SHARED
      src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "p9n_example::DisplayNode"
    EXECUTABLE ${TARGET}_exec)
# End Display Node ==================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
