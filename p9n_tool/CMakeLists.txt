cmake_minimum_required(VERSION 3.8)
project(p9n_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
# find_package(hidapi REQUIRED)
ament_auto_find_build_dependencies()

# p9n tool =====================================================
ament_auto_add_library(
  ${PROJECT_NAME}
  STATIC
    ./src/${PROJECT_NAME}.cpp
    ./src/p9n_utilities.cpp
    ./src/crc32.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
# End p9n tool =================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()