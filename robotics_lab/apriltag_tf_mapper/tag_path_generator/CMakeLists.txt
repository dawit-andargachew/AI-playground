cmake_minimum_required(VERSION 3.8)
project(tag_path_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)   # <-- ADD THIS
find_package(geometry_msgs REQUIRED)

add_executable(path_saver src/path_saver.cpp)
ament_target_dependencies(path_saver
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs   # <-- AND ADD THIS
  geometry_msgs
)

install(TARGETS path_saver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()