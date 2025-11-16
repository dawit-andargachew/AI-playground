cmake_minimum_required(VERSION 3.8)
project(six_ex4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BurrowStatus.srv"
  DEPENDENCIES geometry_msgs
)

# Turtlebot node
add_executable(turtlebot_node src/turtlebot_node.cpp)
ament_target_dependencies(turtlebot_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)

# Burrow node
add_executable(burrow_node src/burrow_node.cpp)
ament_target_dependencies(burrow_node 
  rclcpp 
  geometry_msgs
)

# Link service to nodes
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(turtlebot_node "${cpp_typesupport_target}")
target_link_libraries(burrow_node "${cpp_typesupport_target}")

install(TARGETS
  turtlebot_node
  burrow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()