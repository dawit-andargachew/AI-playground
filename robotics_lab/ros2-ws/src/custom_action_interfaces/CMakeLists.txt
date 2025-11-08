cmake_minimum_required(VERSION 3.8)
project(custom_action_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


find_package(rosidl_default_generators REQUIRED)

# Ensure the generated interface runtime is exported to dependents
find_package(rosidl_default_runtime REQUIRED)

# Debug: print package.xml contents to help diagnose rosidl_generate_interfaces checks
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" _pkg_xml_contents)
string(REPLACE "\n" "\\n" _pkg_xml_escaped "${_pkg_xml_contents}")
message(STATUS "[DEBUG] package.xml contents: ${_pkg_xml_escaped}")

rosidl_generate_interfaces(${PROJECT_NAME}
  SKIP_GROUP_MEMBERSHIP_CHECK
  "action/Fibonacci.action"
)

## export the rosidl runtime so packages depending on this interface can find the generated types
ament_export_dependencies(rosidl_default_runtime)

ament_package()
