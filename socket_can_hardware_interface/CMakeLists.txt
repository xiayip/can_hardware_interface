cmake_minimum_required(VERSION 3.8)
project(socket_can_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  can_data_plugins
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

# add hardware driver library
add_library(
  ${PROJECT_NAME} 
  SHARED 
  src/socket_can_hardware_interface.cpp)
target_link_libraries(
  ${PROJECT_NAME}
)
ament_target_dependencies(
  ${PROJECT_NAME} 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware 
pluginlib_export_plugin_description_file(hardware_interface socket_can_hardware_interface.xml)

# install hardware
install(
  TARGETS 
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY description 
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(
  include
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

## EXPORTS
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
