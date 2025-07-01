cmake_minimum_required(VERSION 3.8)
project(can_data_plugins)

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
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

# Export can data plugins
pluginlib_export_plugin_description_file(can_data_plugins can_data_plugins.xml)

# add can data plugin
add_library(
  can_data_plugins
  SHARED
  src/piper_motion_ctrl_2_data.cpp
)
ament_target_dependencies(
  can_data_plugins
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# install can data plugins
install(
  TARGETS 
  can_data_plugins
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

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

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
