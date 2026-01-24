cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  terrence_hwc
  SHARED
  src/terrence_hwc.cpp
)

target_compile_features(terrence_hwc PUBLIC cxx_std_17)

# IMPORTANT: include/ is now directly under this package root
target_include_directories(terrence_hwc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  terrence_hwc PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface config/terrence_hwc.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the shared library
install(TARGETS terrence_hwc
  EXPORT export_terrence_hwc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin XML so hardware_interface can find it
install(
  # FILES config/terrence_hwc.xml
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_terrence_hwc HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()