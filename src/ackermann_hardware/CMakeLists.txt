cmake_minimum_required(VERSION 3.16)
project(ackermann_hardware LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
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


## COMPILE
## COMPILE
add_library(
  ackermann_hardware
  SHARED
  src/ackermann_system.cpp
  src/can_comms.cpp
  src/v_pace_db.c
)

target_compile_features(ackermann_hardware PUBLIC cxx_std_17)
target_include_directories(ackermann_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include/ackermann_hardware>
)
ament_target_dependencies(
  ackermann_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# target_link_libraries(ackermann_hardware PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ACKERMANN_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ackermann_hardware.xml)

# INSTALL
install(
  DIRECTORY src/include/
  DESTINATION include/ackermann_hardware
)

install(TARGETS ackermann_hardware
  EXPORT export_ackermann_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ackermann_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()