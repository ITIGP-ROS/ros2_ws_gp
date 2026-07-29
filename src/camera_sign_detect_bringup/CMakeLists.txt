cmake_minimum_required(VERSION 3.8)
project(camera_sign_detect_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vision_msgs REQUIRED)


add_library(can_comms SHARED src/can_comms.cpp)
target_include_directories(can_comms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_components SHARED src/yolo_class_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  vision_msgs
)
target_link_libraries(${PROJECT_NAME}_components can_comms)
target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "YoloClassCanNode"
  EXECUTABLE yolo_class_can_node
)

install(TARGETS can_comms ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
