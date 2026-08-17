cmake_minimum_required(VERSION 3.8)
project(ackermann_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


add_executable(emergency_stop_server src/emergency_stop_server.cpp)
target_include_directories(emergency_stop_server PRIVATE src)
# NOTE: rosidl interface packages (std_msgs, std_srvs, action_msgs) do NOT export a
# plain pkg::pkg CMake target — only pkg::pkg__rosidl_typesupport_cpp and friends.
# Naming them here failed the build on Humble. ament_target_dependencies() below
# already links them correctly, so only the genuine rclcpp target is listed.
target_link_libraries(emergency_stop_server
  rclcpp::rclcpp
)
ament_target_dependencies(emergency_stop_server
  rclcpp
  std_msgs
  std_srvs
  action_msgs
)
install(TARGETS emergency_stop_server
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(road_classification_node src/road_classification_node.cpp)
target_include_directories(road_classification_node PRIVATE src)
ament_target_dependencies(road_classification_node
  rclcpp
  nav2_msgs
)
install(TARGETS road_classification_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)


# NOTHING HERE MAY PUBLISH TO /cmd_vel_stamped.
#
# odom_test.py and circle_test.py used to be installed here and were REMOVED as a
# safety defect. Both published directly to /cmd_vel_stamped, which robot.launch.py
# gives to the controller as /ackermann_controller/reference — i.e. DOWNSTREAM of
# twist_mux, and therefore downstream of the emergency_stop lock (priority 255 in
# twist_mux_locks.yaml). Running either one drove the vehicle with the emergency stop
# bypassed. They were verified installed and runnable on the unit at
# /opt/ros/humble/lib/ackermann_bringup/.
#
# Anything that commands motion must go in through twist_mux — /cmd_vel_teleop,
# /cmd_vel_nav_smoothed or /cmd_vel_recovery — never at the controller's input.
# twist_stamper.py is the one legitimate publisher of /cmd_vel_stamped: it is the
# bridge that stamps twist_mux's own /cmd_vel output, so it sits INSIDE the chain.
install(PROGRAMS
  src/twist_stamper.py
  src/camera_compressor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(CODE "
  execute_process(COMMAND chmod +x
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/twist_stamper.py
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/camera_compressor.py
  )
")

ament_package()