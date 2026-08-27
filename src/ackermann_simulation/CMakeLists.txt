cmake_minimum_required(VERSION 3.8)
project(ackermann_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# There is nothing to compile here. Everything this package ships is a launch file, a
# world, a map or an RViz layout; the nodes it starts all belong to ackermann_bringup or
# to upstream packages, which is what keeps the simulated stack and the vehicle's stack
# the same stack.
install(DIRECTORY
  launch
  config
  worlds
  maps
  tools
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
