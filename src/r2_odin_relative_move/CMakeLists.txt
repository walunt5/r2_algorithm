cmake_minimum_required(VERSION 3.8)
project(r2_odin_relative_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(r2_odin_interfaces REQUIRED)

add_executable(odin_relative_move_server
  src/relative_move_server.cpp
)

ament_target_dependencies(odin_relative_move_server
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  r2_odin_interfaces
)

target_compile_features(odin_relative_move_server PUBLIC cxx_std_17)

install(
  TARGETS odin_relative_move_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
