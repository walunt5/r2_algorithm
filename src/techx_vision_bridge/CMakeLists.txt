cmake_minimum_required(VERSION 3.8)
project(techx_vision_bridge)

# ── 默认 C++17，ROS 2 Humble 推荐 ──
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖查找 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── 自定义消息生成 ──
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target3D.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ── 可执行文件 ──
add_executable(vision_bridge_node
  src/vision_bridge_node.cpp
)

target_include_directories(vision_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接 ROS 2 库 + 自定义消息库
ament_target_dependencies(vision_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
)

rosidl_target_interfaces(vision_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 链接 pthread（Linux 需要，aarch64 兼容）
find_package(Threads REQUIRED)
target_link_libraries(vision_bridge_node Threads::Threads)

# ── 安装 ──
install(TARGETS
  vision_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ── 测试 ──
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()