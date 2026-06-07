cmake_minimum_required(VERSION 3.8)
project(r2_bt_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(r2_bt_nodes REQUIRED)

# =========================
# Executable
# =========================
add_executable(r2_bt_executor_node
  src/bt_executor_node.cpp
)

ament_target_dependencies(r2_bt_executor_node
  rclcpp
  behaviortree_cpp
  r2_bt_nodes
)

target_compile_features(r2_bt_executor_node PUBLIC cxx_std_17)

# =========================
# Install executable
# =========================
install(TARGETS
  r2_bt_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Install XML config files
# =========================
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
