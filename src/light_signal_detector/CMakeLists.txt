cmake_minimum_required(VERSION 3.8)
project(light_signal_detector)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/light_signal_detector_node.py
  scripts/light_signal_wait_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/light_signal_detector_core.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_light_signal_detector_core
    test/test_light_signal_detector_core.py
  )
  ament_add_pytest_test(
    test_light_signal_wait_action
    test/test_light_signal_wait_action.py
    TIMEOUT 30
  )
endif()

ament_package()
