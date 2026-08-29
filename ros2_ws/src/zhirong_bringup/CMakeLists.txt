cmake_minimum_required(VERSION 3.8)
project(zhirong_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/zhirong_test_map.pgm
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/zhirong_test_map.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/zhirong_test_map_preview.png
  DESTINATION share/${PROJECT_NAME}/maps
)

install(
  PROGRAMS
    scripts/gamepad_udp_teleop.py
    scripts/goal_guard.py
    scripts/keyboard_hold_teleop.py
    scripts/keyboard_teleop.py
    scripts/patrol_angular_guard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/goal_safety.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_goal_safety
    test/test_goal_safety.py
  )
endif()

ament_package()
