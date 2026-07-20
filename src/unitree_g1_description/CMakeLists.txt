cmake_minimum_required(VERSION 3.8)
project(unitree_g1_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS unitree_g1_description/lowstate_to_joint_states_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lowstate_to_joint_states
)

install(
  PROGRAMS unitree_g1_description/mit_position_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mit_position_controller
)

install(
  DIRECTORY config model
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_joint_state_mapping
    test/test_joint_state_mapping.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_mit_command
    test/test_mit_command.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_mit_position_controller
    test/test_mit_position_controller.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_motion_switcher
    test/test_motion_switcher.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()