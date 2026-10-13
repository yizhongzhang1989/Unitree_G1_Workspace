cmake_minimum_required(VERSION 3.8)
project(lower_footstep_tracking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/FootstepPlan.msg")

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY lower_footstep_tracking/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py")
install(DIRECTORY lower_footstep_tracking/static
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/footstep_task scripts/footstep_web scripts/export_policy_gains DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_footstep_task test/test_footstep_task.py)
  ament_add_pytest_test(test_footstep_policy test/test_footstep_policy.py)
  ament_add_pytest_test(test_footstep_planner test/test_footstep_planner.py)
  ament_add_pytest_test(test_footstep_web test/test_footstep_web.py)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()