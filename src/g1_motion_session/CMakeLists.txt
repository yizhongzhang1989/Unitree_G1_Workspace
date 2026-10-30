cmake_minimum_required(VERSION 3.18)
project(g1_motion_session)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "msg/MotionEvent.msg"
  "srv/RegisterTask.srv"
  LIBRARY_NAME ${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pinocchio REQUIRED)
Python3_add_library(_arm_ik_native MODULE src/arm_ik_native.cpp)
target_link_libraries(_arm_ik_native PRIVATE pinocchio::pinocchio)
target_compile_features(_arm_ik_native PRIVATE cxx_std_17)
target_compile_options(_arm_ik_native PRIVATE -O3 -ffp-contract=off)
install(TARGETS _arm_ik_native LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/g1_motion_session)
install(DIRECTORY g1_motion_session/
  DESTINATION ${PYTHON_INSTALL_DIR}/g1_motion_session
  FILES_MATCHING PATTERN "*.py")
install(DIRECTORY config launch dashboard DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/motion_session scripts/dashboard_node
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(test_pythonpath "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:$ENV{PYTHONPATH}")
  ament_add_pytest_test(test_registry test/test_registry.py
    ENV "${test_pythonpath}")
  ament_add_pytest_test(test_arm_ik test/test_arm_ik.py
    ENV "${test_pythonpath}")
  ament_add_pytest_test(test_dashboard test/test_dashboard.py
    ENV "${test_pythonpath}")
  ament_add_pytest_test(test_ros_lifecycle test/test_ros_lifecycle.py
    ENV "${test_pythonpath}" "ROS_DOMAIN_ID=216")
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
