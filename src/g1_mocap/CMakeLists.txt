cmake_minimum_required(VERSION 3.8)
project(g1_mocap)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "msg/MocapController.msg"
  "msg/MocapControllers.msg"
  "msg/MocapFrame.msg"
  "msg/MocapStatus.msg"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY g1_mocap/
  DESTINATION ${PYTHON_INSTALL_DIR}/g1_mocap
  FILES_MATCHING PATTERN "*.py")
install(PROGRAMS scripts/mocap_node scripts/dashboard_node scripts/motion_capture_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config docs launch static DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md pyroki_offboard.md retarget_and_postprocess_guide.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(test_pythonpath "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:$ENV{PYTHONPATH}")
  ament_add_pytest_test(test_capture_stream test/test_capture_stream.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_consumer test/test_consumer.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_contact_refine test/test_contact_refine.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_motion_capture test/test_motion_capture.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_motion_capture_node test/test_motion_capture_node.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_motion_library test/test_motion_library.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_motion_model test/test_motion_model.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_retarget test/test_retarget.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_source_replay test/test_source_replay.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_stream test/test_stream.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_urdf test/test_urdf.py ENV "${test_pythonpath}")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()