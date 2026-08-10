cmake_minimum_required(VERSION 3.5)
project(gloria_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MitCommand.msg"
  "msg/PvCommand.msg"
)

# rosidl_generate_interfaces() 与 ament_python_install_package() 用同一项目名时
# 会创建重名 CMake target。消息生成器已负责 Python 环境 hook，
# 这里直接安装同名包内的节点源码，与生成的 gloria_ros/msg 目录合并。
install(DIRECTORY
  gloria_ros/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/gripper_node
  scripts/web_gripper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_can_feedback test/test_can_feedback.py)
  ament_add_pytest_test(test_gripper_state test/test_gripper_state.py)
  ament_add_pytest_test(test_web_gripper test/test_web_gripper.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
