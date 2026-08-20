cmake_minimum_required(VERSION 3.8)
project(unitree_g1_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# colcon leaves CMAKE_BUILD_TYPE empty, which means no -O at all. Measured on
# the Jetson: one gravity feed-forward evaluation costs 21.7 us unoptimised and
# 0.89 us at -O2, i.e. 1.1% versus 0.05% of a 500 Hz cycle - and the same factor
# applies to the hardware plugin on that loop. RelWithDebInfo keeps the frame
# pointers and symbols needed to profile it. An explicit -DCMAKE_BUILD_TYPE on
# the colcon command line still wins.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gloria_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(imu_sensor_broadcaster REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/adaptive_stiffness.cpp
  src/forward_position_controller.cpp
  src/ft_compensation.cpp
  src/gravity_feedforward.cpp
  src/gravity_table.cpp
  src/gravity_joint_trajectory_controller.cpp
  src/g1_topic_system.cpp
  src/throttled_broadcasters.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  controller_interface
  geometry_msgs
  gloria_ros
  hardware_interface
  imu_sensor_broadcaster
  joint_state_broadcaster
  joint_trajectory_controller
  pluginlib
  rclcpp
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  unitree_api
  unitree_hg
  yaml_cpp_vendor
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# 把 KWR57 原始读数变成负载净力旋量。独立进程：通用 CAN bridge 不该知道手臂在哪。
add_executable(ft_wrench_compensator src/ft_wrench_compensator_node.cpp)
target_link_libraries(ft_wrench_compensator ${PROJECT_NAME})
ament_target_dependencies(ft_wrench_compensator
  geometry_msgs
  rclcpp
  sensor_msgs
  std_srvs
  unitree_hg
)

pluginlib_export_plugin_description_file(
  controller_interface controller_plugins.xml)
pluginlib_export_plugin_description_file(
  hardware_interface hardware_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS ft_wrench_compensator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  ament_add_gtest(
    test_periodic_deadline
    test/test_periodic_deadline.cpp
  )
  target_include_directories(test_periodic_deadline PRIVATE include)
  ament_add_gtest(
    test_g1_topic_system
    test/test_g1_topic_system.cpp
  )
  target_link_libraries(
    test_g1_topic_system
    ${PROJECT_NAME}
    yaml-cpp
  )
  ament_target_dependencies(
    test_g1_topic_system
    hardware_interface
    rclcpp
  )
  target_compile_definitions(test_g1_topic_system PRIVATE
    CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_add_gtest(
    test_adaptive_stiffness
    test/test_adaptive_stiffness.cpp
  )
  target_link_libraries(
    test_adaptive_stiffness
    ${PROJECT_NAME}
  )
  ament_target_dependencies(
    test_adaptive_stiffness
    controller_interface
    hardware_interface
    rclcpp
  )
  ament_add_gtest(
    test_forward_position_controller
    test/test_forward_position_controller.cpp
  )
  target_link_libraries(
    test_forward_position_controller
    ${PROJECT_NAME}
  )
  ament_target_dependencies(
    test_forward_position_controller
    hardware_interface
    rclcpp
    std_msgs
  )
  ament_add_gtest(
    test_gravity_feedforward
    test/test_gravity_feedforward.cpp
  )
  target_link_libraries(
    test_gravity_feedforward
    ${PROJECT_NAME}
  )
  ament_add_gtest(
    test_ft_compensation
    test/test_ft_compensation.cpp
  )
  target_link_libraries(
    test_ft_compensation
    ${PROJECT_NAME}
  )
  # 跨语言对拍数据由 arm_gravity_compensation 的 Python 参考实现生成。
  target_compile_definitions(test_ft_compensation PRIVATE
    TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gtest(
    test_gravity_joint_trajectory_controller
    test/test_gravity_joint_trajectory_controller.cpp
  )
  target_link_libraries(
    test_gravity_joint_trajectory_controller
    ${PROJECT_NAME}
  )
  ament_target_dependencies(
    test_gravity_joint_trajectory_controller
    hardware_interface
    joint_trajectory_controller
    rclcpp
    trajectory_msgs
  )
  ament_add_pytest_test(
    test_default_controller_config
    test/test_default_controller_config.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  controller_interface
  geometry_msgs
  gloria_ros
  hardware_interface
  imu_sensor_broadcaster
  joint_state_broadcaster
  joint_trajectory_controller
  pluginlib
  rclcpp
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  unitree_api
  unitree_hg
  yaml_cpp_vendor
)

ament_package()