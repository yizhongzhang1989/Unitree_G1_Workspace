cmake_minimum_required(VERSION 3.8)
project(unitree_g1_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gloria_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(imu_sensor_broadcaster REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
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
  src/forward_position_controller.cpp
  src/g1_topic_system.cpp
  src/throttled_broadcasters.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  geometry_msgs
  gloria_ros
  hardware_interface
  imu_sensor_broadcaster
  joint_state_broadcaster
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(
    test_periodic_deadline
    test/test_periodic_deadline.cpp
  )
  target_include_directories(test_periodic_deadline PRIVATE include)
  ament_add_pytest_test(
    test_default_controller_config
    test/test_default_controller_config.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  geometry_msgs
  gloria_ros
  hardware_interface
  imu_sensor_broadcaster
  joint_state_broadcaster
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