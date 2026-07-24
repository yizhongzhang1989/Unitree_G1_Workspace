cmake_minimum_required(VERSION 3.8)
project(canalystii_native_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

add_library(canalystii_protocol
  src/config.cpp
  src/protocol.cpp
  src/transport.cpp
)
target_include_directories(canalystii_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(canalystii_protocol PkgConfig::LIBUSB yaml-cpp)

add_executable(native_bridge_node
  src/kwr57_device_node.cpp
  src/main.cpp
  src/native_bridge_node.cpp
)
target_include_directories(native_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(native_bridge_node
  can_msgs
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(native_bridge_node canalystii_protocol)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS canalystii_protocol native_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol canalystii_protocol)
  ament_add_gtest(test_transport test/test_transport.cpp)
  target_link_libraries(test_transport canalystii_protocol)
  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config canalystii_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(canalystii_protocol)
ament_package()
