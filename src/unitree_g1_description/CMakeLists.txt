cmake_minimum_required(VERSION 3.8)
project(unitree_g1_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()