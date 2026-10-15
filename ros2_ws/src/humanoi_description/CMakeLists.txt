cmake_minimum_required(VERSION 3.8)
project(humanoi_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf srdf worlds launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
