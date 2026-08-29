cmake_minimum_required(VERSION 3.8)
project(zhirong_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

