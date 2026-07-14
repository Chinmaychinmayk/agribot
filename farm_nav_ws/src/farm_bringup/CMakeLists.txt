cmake_minimum_required(VERSION 3.8)
project(farm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
