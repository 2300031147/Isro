cmake_minimum_required(VERSION 3.5)
project(ascend_bringup)

find_package(ament_cmake REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
