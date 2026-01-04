cmake_minimum_required(VERSION 3.5)
project(ascend_rock_detection)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/rock_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
