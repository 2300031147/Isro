cmake_minimum_required(VERSION 3.5)
project(ascend_precision_landing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/precision_landing.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
