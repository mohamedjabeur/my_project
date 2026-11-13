cmake_minimum_required(VERSION 3.8)
project(vehicle_drivers)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/camera_driver.py scripts/cloud_to_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
