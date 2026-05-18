cmake_minimum_required(VERSION 3.8)
project(swerve_description)

find_package(ament_cmake REQUIRED)

# Install semua file resource
install(DIRECTORY
  urdf
  launch
  worlds
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
