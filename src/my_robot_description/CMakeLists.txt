cmake_minimum_required(VERSION 3.10)
project(my_robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY urdf mjcf meshes config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
