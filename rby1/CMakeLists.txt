cmake_minimum_required(VERSION 3.8)
project(rby1)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(rby1)
ament_python_install_package(rb)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS rby1/bringup_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bringup_node
)

ament_package()
