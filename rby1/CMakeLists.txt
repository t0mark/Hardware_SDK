cmake_minimum_required(VERSION 3.8)
project(rby1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rby1-sdk REQUIRED)

# ── hardware_node (C++) ───────────────────────────────────────────────────────
add_executable(hardware_node src/hardware_node.cpp)
ament_target_dependencies(hardware_node rclcpp sensor_msgs)
target_link_libraries(hardware_node rby1-sdk::rby1-sdk)

# ── mobility_node (C++) ───────────────────────────────────────────────────────
add_executable(mobility_node src/mobility_node.cpp)
ament_target_dependencies(mobility_node rclcpp geometry_msgs)
target_link_libraries(mobility_node rby1-sdk::rby1-sdk)

install(
  TARGETS hardware_node mobility_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── 공유 디렉터리 ─────────────────────────────────────────────────────────────
install(
  DIRECTORY launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
