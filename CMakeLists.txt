cmake_minimum_required(VERSION 3.8)
project(rby1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rby1-sdk REQUIRED)
find_package(CURL REQUIRED)
find_package(tf2_ros REQUIRED)
# ── hardware_node (C++) ───────────────────────────────────────────────────────
add_executable(hardware_node src/hardware_node.cpp)
ament_target_dependencies(hardware_node rclcpp sensor_msgs)
target_link_libraries(hardware_node rby1-sdk::rby1-sdk)

# ── mobility_node (C++) ───────────────────────────────────────────────────────
add_executable(mobility_node src/mobility_node.cpp)
ament_target_dependencies(mobility_node rclcpp geometry_msgs)
target_link_libraries(mobility_node rby1-sdk::rby1-sdk)

# ── wholebody_control_node (C++) ──────────────────────────────────────────────
add_executable(wholebody_control_node src/wholebody_control_node.cpp)
ament_target_dependencies(wholebody_control_node
  rclcpp rclcpp_action sensor_msgs control_msgs)
target_link_libraries(wholebody_control_node rby1-sdk::rby1-sdk)

# ── lidar_node (C++) ──────────────────────────────────────────────────────────
add_executable(lidar_node src/lidar_node.cpp src/lidar_remote.cpp)
target_include_directories(lidar_node PRIVATE include)
ament_target_dependencies(lidar_node rclcpp sensor_msgs)
target_link_libraries(lidar_node curl)


# ── merge_laserscan_node (C++) ────────────────────────────────────────────────
add_executable(merge_laserscan_node src/merge_laserscan_node.cpp)
ament_target_dependencies(merge_laserscan_node rclcpp sensor_msgs geometry_msgs tf2_ros)

# ── zero_pose_node (C++) ──────────────────────────────────────────────────────
add_executable(zero_pose_node src/zero_pose_node.cpp)
ament_target_dependencies(zero_pose_node rclcpp)
target_link_libraries(zero_pose_node rby1-sdk::rby1-sdk)

# ── turn_off_node (C++) ───────────────────────────────────────────────────────
add_executable(turn_off_node src/turn_off_node.cpp)
ament_target_dependencies(turn_off_node rclcpp)
target_link_libraries(turn_off_node rby1-sdk::rby1-sdk)

# ── gain_node (C++) ───────────────────────────────────────────────────────────
add_executable(gain_node src/gain_node.cpp)
ament_target_dependencies(gain_node rclcpp)
target_link_libraries(gain_node rby1-sdk::rby1-sdk)

install(
  TARGETS hardware_node mobility_node wholebody_control_node lidar_node merge_laserscan_node
          zero_pose_node turn_off_node gain_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── 공유 디렉터리 ─────────────────────────────────────────────────────────────
install(
  DIRECTORY launch urdf meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
