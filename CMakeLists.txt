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

# ── cmd_vel_node (C++) ───────────────────────────────────────────────────────
add_executable(cmd_vel_node src/cmd_vel_node.cpp)
ament_target_dependencies(cmd_vel_node rclcpp geometry_msgs)
target_link_libraries(cmd_vel_node rby1-sdk::rby1-sdk)

# ── moveit_node (C++) ─────────────────────────────────────────────────────────
add_executable(moveit_node src/moveit_node.cpp)
ament_target_dependencies(moveit_node
  rclcpp rclcpp_action sensor_msgs control_msgs)
target_link_libraries(moveit_node rby1-sdk::rby1-sdk)

# ── joint_control_node (C++) ─────────────────────────────────────────────────
add_executable(joint_control_node src/joint_control_node.cpp)
ament_target_dependencies(joint_control_node rclcpp sensor_msgs)
target_link_libraries(joint_control_node rby1-sdk::rby1-sdk)

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

# ── head_gain_node (C++) ──────────────────────────────────────────────────────
add_executable(head_gain_node src/head_gain_node.cpp)
ament_target_dependencies(head_gain_node rclcpp)
target_link_libraries(head_gain_node rby1-sdk::rby1-sdk)

install(
  TARGETS hardware_node cmd_vel_node moveit_node joint_control_node
          lidar_node merge_laserscan_node
          zero_pose_node turn_off_node head_gain_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── 공유 디렉터리 ─────────────────────────────────────────────────────────────
install(
  DIRECTORY launch urdf meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
