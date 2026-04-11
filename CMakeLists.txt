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
# ── init_hardware_node (C++) ──────────────────────────────────────────────────
add_executable(init_hardware_node src/init_hardware_node.cpp)
ament_target_dependencies(init_hardware_node rclcpp sensor_msgs)
target_link_libraries(init_hardware_node rby1-sdk::rby1-sdk)

# ── control_cmd_vel_node (C++) ────────────────────────────────────────────────
add_executable(control_cmd_vel_node src/control_cmd_vel_node.cpp)
ament_target_dependencies(control_cmd_vel_node rclcpp geometry_msgs)
target_link_libraries(control_cmd_vel_node rby1-sdk::rby1-sdk)

# ── control_moveit_node (C++) ─────────────────────────────────────────────────
add_executable(control_moveit_node src/control_moveit_node.cpp)
ament_target_dependencies(control_moveit_node
  rclcpp rclcpp_action sensor_msgs control_msgs)
target_link_libraries(control_moveit_node rby1-sdk::rby1-sdk)

# ── control_joint_node (C++) ──────────────────────────────────────────────────
add_executable(control_joint_node src/control_joint_node.cpp)
ament_target_dependencies(control_joint_node rclcpp sensor_msgs)
target_link_libraries(control_joint_node rby1-sdk::rby1-sdk)

# ── lidar_node (C++) ──────────────────────────────────────────────────────────
add_executable(lidar_node src/lidar_node.cpp)
target_include_directories(lidar_node PRIVATE include)
ament_target_dependencies(lidar_node rclcpp sensor_msgs)
target_link_libraries(lidar_node curl)

# ── lidar_merge_node (C++) ────────────────────────────────────────────────────
add_executable(lidar_merge_node src/lidar_merge_node.cpp)
ament_target_dependencies(lidar_merge_node rclcpp sensor_msgs geometry_msgs tf2_ros)

# ── control_zero_pose_node (C++) ──────────────────────────────────────────────
add_executable(control_zero_pose_node src/control_zero_pose_node.cpp)
ament_target_dependencies(control_zero_pose_node rclcpp)
target_link_libraries(control_zero_pose_node rby1-sdk::rby1-sdk)

# ── turn_off_hardware_node (C++) ──────────────────────────────────────────────
add_executable(turn_off_hardware_node src/turn_off_hardware_node.cpp)
ament_target_dependencies(turn_off_hardware_node rclcpp)
target_link_libraries(turn_off_hardware_node rby1-sdk::rby1-sdk)

# ── control_head_gain_node (C++) ──────────────────────────────────────────────
add_executable(control_head_gain_node src/control_head_gain_node.cpp)
ament_target_dependencies(control_head_gain_node rclcpp)
target_link_libraries(control_head_gain_node rby1-sdk::rby1-sdk)

install(
  TARGETS init_hardware_node control_cmd_vel_node control_moveit_node control_joint_node
          lidar_node lidar_merge_node
          control_zero_pose_node turn_off_hardware_node control_head_gain_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── 공유 디렉터리 ─────────────────────────────────────────────────────────────
install(
  DIRECTORY launch urdf meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
