cmake_minimum_required(VERSION 3.5)
project(unitree_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)

set(UNITREE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../example/src/include)

# Go2 bridge node
add_executable(go2_loco_bridge_node src/go2_loco_bridge_node.cpp)
target_include_directories(go2_loco_bridge_node PRIVATE ${UNITREE_INCLUDE_DIR})
ament_target_dependencies(go2_loco_bridge_node rclcpp geometry_msgs unitree_api)

# G1 bridge node
add_executable(g1_loco_bridge_node src/g1_loco_bridge_node.cpp)
target_include_directories(g1_loco_bridge_node PRIVATE ${UNITREE_INCLUDE_DIR})
ament_target_dependencies(g1_loco_bridge_node rclcpp geometry_msgs unitree_api)

install(TARGETS
  go2_loco_bridge_node
  g1_loco_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
