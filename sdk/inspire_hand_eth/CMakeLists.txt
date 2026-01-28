cmake_minimum_required(VERSION 3.8)
project(inspire_hand_eth)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===========================================
# Find dependencies
# ===========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Modbus library
find_library(MODBUS_LIBRARIES NAMES modbus REQUIRED)
find_path(MODBUS_INCLUDE_DIR modbus/modbus.h)

# yaml-cpp library
find_package(yaml-cpp REQUIRED)

# ===========================================
# Include directories
# ===========================================
include_directories(
  include
  ${MODBUS_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

# ===========================================
# Library: Modbus Client
# ===========================================
add_library(modbus_client SHARED
  src/modbus_client.cpp
)
target_link_libraries(modbus_client
  ${MODBUS_LIBRARIES}
)
ament_target_dependencies(modbus_client
  rclcpp
)

# ===========================================
# Executable: Service Server Node
# ===========================================
add_executable(inspire_service_server
  src/bringup_service/service_server_node.cpp
)
target_link_libraries(inspire_service_server
  modbus_client
  yaml-cpp
)
ament_target_dependencies(inspire_service_server
  rclcpp
  service_interfaces
  std_msgs
  ament_index_cpp
)

# ===========================================
# Executable: Topic Node
# ===========================================
add_executable(inspire_topic_node
  src/bringup_topic/topic_node.cpp
)
target_link_libraries(inspire_topic_node
  modbus_client
  yaml-cpp
)
ament_target_dependencies(inspire_topic_node
  rclcpp
  service_interfaces
  std_msgs
  ament_index_cpp
)

# ===========================================
# Executable: Test Node
# ===========================================
add_executable(inspire_test_node
  src/test_node.cpp
)
target_link_libraries(inspire_test_node
  modbus_client
  yaml-cpp
)
ament_target_dependencies(inspire_test_node
  rclcpp
  ament_index_cpp
)

# ===========================================
# Install
# ===========================================
# Executables
install(TARGETS
  inspire_service_server
  inspire_topic_node
  inspire_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Library
install(TARGETS
  modbus_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Headers
install(DIRECTORY include/
  DESTINATION include
)

# Config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ===========================================
# Testing
# ===========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ===========================================
# Export
# ===========================================
ament_export_include_directories(include)
ament_export_libraries(modbus_client)
ament_export_dependencies(rclcpp)

ament_package()
