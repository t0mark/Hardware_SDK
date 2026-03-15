cmake_minimum_required(VERSION 3.8)
project(rby1)

set(CMAKE_CXX_STANDARD 17)

# ---------------------------------------------------------------------------
# ROS dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Protobuf (cmake's built-in FindProtobuf works even without config file)
# ---------------------------------------------------------------------------
find_package(Protobuf REQUIRED)

# ---------------------------------------------------------------------------
# gRPC — try config-mode first, fall back to find_library
# ---------------------------------------------------------------------------
find_package(gRPC CONFIG QUIET)

if(gRPC_FOUND)
  message(STATUS "Found gRPC via cmake config")
  set(GRPC_LIBRARIES gRPC::grpc++)
  get_target_property(GRPC_CPP_PLUGIN gRPC::grpc_cpp_plugin LOCATION)
else()
  message(STATUS "gRPCConfig.cmake not found, falling back to find_library")

  find_library(GRPC_LIBRARY     grpc    REQUIRED)
  find_library(GRPCPP_LIBRARY   grpc++  REQUIRED)
  find_library(GPR_LIBRARY      gpr     REQUIRED)

  set(GRPC_LIBRARIES ${GRPCPP_LIBRARY} ${GRPC_LIBRARY} ${GPR_LIBRARY})

  find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED
    HINTS /usr/bin /usr/local/bin)

  # Compatibility target so protobuf_generate() can use it
  if(NOT TARGET gRPC::grpc_cpp_plugin)
    add_executable(gRPC::grpc_cpp_plugin IMPORTED)
    set_target_properties(gRPC::grpc_cpp_plugin PROPERTIES
      IMPORTED_LOCATION "${GRPC_CPP_PLUGIN}")
  endif()
  if(NOT TARGET gRPC::grpc++)
    add_library(gRPC::grpc++ UNKNOWN IMPORTED)
    set_target_properties(gRPC::grpc++ PROPERTIES
      IMPORTED_LOCATION "${GRPCPP_LIBRARY}")
  endif()
endif()

message(STATUS "gRPC libraries: ${GRPC_LIBRARIES}")
message(STATUS "grpc_cpp_plugin: ${GRPC_CPP_PLUGIN}")

# ---------------------------------------------------------------------------
# Proto code generation
# ---------------------------------------------------------------------------
set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/_proto_gen")
set(PROTO_IMPORT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
file(MAKE_DIRECTORY ${PROTO_BINARY_DIR})

file(GLOB_RECURSE PROTO_FILES "${PROTO_IMPORT_DIR}/*.proto")

# ---------------------------------------------------------------------------
# Explicit per-file protoc invocations
# Using add_custom_command ensures -I ${PROTO_IMPORT_DIR} is used consistently,
# so all generated symbols are encoded as rb_2fapi_2f... (not proto_2frb_2fapi_2f...)
# and output lands at _proto_gen/rb/api/*.{pb,grpc.pb}.{h,cc}
# ---------------------------------------------------------------------------
set(ALL_PROTO_SRCS "")

foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  set(PB_CC  "${PROTO_BINARY_DIR}/rb/api/${PROTO_NAME}.pb.cc")
  set(PB_H   "${PROTO_BINARY_DIR}/rb/api/${PROTO_NAME}.pb.h")
  set(GRC_CC "${PROTO_BINARY_DIR}/rb/api/${PROTO_NAME}.grpc.pb.cc")
  set(GRC_H  "${PROTO_BINARY_DIR}/rb/api/${PROTO_NAME}.grpc.pb.h")

  add_custom_command(
    OUTPUT ${PB_CC} ${PB_H} ${GRC_CC} ${GRC_H}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      --cpp_out=${PROTO_BINARY_DIR}
      --grpc_out=${PROTO_BINARY_DIR}
      --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
      -I ${PROTO_IMPORT_DIR}
      ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating proto/gRPC code for ${PROTO_NAME}"
  )
  list(APPEND ALL_PROTO_SRCS ${PB_CC} ${GRC_CC})
endforeach()

# Wrap generated sources in a static library so rby1_hardware can link them
add_library(rby1_proto_objects STATIC ${ALL_PROTO_SRCS})
set_property(TARGET rby1_proto_objects PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(rby1_proto_objects PUBLIC
  protobuf::libprotobuf
  gRPC::grpc++
)
# Generated files land at _proto_gen/rb/api/*.pb.h
target_include_directories(rby1_proto_objects PUBLIC
  ${PROTO_BINARY_DIR}
)

# ---------------------------------------------------------------------------
# Hardware interface plugin (shared library)
# ---------------------------------------------------------------------------
add_library(rby1_hardware SHARED
  src/rby1_hardware_interface.cpp
)

target_include_directories(rby1_hardware PUBLIC
  include
  ${PROTO_BINARY_DIR}
)

target_link_libraries(rby1_hardware
  rby1_proto_objects
  ${GRPC_LIBRARIES}
  protobuf::libprotobuf
)

ament_target_dependencies(rby1_hardware
  rclcpp
  hardware_interface
  pluginlib
  geometry_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface rby1_hardware_plugins.xml)



# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS rby1_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY meshes urdf launch rviz config proto
  DESTINATION share/${PROJECT_NAME}
)

install(FILES rby1_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
ament_package()
