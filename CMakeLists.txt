cmake_minimum_required(VERSION 3.20)
project(cuopt_intra_factory_viz LANGUAGES CXX CUDA)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# vehicle_tf_broadcaster_node solves the Intra-factory Transport scenario by
# linking directly against libcuopt (cuopt_solver.cu), the same way
# cuopt_routing_node does -- see that package's CMakeLists.txt/README for why
# CUOPT_SDK_PREFIX + this exact RMM/RAFT setup is needed.
if(NOT DEFINED ENV{CUOPT_SDK_PREFIX})
  message(FATAL_ERROR "請先 export CUOPT_SDK_PREFIX=<conda env 路徑>")
endif()
set(CUOPT_SDK_PREFIX $ENV{CUOPT_SDK_PREFIX})
list(APPEND CMAKE_PREFIX_PATH ${CUOPT_SDK_PREFIX})

set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_ARCHITECTURES native)

find_package(rmm CONFIG REQUIRED)
find_package(raft CONFIG REQUIRED)

find_path(CUOPT_INCLUDE_DIR
  NAMES cuopt/routing/solve.hpp
  HINTS ${CUOPT_SDK_PREFIX}/include
  REQUIRED)
find_library(CUOPT_LIBRARY cuopt
  HINTS ${CUOPT_SDK_PREFIX}/lib REQUIRED)

add_library(waypoint_graph STATIC src/waypoint_graph.cpp src/shortest_path.cpp)
target_include_directories(waypoint_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(diff_drive_path STATIC src/diff_drive_path.cpp)
target_link_libraries(diff_drive_path PUBLIC waypoint_graph)
target_include_directories(diff_drive_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# The only target in this package that touches CUDA/RAFT/RMM/libcuopt --
# everything else (waypoint_graph, diff_drive_path, waypoint_marker_publisher_node)
# stays plain C++.
add_library(cuopt_solver STATIC src/cuopt_solver.cu)
target_include_directories(cuopt_solver PRIVATE ${CUOPT_INCLUDE_DIR})
target_link_libraries(cuopt_solver PUBLIC waypoint_graph PRIVATE ${CUOPT_LIBRARY} rmm::rmm raft::raft)
set_target_properties(cuopt_solver PROPERTIES
  BUILD_RPATH   "${CUOPT_SDK_PREFIX}/lib"
  INSTALL_RPATH "${CUOPT_SDK_PREFIX}/lib")

add_executable(waypoint_marker_publisher_node src/waypoint_marker_publisher_node.cpp)
target_link_libraries(waypoint_marker_publisher_node waypoint_graph)
ament_target_dependencies(waypoint_marker_publisher_node rclcpp geometry_msgs visualization_msgs)

add_executable(vehicle_tf_broadcaster_node src/vehicle_tf_broadcaster_node.cpp)
target_link_libraries(vehicle_tf_broadcaster_node diff_drive_path waypoint_graph cuopt_solver)
ament_target_dependencies(vehicle_tf_broadcaster_node rclcpp tf2 tf2_ros geometry_msgs visualization_msgs)
set_target_properties(vehicle_tf_broadcaster_node PROPERTIES
  BUILD_RPATH   "${CUOPT_SDK_PREFIX}/lib"
  INSTALL_RPATH "${CUOPT_SDK_PREFIX}/lib")

install(TARGETS waypoint_marker_publisher_node vehicle_tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
