cmake_minimum_required(VERSION 3.8)
project(mtc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tl_expected REQUIRED)
find_package(geometric_shapes REQUIRED)

find_package(detector_interfaces REQUIRED)

add_library(tl::expected ALIAS tl_expected::tl_expected)

add_executable(mtc_place_node
  src/mtc_place_node.cpp
)
target_include_directories(mtc_place_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_place_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  detector_interfaces
  tl_expected
  )
target_link_libraries(mtc_place_node
  ${EIGEN3_LIBRARIES}
  /opt/ros/humble/lib/libgeometric_shapes.so
  /opt/ros/humble/lib/x86_64-linux-gnu/liboctomap.so
)

# 3. 安装
install(
  TARGETS mtc_place_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()