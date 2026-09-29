cmake_minimum_required(VERSION 3.8)
project(detector)

## 使用 C++17，REQUIRED 保证 -std=gnu++17 写入 compile_commands.json（gcc 默认已是 17，CMake 否则会省略该参数）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(detector_interfaces REQUIRED)

find_package(yaml-cpp REQUIRED) # 新增 YAML 解析库

# 1. 编译为组件库
add_library(detector_component SHARED
  src/detector_node.cpp
  src/detector.cpp
)
target_include_directories(detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(detector_component
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  cv_bridge image_transport 
  geometry_msgs 
  visualization_msgs 
  tf2_geometry_msgs OpenCV tf2_ros detector_interfaces
)

# 2. 编译独立可执行文件
add_executable(detector_node src/detector_node_main.cpp)
target_link_libraries(detector_node detector_component)
ament_target_dependencies(detector_node rclcpp)

add_executable(detector_main src/detector_main.cpp src/detector.cpp)
target_link_libraries(detector_main 
    ${OpenCV_LIBS}
    yaml-cpp
)
target_include_directories(detector_main PRIVATE include)

# 3. 注册组件 (供 container 调用)
rclcpp_components_register_nodes(detector_component "exchange_slot::ExchangeSlotDetectorNode")

# 4. 安装
install(TARGETS detector_component detector_node detector_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()