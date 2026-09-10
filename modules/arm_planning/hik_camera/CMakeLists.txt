cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## 使用 C++17 以更好地支持 cv_bridge 和现代 ROS 2 接口
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 ROS 2 基础依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 查找 OpenCV (用于视频/图片读取)
find_package(OpenCV REQUIRED)

# 添加组件库
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
)

# 包含头文件路径
target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)

# 根据架构选择海康 SDK 库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(HIK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HIK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64")
endif()

target_link_directories(${PROJECT_NAME} PUBLIC ${HIK_LIB_DIR})

# 链接海康动态库及 OpenCV
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  MvCameraControl
  FormatConversion
  MediaProcess
  MVRender
  MvUsb3vTL
)

# 注册为 ROS 2 组件并生成可执行程序
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE hik_camera_node
)

# 安装 SDK 库到 lib 目录，确保运行时能找到 .so 文件
install(
  DIRECTORY ${HIK_LIB_DIR}/
  DESTINATION lib
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)