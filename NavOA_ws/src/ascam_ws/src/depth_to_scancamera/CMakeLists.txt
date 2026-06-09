cmake_minimum_required(VERSION 3.5)
project(depth_to_scancamera)

# C++14 标准
add_compile_options(-std=c++14)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
)

# Catkin 包配置
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_transport
    image_geometry
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 可执行文件
add_executable(
  depth_to_scancamera_node
  src/depth_to_scancamera_node.cpp
  src/depth_to_scancamera.cpp
)

# 链接库
target_link_libraries(
  depth_to_scancamera_node
  ${catkin_LIBRARIES}
)

# 安装
install(
  TARGETS depth_to_scancamera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
