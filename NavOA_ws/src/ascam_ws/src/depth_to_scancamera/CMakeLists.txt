cmake_minimum_required(VERSION 3.5)
project(depth_to_scancamera)

# C++14 ±ê×¼
add_compile_options(-std=c++14)

# ²éÕÒÒÀÀµ°ü
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
)

# Catkin °üÅäÖÃ
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_transport
    image_geometry
)

# °üº¬Ä¿Â¼
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ¿ÉÖ´ÐÐÎÄ¼þ
add_executable(
  depth_to_scancamera_node
  src/depth_to_scancamera_node.cpp
  src/depth_to_scancamera.cpp
)

# Á´½Ó¿â
target_link_libraries(
  depth_to_scancamera_node
  ${catkin_LIBRARIES}
)

# °²×°
install(
  TARGETS depth_to_scancamera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
