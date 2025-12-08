cmake_minimum_required(VERSION 2.8)
project(lidar_ros)

# set base info
set(CMAKE_CXX_STANDARD 11)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)  

#find package 
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

# catkin package
catkin_package()

# add include 
include_directories(
  ${catkin_INCLUDE_DIRS}
  sdk/include
  ros/include
)

# add src
FILE(GLOB LIDAR_SDK_SRC 
  "sdk/src/lidar/lidar_protocol.cpp"
  "sdk/src/lidar.cpp"
  "sdk/src/interface/console/interface_console.cpp"
)
if(WIN32)  
  list(APPEND LIDAR_SDK_SRC "sdk/src/interface/serial/win/interface_serial.cpp")  
elseif(UNIX)  
  list(APPEND LIDAR_SDK_SRC "sdk/src/interface/serial/unix/interface_serial.cpp")  
endif()

#link client
add_executable(lidar_client ros/src/ros_client.cpp)
target_link_libraries(lidar_client ${catkin_LIBRARIES})
#link node 
add_executable(lidar_node ros/src/ros_node.cpp  ros/src/ros_convert.cpp ${LIDAR_SDK_SRC})
target_link_libraries(lidar_node pthread ${catkin_LIBRARIES})

#install
install(TARGETS lidar_node lidar_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rules rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

