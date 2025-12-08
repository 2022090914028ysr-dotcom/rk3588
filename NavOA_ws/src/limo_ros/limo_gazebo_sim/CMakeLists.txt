cmake_minimum_required(VERSION 2.8.3)
project(limo_gazebo_sim)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS    
  roslaunch
  roslint
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros

  gazebo_msgs
)
## gazebo_dev is optional: on systems without Gazebo (or cross-compiled targets)
## the package may not be available. Try to find it below without failing the
## whole workspace configuration.
find_package(gazebo_dev QUIET)
if(NOT gazebo_dev_FOUND)
  message(WARNING "gazebo_dev not found — gazebo simulation targets will be disabled. To enable, install Gazebo/ros gazebo packages (e.g. libgazebo11-dev and ros-noetic-gazebo-ros-pkgs) or add gazebo_dev to your CMAKE_PREFIX_PATH.")
endif()
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES gazebo_ros_ackerman is only provided when Gazebo dev headers are available
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

if(gazebo_dev_FOUND)
  # When gazebo_dev is present, build the Gazebo plugin/library
  add_library(gazebo_ros_ackerman src/gazebo_ros_ackerman_drive.cpp)
  target_link_libraries(gazebo_ros_ackerman ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  install(TARGETS gazebo_ros_ackerman
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
else()
  message(STATUS "Skipping build of gazebo_ros_ackerman because gazebo_dev was not found.")
endif()

