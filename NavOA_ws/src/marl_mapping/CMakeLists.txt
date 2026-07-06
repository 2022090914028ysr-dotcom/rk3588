cmake_minimum_required(VERSION 3.5)
project(marl_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  cv_bridge
)

catkin_package()

# Setup Python package
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/exploration_node.py
  scripts/robot_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python package
install(DIRECTORY src/marl_mapping
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
