cmake_minimum_required(VERSION 3.5)
project(runner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  script/runner_node.py
  script/mode_manager.py
  script/status_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

# Install param files
install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  USE_SOURCE_PERMISSIONS
)

# Install shell scripts
install(PROGRAMS
  script/run.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/script
)
