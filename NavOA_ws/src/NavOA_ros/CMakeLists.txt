cmake_minimum_required(VERSION 3.5)
project(NavOA_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf
  rosgraph_msgs
  cartographer_ros_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/RefSpeed.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES NavOA_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/NavOA_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/NavOA_ros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Compile Python bytecode and install only compiled artifacts.
file(GLOB_RECURSE PYTHON_SCRIPT_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)

set(PYTHON_SOURCE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_custom_target(compile_python_bytecode ALL
  COMMAND ${Python3_EXECUTABLE} -m compileall -b -f -q ${PYTHON_SOURCE_DIRS}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Compile Python sources into bytecode (.pyc)"
  VERBATIM
)

set(PYTHON_WRAPPER_DIR ${CMAKE_CURRENT_BINARY_DIR}/pywrappers)
file(MAKE_DIRECTORY ${PYTHON_WRAPPER_DIR})

set(PYTHON_WRAPPER_FILES "")
foreach(PYTHON_SCRIPT ${PYTHON_SCRIPT_FILES})
  get_filename_component(PYTHON_NAME ${PYTHON_SCRIPT} NAME_WE)
  set(PYTHON_WRAPPER ${PYTHON_WRAPPER_DIR}/${PYTHON_NAME})
  add_custom_command(
    OUTPUT ${PYTHON_WRAPPER}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PYTHON_WRAPPER_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "#!/usr/bin/env sh" > ${PYTHON_WRAPPER}
    COMMAND ${CMAKE_COMMAND} -E echo "DIR=\"$$(cd \"$$(dirname \"$$0\")\" && pwd)\"" >> ${PYTHON_WRAPPER}
    COMMAND ${CMAKE_COMMAND} -E echo "exec \"${Python3_EXECUTABLE}\" \"$$(cd \"$$(dirname \"$$0\")\" && pwd)/../share/${PROJECT_NAME}/scripts/${PYTHON_NAME}.pyc\" \"$$@\"" >> ${PYTHON_WRAPPER}
    COMMAND ${CMAKE_COMMAND} -E chmod +x ${PYTHON_WRAPPER}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${PYTHON_SCRIPT}
  )
  list(APPEND PYTHON_WRAPPER_FILES ${PYTHON_WRAPPER})
endforeach()

add_custom_target(generate_python_wrappers
  DEPENDS ${PYTHON_WRAPPER_FILES}
)
add_dependencies(generate_python_wrappers compile_python_bytecode)

# Install Python script entry points directly (avoid relying on generated wrappers
# which can fail during shell execution in some build environments). This will
# place the original script files in the package `bin` so they are runnable.
set(PYTHON_SCRIPT_FULL_PATHS "")
foreach(_py ${PYTHON_SCRIPT_FILES})
  if(NOT _py MATCHES "/__init__\\.py$")
    list(APPEND PYTHON_SCRIPT_FULL_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_py})
  endif()
endforeach()

install(PROGRAMS ${PYTHON_SCRIPT_FULL_PATHS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY NavOA
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.pyc"
  PATTERN "*.py" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY cfg example launch maps param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.cpp" EXCLUDE
  PATTERN "*.cc" EXCLUDE
  PATTERN "*.c" EXCLUDE
  PATTERN "*.h" EXCLUDE
  PATTERN "*.hpp" EXCLUDE
)

## Raw runtime Python source files are installed for deployment so ROS nodes
## and Python imports resolve correctly after package installation.

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_NavOA_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
