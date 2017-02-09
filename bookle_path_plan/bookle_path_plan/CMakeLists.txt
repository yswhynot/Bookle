cmake_minimum_required(VERSION 2.8.3)
project(bookle_path_plan)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  message_generation
  nav_msgs
  nodelet
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PathPlan.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_planner
 CATKIN_DEPENDS cmake_modules geometry_msgs message_runtime nav_msgs nodelet roscpp std_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(path_planner
  src/path_planner.cpp
)
target_link_libraries(path_planner
  ${catkin_LIBRARIES}
)

add_dependencies(path_planner 
path_planner_generate_messages_cpp
)

add_library(path_planner_nodelet src/path_planner_nodelet.cpp)
target_link_libraries(path_planner_nodelet
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(path_planner_node src/path_planner_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
  path_planner
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS path_planner path_planner_node path_planner_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/bookle_path_plan/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
