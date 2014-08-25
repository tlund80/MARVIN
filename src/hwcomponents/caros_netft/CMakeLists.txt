cmake_minimum_required(VERSION 2.8.3)
project(netft)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs caros_common)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  caros_common
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs caros_common
)

########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################
########################################################################
#### RobWork and RobWorkSim
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
#message("RW_ROOT: ${RW_ROOT}")
#Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
#message("robwork libraries: ${ROBWORK_LIBRARIES}")
include_directories(include ${ROBWORK_INCLUDE_DIR})
#link_directories(${ROBWORK_LIBRARY_DIRS})
#[ mband: This is not recommended ] - link_directories( ${ROBWORK_LIBRARY_DIRS})

#message("rw inc dir" ${ROBWORK_INCLUDE_DIR})
#message("rw link dir" ${ROBWORK_LIBRARY_DIRS})
#message("rw libraries" ${ROBWORK_LIBRARIES})
#message("rw libs" ${ROBWORK_LIBS})

# mband: link_directories is required until RobWork issue ## is solved
######link_directories(${ROBWORK_LIBRARY_DIRS})

#add_executable(caros_test src/Caros_Test.cpp)
#target_link_libraries(caros_test ${conv_library} ${ROBWORK_LIBRARIES})
