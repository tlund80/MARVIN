cmake_minimum_required(VERSION 2.8.3)
project(caros_robotiq)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs caros_common caros_control)

########################################################################
#### CoViS
########################################################################
find_package(
  CoViS
  REQUIRED
  #HINTS $ENV{DTI_COMMON_ROOT}/CoViS
)
LINK_DIRECTORIES(${COVIS_LIBRARY_DIRS})

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
  CATKIN_DEPENDS message_runtime roscpp std_msgs caros_common caros_control
)

########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################
########################################################################
#### RobWork and RobWorkSim
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
message("RW_ROOT: ${RWHW_ROOT}/build")

SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
#message("robwork libraries: ${ROBWORK_LIBRARIES}")
include_directories(include ${ROBWORK_INCLUDE_DIR} ${caros_control_INCLUDE_DIRS})
#link_directories(${ROBWORK_LIBRARY_DIRS})
#[ mband: This is not recommended ] - link_directories( ${ROBWORK_LIBRARY_DIRS})
SET(CMAKE_MODULE_PATH ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})
find_package(RobWorkHardware REQUIRED COMPONENTS robotiq)
include_directories(include ${ROBWORKHARDWARE_INCLUDE_DIR} ${COVIS_INCLUDE_DIR})

# mband: link_directories is required until RobWork issue ## is solved
#link_directories(${ROBWORKHARDWARE_LIBRARY_DIRS})

#MESSAGE("ROBWORKHARDWARE_LIBRARY_DIRS ${ROBWORKHARDWARE_LIBRARY_DIRS}")
#MESSAGE("ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES}")
MESSAGE(" caros_control_LIBRARIES : ${catkin_LIBRARIES}")
add_executable(robotiq3 src/mainR3.cpp src/Robotiq3Node.cpp)
target_link_libraries(robotiq3 ${conv_library} 
  ${ROBWORK_LIBRARIES} 
  ${ROBWORKHARDWARE_LIBRARIES} 
  ${COVIS_LIBRARIES}
  ${caros_common_LIBRARIES} 
  ${caros_control_LIBRARIES} 
  ${catkin_LIBRARIES}
)
add_dependencies(robotiq3
  caros_common
  caros_control
)
