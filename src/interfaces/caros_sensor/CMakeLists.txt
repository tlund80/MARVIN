cmake_minimum_required(VERSION 2.8.3)
project(caros_sensor)

set(library_name "${PROJECT_NAME}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common_msgs caros_common caros_sensor_msgs)

find_package( OpenCV REQUIRED )

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

# add_message_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs caros_common_msgs caros_common caros_sensor_msgs
  DEPENDS RobWork
)

########################################################################
#### RobWork and RobWorkSim
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
#Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

include_directories(include ${ROBWORK_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${COVIS_INCLUDE_DIR})

add_library(${library_name} 
  src/ButtonSensorSIProxy.cpp
  src/FTSensorServiceInterface.cpp
  src/FTSensorSIProxy.cpp
  src/PoseSensorServiceInterface.cpp
  src/PoseSensorSIProxy.cpp
)

add_dependencies(${library_name}
  caros_sensor_msgs_generate_messages_cpp
  caros_common
)

target_link_libraries(${library_name}
  ${catkin_LIBRARIES}
  ${COVIS_LIBRARIES}
  ${ROBWORK_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
