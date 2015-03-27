cmake_minimum_required(VERSION 2.8.3)
project(caros_pg70)

########################################################################
#### Local Variables
########################################################################
set(library_name ${PROJECT_NAME})

########################################################################
#### Catkin Packages
########################################################################
#find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common)
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole caros_common caros_control)

find_package(
  CoViS
  REQUIRED
  #HINTS $ENV{DTI_COMMON_ROOT}/CoViS
)
LINK_DIRECTORIES(${COVIS_LIBRARY_DIRS})

########################################################################
#### RobWork
#### TODO:
#### Once RobWorkHardware gets a RobWorkHardware cmake config setup, then the find_package(RobWorkHardware ...) and supporting cmake configuration should be updated.
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
# The following CMAKE_MODULE_PATH configuration is required due to RobWorkHardware placing the things in build instead of cmake as RobWork is doing.
set(CMAKE_MODULE_PATH ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork.")
ENDIF(NOT ROBWORK_FOUND)

# Using find_package module version due to finding a FindRobWorkHardware cmake file and the CMAKE_MODULE_PATH above.
# Need to specify CMAKE_BUILD_TYPE=Release to be able to find the proper RobWorkHardware cmake files
set(CMAKE_BUILD_TYPE Release)
find_package(RobWorkHardware COMPONENTS schunkpg70 pcube serialport universalrobots)

if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Skipping ${PROJECT_NAME}. Require both schunkpg70, pcube, serialport and universalrobots components of RobWorkHardware!!!")
else()


########################################################################
#### Old SDH CMake setup
########################################################################
#SET(RWHW_ROOT "$ENV{RWHW_ROOT}")
#The following is a workaround for the old version of RobWork. Remove once no longer needed (everyone switched to a RobWork version newer than April 2012)
#SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})
#IF(NOT ROBWORK_FOUND)
#  FIND_PACKAGE(RobWork REQUIRED)
#ENDIF(NOT ROBWORK_FOUND)


################################################
## Declare ROS messages, services and actions ##
################################################
# add_service_files(
#   FILES
#   # SerialDeviceForceControlStart.srv
#   # SerialDeviceForceControlStop.srv
#   # SerialDeviceForceControlUpdate.srv
#   # SerialDeviceMoveLin.srv
#   # SerialDeviceMoveLinFC.srv
#   # SerialDeviceMovePTP_T.srv
#   # SerialDeviceMovePTP.srv
#   # SerialDeviceMoveVelQ.srv
#   # SerialDeviceMoveVelT.srv
#   GripperMoveQ.srv
#   GripperGripQ.srv
#   GripperSetForceQ.srv
#   GripperSetVelocityQ.srv
#   GripperStopMovement.srv
# )

# add_message_files(
#   FILES
# #  RobotState.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   caros_common
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${library_name}
  # CATKIN_DEPENDS:
  # - rosconsole is not used in the header files, so should not be added to the list here.
  CATKIN_DEPENDS roscpp caros_common caros_control
  DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
message("---------- include directories ----------")
message("ROBWORK_INCLUDE_DIR: ${ROBWORK_INCLUDE_DIR}")
message("ROBWORKHARDWARE_INCLUDE_DIRS: ${ROBWORKHARDWARE_INCLUDE_DIRS}")
message("catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message("-----------------------------------------")
include_directories(
  include
  ${ROBWORK_INCLUDE_DIR}
  ${ROBWORKHARDWARE_INCLUDE_DIRS}
  ${PEAKCAN_INCLUDE_DIR}
  ${SDH_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# add_library(${library_name}
#   src/SDH.cpp
#   src/SDHSim.cpp
# )

message("---------- RobWorkHardware linking --------")
message("rwhw libraries: ${ROBWORKHARDWARE_LIBRARIES}")
message("rw libraries: ${ROBWORK_LIBRARIES}")
message("-------------------------------------------")

########################################################################
#### pg70 Node
########################################################################
set(SRCFILES 
    #src/PG70ServiceInterface.cpp
    src/PG70Node.cpp
   # src/PG70Sim.cpp
)

add_library(${PROJECT_NAME}_lib STATIC ${SRCFILES})

add_executable(${PROJECT_NAME} src/pg70_main.cpp )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PROJECT_NAME}_lib ${ROBWORKHARDWARE_LIBRARIES} ${ROBWORK_LIBRARIES} ${catkin_LIBRARIES} )

# Add dependencies on the caros_common and caros_control targets (those are the make targets for producing the libraries that are being linked with)
#add_dependencies(sdh_node
#  caros_common
#  caros_control
#)

endif()
