cmake_minimum_required(VERSION 2.8.3)
project(caros_sdh)

########################################################################
#### Local Variables
########################################################################
set(library_name ${PROJECT_NAME})

########################################################################
#### Catkin Packages
########################################################################
#find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common)
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole caros_common caros_control)

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
#find_package(RobWorkHardware REQUIRED sdh serialport PATHS "{RWHW_ROOT}")
# Using find_package module version due to finding a FindRobWorkHardware cmake file and the CMAKE_MODULE_PATH above.
# Need to specify CMAKE_BUILD_TYPE=Release to be able to find the proper RobWorkHardware cmake files
set(CMAKE_BUILD_TYPE Release)
find_package(RobWorkHardware COMPONENTS sdh serialport)
# TODO: find RobWorkHardware - see the components/sdh/CMakeLists.txt file for more information on sdh, serialport, PEAK CAN and other things.
# TODO: Keep as much as possible of the functionality from components/sdh/CMakeLists.txt due to not ending up with a broken SDH node.

if (NOT ROBWORKHARDWARE_FOUND)

  message(WARNING "Skipping ${PROJECT_NAME}. Require both SDH and Serialport components of RobWorkHardware!!!")

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

# we use peak so find the library to use
FIND_PACKAGE(SDH REQUIRED)
FIND_PACKAGE(PEAKCAN)

IF(NOT PEAKCAN_FOUND)
  SET(PEAKCAN_INCLUDE_DIR "")
  SET(PEAKCAN_LIBRARIES "")
  MESSAGE(STATUS "The PEAK CAN libraries have not been found. You will only be able to use RS232")
ENDIF(NOT PEAKCAN_FOUND)  

#find_package(RobWorkHardware REQUIRED sdh serialport)

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
message("PEAKCAN_INCLUDE_DIR: ${PEAKCAN_INCLUDE_DIR}")
message("SDH_INCLUDE_DIR: ${SDH_INCLUDE_DIR}")
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
# target_link_libraries(${library_name}
#   ${ROBWORK_LIBRARIES}
#   ${ROBWORKHARDWARE_LIBRARIES}
# )

# # TODO: Fix  
# LINK_DIRECTORIES(
#   ${ROBWORK_LIBRARY_DIRS}
#   ${ROBWORKHARDWARE_LIBRARY_DIRS})

# rosbuild_add_executable(runsdh src/sdh_main.cpp )
# TARGET_LINK_LIBRARIES(runsdh ${PROJECT_NAME} ${ROBWORKHARDWARE_LIBRARIES} ${ROBWORK_LIBRARIES}  ${PEAKCAN_LIBRARIES})

# rosbuild_add_executable(runsdhtest src/sdh_test.cpp )
# TARGET_LINK_LIBRARIES(runsdhtest ${PROJECT_NAME} ${ROBWORKHARDWARE_LIBRARIES} ${ROBWORK_LIBRARIES}  ${PEAKCAN_LIBRARIES})

# rosbuild_add_executable(sdhsim src/sdhsim_main.cpp )
# TARGET_LINK_LIBRARIES(sdhsim ${PROJECT_NAME} ${ROBWORK_LIBRARIES})



# Create one library for the whole package (don't split it into gripper, serialDevice, etc. - if this is wanted then this package should rather be split into gripper, serialDevice and similar packages) 
# add_library(${library_name}
#   src/GripperServiceInterface.cpp
# )

# add_dependencies(${library_name}
# # catkin_EXPORTED_TARGETS should be used if this target depends on things (e.g. message or service headers) that are imported from other catkin packages. (see http://docs.ros.org/hydro/api/catkin/html/howto/building_msgs.html)
# # OBS: But this will cause every target in a package to be required and thereby requiring that target to build code dependent on e.g. RobWork and CoViS.
# # OBS2: What happens once a third package wants to use the second package? (does the third package need to also depend on the first package and making it a very bad design solution to have msg,srv,action in the same package as the interfaces/other API?) or is it not required for msg,srv,action as their dependencies should be resolved through the generate_messages(DEPENDENCIES...)?
# #  ${catkin_EXPORTED_TARGETS}
# # So use specific dependencies instead
#   caros_common_generate_messages_cpp
# #####
#   ${PROJECT_NAME}_generate_messages_cpp
# )

# target_link_libraries(${library_name}
#   ${catkin_LIBRARIES}
#   ${ROBWORK_LIBRARIES}
# )

########################################################################
#### SDH Node
########################################################################
add_executable(sdh_node src/main_SDH.cpp src/SDHNode.cpp)
target_link_libraries(sdh_node
  ${ROBWORK_LIBRARIES}
  ${ROBWORKHARDWARE_LIBRARIES}
  ${PEAKCAN_LIBRARIES}
  ${caros_common_LIBRARIES}
  ${caros_control_LIBRARIES}
  ${catkin_LIBRARIES}
)
# Add dependencies on the caros_common and caros_control targets (those are the make targets for producing the libraries that are being linked with)
add_dependencies(sdh_node
  caros_common
  caros_control
)

endif()
