cmake_minimum_required(VERSION 2.8.3)
project(universalrobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs caros_common caros_control caros_control_msgs)

find_package(Boost REQUIRED QUIET COMPONENTS thread)

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
  CATKIN_DEPENDS message_runtime std_msgs caros_common caros_control caros_control_msgs
)

########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################
########################################################################
#### Find RobWork
########################################################################
SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWHW_ROOT "$ENV{RWHW_ROOT}")
SET(CMAKE_BUILD_TYPE Release)

FIND_PACKAGE(
  RobWork
  QUIET
  HINTS
    "${RW_ROOT}")
IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork with the new method. Trying the old one.")
  INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
ENDIF(NOT ROBWORK_FOUND)

SET(CMAKE_MODULE_PATH ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})
find_package(RobWorkHardware REQUIRED COMPONENTS universalrobots)

#link_directories(${ROBWORK_LIBRARY_DIRS})
#link_directories(${ROBWORKSTUDIO_LIBRARY_DIRS})
#link_directories(${ROBWORKSIM_LIBRARY_DIRS})

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${caros_control_INCLUDE_DIRS}
  ${caros_common_INCLUDE_DIRS}
  ${COVIS_INCLUDE_DIR}
  ${ROBWORKHARDWARE_INCLUDE_DIR}
  ${ROBWORK_INCLUDE_DIR})
#link_directories(${ROBWORK_LIBRARY_DIRS})
#link_directories(${ROBWORKHARDWARE_DIRS})
#[ mband: This is not recommended ] - link_directories( ${ROBWORK_LIBRARY_DIRS})

#message("rw inc dir" ${ROBWORK_INCLUDE_DIR})
#message("rw link dir" ${ROBWORK_LIBRARY_DIRS})
#message("rw libraries" ${ROBWORK_LIBRARIES})
#message("rw libs" ${ROBWORK_LIBS})

# mband: link_directories is required until RobWork issue ## is solved
######link_directories(${ROBWORK_LIBRARY_DIRS})

#add_executable(caros_test src/Caros_Test.cpp)
#target_link_libraries(caros_test ${conv_library} ${ROBWORK_LIBRARIES})


set(SRCFILES 
    src/UniversalRobots.cpp
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()



#common commands for building c++ executables and libraries
#set(ROS_BUILD_STATIC_LIBS true)
#set(ROS_BUILD_SHARED_LIBS false)
#MESSAGE(" UniversalRobots_LIBRAIES : ${catkin_LIBRARIES}")
add_library(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME}  
		      ${catkin_LIBRARIES}
		      ${COVIS_LIBRARIES}
		      ${ROBWORK_LIBRARIES}
		      ${ROBWORKHARDWARE_LIBRARIES}
		      ${caros_common_LIBRARIES}
		      ${caros_control_LIBRARIES}
)

add_executable(ur src/ur_main.cpp )
target_link_libraries(ur 
		      ${PROJECT_NAME} 
		      ${catkin_LIBRARIES}
		      ${COVIS_LIBRARIES}
		      ${ROBWORK_LIBRARIES}
		      ${ROBWORKHARDWARE_LIBRARIES}
		      ${caros_common_LIBRARIES}
		      ${caros_control_LIBRARIES}
)

add_dependencies(ur
  caros_common
  caros_control
)
  #rosbuild_add_executable(ursafe src/ursafe_main.cpp )
  #TARGET_LINK_LIBRARIES(ursafe ${PROJECT_NAME} ${ROBWORKHARDWARE_LIBRARIES} ${ROBWORK_LIBRARIES}  )

  #rosbuild_add_executable(ursim src/ursim_main.cpp )	
#TARGET_LINK_LIBRARIES(ursim ${PROJECT_NAME} ${ROBWORK_LIBRARIES})
