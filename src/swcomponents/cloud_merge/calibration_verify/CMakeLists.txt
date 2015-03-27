cmake_minimum_required(VERSION 2.8.3)
project(calibration_verify)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  rospy
  sensor_msgs
  image_transport
  cv_bridge
  caros_common
)
########################################################################
#### Find QT
########################################################################
find_package(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
########################################################################
#### OpenCV
########################################################################
find_package(OpenCV REQUIRED)
########################################################################
#### CoViS
########################################################################
find_package(
  CoViS
  REQUIRED
  #HINTS $ENV{DTI_COMMON_ROOT}/CoViS
)
LINK_DIRECTORIES(${COVIS_LIBRARY_DIRS})
if(NOT CoViS_FOUND)
  message(FATAL_ERROR "CoViS NOT found! You should append a proper hint path for the CoViS directory tree inside pose_estimation_new/CMakeLists.txt")
endif()
########################################################################
#### RobWork
########################################################################
SET(CMAKE_BUILD_TYPE Release)
SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")
SET(RWHW_ROOT "$ENV{RWHW_ROOT}")

#Include default settings for constructing a robwork dependent project
#INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
#FIND_PACKAGE(RobWork QUIET HINTS "${RW_ROOT}")
FIND_PACKAGE(
  RobWork
  QUIET
  HINTS
    "${RW_ROOT}")
#Include default settings for constructing a robworkstudio dependent project
#INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
#FIND_PACKAGE(RobWorkStudio QUIET HINTS "${RWSTUDIO_ROOT}")
FIND_PACKAGE(
  RobWorkStudio
  QUIET
  HINTS
    "${RWSTUDIO_ROOT}")
#For robworkhardware
#SET(CMAKE_MODULE_PATH ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})
#FIND_PACKAGE(
#	RobWorkHardware 
#	REQUIRED 	
#	QUIET)

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
# add_message_files(
#   FILES
# )

 add_service_files(
   FILES
   kinect_chessboard_detector.srv
   stereo_chessboard_detector.srv
 )

 generate_messages(
   DEPENDENCIES
sensor_msgs
 )

########################################################################
#### Catkin Specific Configuration
########################################################################
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS message_runtime roscpp
  DEPENDS RobWork CoViS caros_common
)

include_directories(
  #include
  ${COVIS_INCLUDE_DIR}
  ${ROBWORK_INCLUDE_DIRS}
  ${ROBWORKSTUDIO_INCLUDE_DIRS}
 # ${ROBWORKHARDWARE_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS}
                  ${ROBWORKSTUDIO_LIBRARY_DIRS}
                  #${ROBWORKHARDWARE_LIBRARY_DIRS}
                  )

#From here we add the plugins
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/calibration_verify.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/calibration_verify.cpp)

add_library(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} 
		      ${ROBWORK_LIBRARIES}
		      ${ROBWORKSTUDIO_LIBRARIES}
                   #   ${ROBWORKHARDWARE_LIBRARIES}
                      ${catkin_LIBRARIES} 
		      ${OpenCV_LIBRARIES}
                      ${COVIS_LIBRARIES}
                      rw_sandbox)
                      
add_executable(stereo_chessboard_detector src/stereo_chessboard_detector.cpp)
add_dependencies(stereo_chessboard_detector stereo_chessboard_detector_generate_messages_cpp)
target_link_libraries(stereo_chessboard_detector ${OpenCV_LIBS} ${COVIS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(kinect_chessboard_detector src/kinect_chessboard_detector.cpp)
add_dependencies(kinect_chessboard_detector kinect_chessboard_detector_generate_messages_cpp)
target_link_libraries(kinect_chessboard_detector ${OpenCV_LIBS} ${COVIS_LIBRARIES} ${catkin_LIBRARIES})


