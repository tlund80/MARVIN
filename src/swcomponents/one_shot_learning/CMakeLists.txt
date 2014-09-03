cmake_minimum_required(VERSION 2.8.3)
project(one_shot_learning)
SET(CMAKE_BUILD_TYPE debug)
########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS roscpp caros_common_msgs caros_common pcl_ros pose_estimation_covis sensor_msgs)

#find_package(OpenCV REQUIRED)

find_package(
  CoViS
  REQUIRED
  #HINTS $ENV{DTI_COMMON_ROOT}/CoViS
)
LINK_DIRECTORIES(${COVIS_LIBRARY_DIRS})

########################################################################
#### Find PCL
########################################################################
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

########################################################################
#### Find QT
########################################################################
find_package(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

########################################################################
#### Find RobWork
########################################################################
SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")
SET(RWSIM_ROOT "$ENV{RWSIM_ROOT}")

FIND_PACKAGE(
  RobWork
  QUIET
  HINTS
    "${RW_ROOT}")
IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork with the new method. Trying the old one.")
  INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
ENDIF(NOT ROBWORK_FOUND)

FIND_PACKAGE(
  RobWorkStudio
  QUIET
  HINTS
    "${RWSTUDIO_ROOT}")
IF(NOT ROBWORKSTUDIO_FOUND)
  MESSAGE(STATUS "Could not find RobWorkStudio with the new method. Trying the old one.")
  INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
ENDIF(NOT ROBWORKSTUDIO_FOUND)

FIND_PACKAGE(
  RobWorkSim
  QUIET
  HINTS
    "${RWSIM_ROOT}")
IF(NOT ROBWORKSIM_FOUND)
  MESSAGE(STATUS "Could not find RobWorkSim with the new method. Trying the old one.")
  INCLUDE(${RWSIM_ROOT}/build/FindRobWorkSim.cmake)
ENDIF(NOT ROBWORKSIM_FOUND)

message("rw link dir: " ${ROBWORK_LIBRARY_DIRS})
message("rw libraries: " ${ROBWORK_LIBRARIES})
message("rw libs: " ${ROBWORK_LIBS})

message("rw link dir: " ${ROBWORKSTUDIO_LIBRARY_DIRS})
message("rws libraries: " ${ROBWORKSTUDIO_LIBRARIES})

link_directories(${ROBWORK_LIBRARY_DIRS})
link_directories(${ROBWORKSTUDIO_LIBRARY_DIRS})
link_directories(${ROBWORKSIM_LIBRARY_DIRS})

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

find_package(Boost REQUIRED COMPONENTS thread signals system)
#find_package(
#  CoViS
#  REQUIRED
  #HINTS $ENV{DTI_COMMON_ROOT}/CoViS
#)
#LINK_DIRECTORIES(${COVIS_LIBRARY_DIRS})
################################################
## Declare ROS messages, services and actions ##
################################################
# add_service_files(
#   FILES
#    estimate.srv
#    prepareEstimation.srv
# )

# add_message_files(
#   FILES
# #  RobotState.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   geometry_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES ${library_name}
  # CATKIN_DEPENDS:
  # - rosconsole is not used in the header files, so should not be added to the list here.
  CATKIN_DEPENDS roscpp sensor_msgs
  #DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
include_directories(
  include
 # ${COVIS_INCLUDE_DIR}
  ${ROBWORK_INCLUDE_DIRS}
  ${ROBWORKSTUDIO_INCLUDE_DIRS}
  ${ROBWORKSIM_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/one_shot_learning/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_library(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${BOOST_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROBWORKSIM_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} QVTK)
        