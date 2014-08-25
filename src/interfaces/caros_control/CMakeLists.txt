cmake_minimum_required(VERSION 2.8.3)
project(caros_control)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp geometry_msgs caros_common_msgs caros_common caros_control_msgs)

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

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
# add_service_files(
#   FILES
# )

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
  CATKIN_DEPENDS message_runtime std_msgs roscpp geometry_msgs caros_common_msgs caros_common caros_control_msgs
  DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
include_directories(
  include
  ${COVIS_INCLUDE_DIR}
  ${ROBWORK_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Create one library for the whole package (don't split it into gripper, serialDevice, etc. - if this is wanted then this package should rather be split into gripper, serialDevice and similar packages) 
add_library(${library_name}
  src/GripperServiceInterface.cpp
  src/SerialDeviceServiceInterface.cpp
  src/GripperSIProxy.cpp
)

add_dependencies(${library_name}
# ${catkin_EXPORTED_TARGETS} should be used if this target depends on things (e.g. message or service headers) that are imported from other catkin packages. (see http://docs.ros.org/hydro/api/catkin/html/howto/building_msgs.html) - The dependency targets can also be specified manually as is done here for the caros_common_msgs and caros_control_msgs dependencies.
  caros_common_msgs_generate_messages_cpp
  caros_control_msgs_generate_messages_cpp
)

target_link_libraries(${library_name}
  ${catkin_LIBRARIES}
  ${COVIS_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)
