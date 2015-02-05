cmake_minimum_required(VERSION 2.8.3)
project(caros_common)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  caros_common_msgs
  image_geometry
)

########################################################################
#### CoViS
########################################################################
find_package(
  CoViS
  REQUIRED
  #HINTS $ENV{DTI_COMMON_ROOT}/CoViS
)
LINK_DIRECTORIES(${COVIS_LIBRARY_DIRS})
########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
# add_message_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
# )

########################################################################
#### Catkin Specific Configuration
########################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS message_runtime std_msgs roscpp caros_common_msgs image_geometry
  DEPENDS RobWork CoViS 
)

include_directories(
  include
  ${COVIS_INCLUDE_DIR}
  ${ROBWORK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${library_name} STATIC
  src/common.cpp
  src/CarosNodeServiceInterface.cpp
  src/CovisRos.cpp
)

target_link_libraries(${library_name}
  ${ROBWORK_LIBRARIES}
  ${COVIS_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(${library_name}
  caros_common_msgs_generate_messages_cpp
)
