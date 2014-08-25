cmake_minimum_required(VERSION 2.8.3)
project(kinect_vibrator)

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################
 add_service_files(
   FILES
    FrameCount.srv
    Reset.srv
    Start.srv
    Stop.srv
 )

# add_message_files(
#   FILES
# #  RobotState.msg
# )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES ${library_name}
  # CATKIN_DEPENDS:
  # - rosconsole is not used in the header files, so should not be added to the list here.
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  #DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

set(SRCFILES 
    src/KinectVibrator.cpp
    #src/CameraTriggerTest.cpp
   # src/kinectVibratorServerInterfaceTest.cpp
)

########################################################################
#### SDH Node
########################################################################
add_executable(kinectVibratorTest  src/KinectVibratorTest.cpp ${SRCFILES})
add_dependencies(kinectVibratorTest kinectVibratorTest_generate_messages_cpp)
target_link_libraries(kinectVibratorTest ${catkin_LIBRARIES})

add_executable(kinectVibratorServiceInterfaceTest  src/kinectVibratorServerInterfaceTest.cpp ${SRCFILES})
add_dependencies(kinectVibratorServiceInterfaceTest kinectVibratorServiceInterfaceTest_generate_messages_cpp)
target_link_libraries(kinectVibratorServiceInterfaceTest ${catkin_LIBRARIES})

## Declare a cpp library
add_library(${PROJECT_NAME}  ${SRCFILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

