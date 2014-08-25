cmake_minimum_required(VERSION 2.8.3)
project(openni_launch_marvin)

########################################################################
#### Local Variables
########################################################################
set(library_name ${PROJECT_NAME})

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole pcl_ros)

# Using find_package module version due to finding a FindRobWorkHardware cmake file and the CMAKE_MODULE_PATH above.
# Need to specify CMAKE_BUILD_TYPE=Release to be able to find the proper RobWorkHardware cmake files
find_package(Boost REQUIRED COMPONENTS signals)

################################################
## Declare ROS messages, services and actions ##
################################################
# add_service_files(
#   FILES
#   # SerialDeviceForceControlStart.srv
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
 # INCLUDE_DIRS include
#  LIBRARIES ${library_name}
  # CATKIN_DEPENDS:
  # - rosconsole is not used in the header files, so should not be added to the list here.
  CATKIN_DEPENDS roscpp
  #DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
include_directories(
  #include
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

# add_library(${library_name}
#   src/SDH.cpp
#   src/SDHSim.cpp
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
add_executable(broadcaster src/broadcaster.cpp)
target_link_libraries(broadcaster ${catkin_LIBRARIES} ${Boost_LIBRARIES})	
# Add dependencies on the caros_common and caros_control targets (those are the make targets for producing the libraries that are being linked with)
#add_dependencies(broadcaster
#  caros_common
#  caros_control
#)


