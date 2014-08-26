cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation_covis)

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation roscpp tf point_cloud_visualization caros_common_msgs pcl_ros sensor_msgs geometry_msgs)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(
  CoViS
  REQUIRED
  #HINTS $ENV{DTI_COMMON_ROOT}/CoViS
)
LINK_DIRECTORIES(${COVIS_LIBRARY_DIRS})
################################################
## Declare ROS messages, services and actions ##
################################################
 add_service_files(
   FILES
    estimate.srv
    prepareEstimation.srv
 )

# add_message_files(
#   FILES
# #  RobotState.msg
# )

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES ${library_name}
  # CATKIN_DEPENDS:
  # - rosconsole is not used in the header files, so should not be added to the list here.
  CATKIN_DEPENDS message_runtime roscpp 
  #DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
include_directories(
  include
  ${COVIS_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



########################################################################
#### SDH Node
########################################################################
add_executable(${PROJECT_NAME} src/pose_estimation_CoViS.cpp src/CADModel.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${COVIS_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_pose_estimation_covis test_node/test_node.cpp)
target_link_libraries(test_pose_estimation_covis ${catkin_LIBRARIES} ${PCL_LIBRARIES})

