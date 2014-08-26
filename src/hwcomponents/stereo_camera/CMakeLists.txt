cmake_minimum_required(VERSION 2.8.3)
project(stereo_camera)

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS 
	     message_generation 
	     roscpp
	     rospy 
	     nodelet
	     image_transport
	     dynamic_reconfigure
	     sensor_msgs
	     pcl_ros
	     tf
	     cv_bridge
	     caros_common
	     std_msgs)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io)
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
# add_service_files(
#   FILES
#    FrameCount.srv
#)

# add_message_files(
#   FILES
# #  RobotState.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES ${library_name}
  # CATKIN_DEPENDS:
  # - rosconsole is not used in the header files, so should not be added to the list here.
  CATKIN_DEPENDS roscpp std_msgs caros_common
  #DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
include_directories(
  #include
  ${COVIS_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
LINK_DIRECTORIES( ${COVIS_LIBRARY_DIRS} )

########################################################################
#### SDH Node
########################################################################

add_library(stereo_device src/nodelets/stereo_camera.cpp)
add_dependencies(stereo_device caros_common_msgs_generate_messages_cpp)
target_link_libraries(stereo_device ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${COVIS_LIBRARIES})

add_library(stereo_preprocess src/nodelets/stereo_preprocess.cpp)
add_dependencies(stereo_preprocess caros_common_msgs_generate_messages_cpp)
target_link_libraries(stereo_preprocess ${catkin_LIBRARIES} ${COVIS_LIBRARIES})

add_library(stereo_crop src/nodelets/stereo_crop.cpp)
add_dependencies(stereo_crop caros_common_msgs_generate_messages_cpp)
target_link_libraries(stereo_crop ${catkin_LIBRARIES} ${COVIS_LIBRARIES})

add_library(stereo_downsample src/nodelets/stereo_downsample.cpp)
add_dependencies(stereo_downsample caros_common_msgs_generate_messages_cpp)
target_link_libraries(stereo_downsample ${catkin_LIBRARIES} ${COVIS_LIBRARIES})

add_library(stereo_point_cloud src/nodelets/stereo_point_cloud.cpp)
add_dependencies(stereo_point_cloud  caros_common_msgs_generate_messages_cpp)
target_link_libraries(stereo_downsample ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# executables
add_executable(stereo_camera src/nodes/stereo_camera_node.cpp)
add_dependencies(stereo_camera caros_common_msgs_generate_messages_cpp)
target_link_libraries(stereo_camera ${catkin_LIBRARIES} ${COVIS_LIBRARIES})

add_executable(stereo_to_points src/stereo_to_points.cpp)
add_dependencies(stereo_to_points caros_common_msgs_generate_messages_cpp)
target_link_libraries(stereo_to_points ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})



