cmake_minimum_required(VERSION 2.8.3)
project(caros_common_msgs)

########################################################################
#### Local Variables
########################################################################

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
add_message_files(
  FILES
  Q.msg
  CarosNodeState.msg
  ECVContour3D.msg
  ECVContours3D.msg
  ECVExtendedLineSegment3D.msg
  ECVExtendedLineSegments3D.msg
  ECVLineSegment2D.msg
  ECVLineSegment3D.msg
  ECVLineSegments3D.msg
  ECVLink.msg
  ECVLinks.msg
  ECVObjectRelations.msg
  ECVRelationsMatrix.msg
  ECVRelationsVector.msg
  ECVScene.msg
  ECVSceneRelations.msg
  ECVSurface3D.msg
  ECVSurfaces3D.msg
  ECVSurfling3D.msg
  ECVSurflings3D.msg
  ECVTexlet2D.msg
  ECVTexlet3D.msg
  ECVTexlets2D.msg
  ECVTexlets3D.msg
  StereoCalibration.msg
  CameraCalibration.msg
)

add_service_files(
  FILES
  Start.srv
  Stop.srv
  Pause.srv
  ConfigBool.srv
  ConfigInt.srv
  CapturePoints2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

########################################################################
#### Catkin Specific Configuration
########################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
