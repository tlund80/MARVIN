cmake_minimum_required(VERSION 2.8.3)
project(caros_control_msgs)

########################################################################
#### Local Variables
########################################################################

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  SerialDeviceForceControlStart.srv
  SerialDeviceForceControlStop.srv
  SerialDeviceForceControlUpdate.srv
  SerialDeviceMoveLin.srv
  SerialDeviceMoveLinFC.srv
  SerialDeviceMovePTP_T.srv
  SerialDeviceMovePTP.srv
  SerialDeviceMoveVelQ.srv
  SerialDeviceMoveVelT.srv
  GripperMoveQ.srv
  GripperGripQ.srv
  GripperSetForceQ.srv
  GripperSetVelocityQ.srv
  GripperStopMovement.srv
)

add_message_files(
  FILES
  RobotState.msg
  GripperState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  caros_common_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs caros_common_msgs
)
