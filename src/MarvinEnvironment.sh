#! /env/bash
#********************************************************************************************************************
# *   MARVIN Bash RC Configuration file 
# *
# * $Revision: 4263 $
# * $Date: 2013-12-03 15:39:45 +0100 (Tue, 03 Dec 2013) $
# * $Author: arf $
# * $Id: MarvinEnvironment.sh 4263 2013-12-03 14:39:45Z arf $
# *
# *******************************************************************************************************************
#********************* NB ***********************
# Only commit persistent changes to this file!
#************************************************

#*** Marvin Configuration ***
# DTI Paths
export MARVIN_ROOT=~/catkin_ws/MARVIN
#export DTI_COMMON_ROOT=~/dti_co_worker/trunk/common

#*** ROS Configuration ***
# ROS Paths
#INDIGO_SETUP=/opt/ros/indigo/setup.bash
HYDRO_SETUP=/opt/ros/hydro/setup.bash
MARVIN_SETUP=$MARVIN_ROOT/devel/setup.sh

#if [ -f $INDIGO_SETUP ]; then
#  source $INDIGO_SETUP
#else
#  echo "Warning: Unable to source '$INDIGO_SETUP'"
#fi

if [ -f $HYDRO_SETUP ]; then
  source $HYDRO_SETUP
else
  echo "Warning: Unable to source '$HYDRO_SETUP'"
fi

if [ -f $MARVIN_SETUP ]; then
  source $MARVIN_SETUP
else
  echo "Warning: Unable to source '$MARVIN_SETUP'"
fi

ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$MARVIN_ROOT

#ROS URI to find the Master server
#export MASTER_IP=tek-marvin-1.sandbox.tek.sdu.dk
export MASTER_IP=thso
export ROS_MASTER_URI=http://$MASTER_IP:11311

export RobWork_DIR=~/libs/RobWork
export RW_ROOT=$RobWork_DIR/RobWork/
export RWS_ROOT=$RobWork_DIR/RobWorkStudio/
export RWHW_ROOT=$RobWork_DIR/RobWorkHardware/
export RWSIM_ROOT=$RobWork_DIR/RobWorkSim/
export RobWorkSim_DIR=$RWSIM_ROOT

export CoViS_DIR=~/libs/CoViS/build/lib/cmake/CoViS

#export RobWork_DIR=~/dti_co_worker/trunk/common/RobWork/RW_new/RobWork
#export RobWorkHardware_DIR=$RobWork_DIR/RobWorkHardware



