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
export DTI_CO_WORKER_ROOT=~/dti_co_worker/trunk/MARVIN
export DTI_COMMON_ROOT=~/dti_co_worker/trunk/common

#*** ROS Configuration ***
# ROS Paths
HYDRO_SETUP=/opt/ros/hydro/setup.bash
COWORKER_SETUP=$DTI_CO_WORKER_ROOT/devel/setup.sh

if [ -f $HYDRO_SETUP ]; then
  source $HYDRO_SETUP
else
  echo "Warning: Unable to source '$HYDRO_SETUP'"
fi

if [ -f $COWORKER_SETUP ]; then
  source $COWORKER_SETUP
else
  echo "Warning: Unable to source '$COWORKER_SETUP'"
fi

#ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DTI_COMMON_ROOT/Stacks

#ROS URI to find the Master server
export ROS_MASTER_URI=http://$MASTER_IP:11311

export RobWork_DIR=~/dti_co_worker/trunk/common/RobWork/RW_new/RobWork
export RW_ROOT=$RobWork_DIR/RobWork/
export RWS_ROOT=$RobWork_DIR/RobWorkStudio/
export RWHW_ROOT=$RobWork_DIR/RobWorkHardware/
export RWSIM_ROOT=$RobWork_DIR/RobWorkSim/

export CoViS_DIR=$DTI_COMMON_ROOT/CoViS/build/lib/cmake/CoViS

#export RobWork_DIR=~/dti_co_worker/trunk/common/RobWork/RW_new/RobWork
#export RobWorkHardware_DIR=$RobWork_DIR/RobWorkHardware



