/**/
#include <caros/PoseSensorServiceInterface.hpp>
#include <caros_sensor_msgs/PoseSensorState.h>

#include <caros/common.hpp>

using namespace rw::common;

PoseSensorServiceInterface::PoseSensorServiceInterface(const std::string& service_name):
        _nodeHnd( ownedPtr( new ros::NodeHandle(service_name) ) )
{
}

PoseSensorServiceInterface::PoseSensorServiceInterface(rw::common::Ptr<ros::NodeHandle> nodeHnd):
		_nodeHnd(nodeHnd)
{
}

bool PoseSensorServiceInterface::configurePoseSensor(){
	_posePublisher = _nodeHnd->advertise<caros_sensor_msgs::PoseSensorState>("poses", 5);

    //_srvStart = _nodeHnd->advertiseService("start", &PoseSensorServiceInterface::start, this);
    //_srvStop = _nodeHnd->advertiseService("stop", &PoseSensorServiceInterface::stop, this);
    //_srvPause = _nodeHnd->advertiseService("pause", &PoseSensorServiceInterface::pause, this);
	return true;
}

bool PoseSensorServiceInterface::cleanupPoseSensor(){
    _posePublisher.shutdown();
    return true;
}


void PoseSensorServiceInterface::publishPoses(
		   const std::vector<rw::math::Transform3D<> >& poses,
		   const std::vector<int>& ids,
		   const std::vector<float>& qualities)
{
	caros_sensor_msgs::PoseSensorState pstate;
	pstate.poses.resize( poses.size() );
	pstate.ids.resize( poses.size() );
	pstate.qualities.resize( poses.size() );

	for(size_t i=0;i<poses.size();i++){
		pstate.poses[i] = caros::toRos(poses[i]);
		if(ids.size()<=i)
			pstate.ids[i] = i;
		else
			pstate.ids[i] = ids[i];

		if(qualities.size()<=i)
			pstate.qualities[i] = i;
		else
			pstate.qualities[i] = qualities[i];


	}

	_posePublisher.publish( pstate );
}
