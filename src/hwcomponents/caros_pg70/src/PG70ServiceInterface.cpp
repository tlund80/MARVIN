/**/
#include <caros/PG70ServiceInterface.hpp>

//#include "marvin_common/Device.h"
//#include <marvin_common_rw/RwRos.hpp>

#include <rw/math.hpp>
#include <boost/foreach.hpp>


//using namespace marvin_common;
//using namespace schunkpg70;

using namespace rw::common;
using namespace rw::math;


PG70ServiceInterface::PG70ServiceInterface(const std::string& service_name, double loopRate):
	_service_name(service_name),
	_loopRate(loopRate)
{
    //_sensor_data_publisher = _nodeHnd.advertise<sensor_data>("sensor_data", 1000);

/*    _statePublisher = _nodeHnd.advertise<ParallelGripperState>(_service_name+"/ParallelGripperState", 5);

    _srvMove = _nodeHnd.advertiseService(_service_name+"/move", &PG70ServiceInterface::moveHandle, this);
    _srvOpen = _nodeHnd.advertiseService(_service_name+"/open", &PG70ServiceInterface::openHandle, this);
    _srvClose = _nodeHnd.advertiseService(_service_name+"/close", &PG70ServiceInterface::closeHandle, this);
    _srvHome = _nodeHnd.advertiseService(_service_name+"/home", &PG70ServiceInterface::homeHandle, this);
    _srvStop = _nodeHnd.advertiseService(_service_name+"/stop", &PG70ServiceInterface::stopHandle, this);
    _marvinTime.initialize(_nodeHnd);
*/
}
/*
bool PG70ServiceInterface::moveHandle(Move::Request& request, Move::Response& response) {
	ROS_INFO("Move %f", request.pos);
	return move(request.pos);
}

bool PG70ServiceInterface::openHandle(schunkpg70::Open::Request& request, schunkpg70::Open::Response& response) {
	ROS_INFO("Open %f", request.power);
	return open(request.power);
}

bool PG70ServiceInterface::closeHandle(schunkpg70::Close::Request& request, schunkpg70::Close::Response& response) {
	ROS_INFO("Close %f", request.power);
	return close(request.power);
}

bool PG70ServiceInterface::homeHandle(schunkpg70::Home::Request& request, schunkpg70::Home::Response& response) {
	ROS_INFO("home");
	return home();
}

bool PG70ServiceInterface::stopHandle(schunkpg70::Stop::Request& request, schunkpg70::Stop::Response& response) {
	ROS_INFO("stop");
	return stop();
}

void PG70ServiceInterface::publish(double q) {
	ParallelGripperState state;
	state.header.frame_id = _service_name;
	state.header.stamp = _marvinTime.getMarvinTime();
	state.pos = q;
	publish(state);
}

void PG70ServiceInterface::publish(const ParallelGripperState& state) {
	_statePublisher.publish(state);
}

bool PG70ServiceInterface::run() {

	  while (ros::ok())
	  {
		  loop();
		  ros::spinOnce();
		  _loopRate.sleep();
	  }
	  stopDriver();
	  return true;
}

double PG70ServiceInterface::getLoopRate() {
	return _loopRate.cycleTime().toSec();
}
*/