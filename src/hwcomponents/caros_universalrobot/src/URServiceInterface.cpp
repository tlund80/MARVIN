/**/
#include "URServiceInterface.hpp"

#include "marvin_common/RobotState.h"
#include <marvin_common_rw/RwRos.hpp>

#include <rw/math.hpp>
#include <boost/foreach.hpp>


using namespace marvin_common;

using namespace rw::common;
using namespace rw::math;


URServiceInterface::URServiceInterface(const std::string& service_name, double loopRate, bool supportSafeMotions):
	_service_name(service_name),
	_loopRate(loopRate),
	_nodeHnd(service_name)
{
    //_sensor_data_publisher = _nodeHnd.advertise<sensor_data>("sensor_data", 1000);

    _deviceStatePublisher = _nodeHnd.advertise<RobotState>("RobotState", 1000);
   // _wrenchDataPublisher = _nodeHnd.advertise<WrenchData>("URWrenchData", 1000);

    _srvMoveQ = _nodeHnd.advertiseService("moveQ", &URServiceInterface::moveQHandle, this);
    _srvMoveL = _nodeHnd.advertiseService("moveL", &URServiceInterface::moveLHandle, this);
    _srvServo = _nodeHnd.advertiseService("servo", &URServiceInterface::servoHandle, this);
    _srvServoQ = _nodeHnd.advertiseService("servoq", &URServiceInterface::servoQHandle, this);
    _srvStop = _nodeHnd.advertiseService("stop", &URServiceInterface::stopHandle, this);

    if (supportSafeMotions) {
    	_srvSafeMoveQ = _nodeHnd.advertiseService("safeMoveQ", &URServiceInterface::safeMoveQHandle, this);
    	_srvSafeMoveL = _nodeHnd.advertiseService("safeMoveL", &URServiceInterface::safeMoveLHandle, this);
    }


 //   _marvinTime.initialize(_nodeHnd);
}


bool URServiceInterface::safeMoveLHandle(URMoveL::Request& request, URMoveL::Response& response) {

	std::vector<Transform3D<> > targets;
	BOOST_FOREACH(geometry_msgs::Transform& target, request.path) {
		targets.push_back(RwRos::toRw(target));
	}

	std::vector<float> blends;
	BOOST_FOREACH(float f, request.blends) {
		blends.push_back(f);
	}

	return safeMoveL(targets, blends, request.speed);

}


bool URServiceInterface::moveLHandle(URMoveL::Request& request, URMoveL::Response& response) {

	std::vector<Transform3D<> > targets;
	BOOST_FOREACH(geometry_msgs::Transform& target, request.path) {
		targets.push_back(RwRos::toRw(target));
	}

	std::vector<float> blends;
	BOOST_FOREACH(float f, request.blends) {
		blends.push_back(f);
	}

	return moveL(targets, blends, request.speed);
}

bool URServiceInterface::safeMoveQHandle(URMoveQ::Request& request, URMoveQ::Response& response)
{

	std::vector<Q> targets;
	BOOST_FOREACH(marvin_common::Q6 qs, request.path) {
		targets.push_back(RwRos::toRw(qs));
	}
	std::vector<float> blends;
	BOOST_FOREACH(float f, request.blends) {
		blends.push_back(f);
	}

	return safeMoveQ(targets, blends, request.speed);
}


bool URServiceInterface::moveQHandle(URMoveQ::Request& request, URMoveQ::Response& response)
{

	std::vector<Q> targets;
	BOOST_FOREACH(marvin_common::Q6 qs, request.path) {
		targets.push_back(RwRos::toRw(qs));
	}
	std::vector<float> blends;
	BOOST_FOREACH(float f, request.blends) {
		blends.push_back(f);
	}

	return moveQ(targets, blends, request.speed);
}


bool URServiceInterface::servoHandle(URServo::Request& request, URServo::Response& response) {
//	ROS_INFO("URServiceInterface::servoHandle");
    std::cout<<"Delay = "<<ros::Time::now() - request.stamp<<std::endl;
	Transform3D<> target = RwRos::toRw(request.target);
	VelocityScrew6D<> vel = RwRos::toRw(request.velocity);

	//std::cout<<"URServiceInterface::servoHandle:"<<target<<" and "<<vel<<std::endl;
	return servoT(target, vel);
}

bool URServiceInterface::servoQHandle(URServoQ::Request& request, URServoQ::Response& response) {
	std::cout<<"URServiceInterface::servoQHandle"<<std::endl;
	Q qtarget = RwRos::toRw(request.qtarget);

	//std::cout<<"URServiceInterface::servoHandle:"<<target<<" and "<<vel<<std::endl;
	return servoQ(qtarget);

}


bool URServiceInterface::stopHandle(URStop::Request& request, URStop::Response& response)
{
//	ROS_INFO("URServiceInterface::stopHandle");
	stop();
	return true;
}

bool URServiceInterface::pauseHandle() {
	return true;
}

bool URServiceInterface::startHandle() {
	return true;
}

bool URServiceInterface::waitHandle() {
	return true;
}


void URServiceInterface::publish(const RobotState& state) {
	_deviceStatePublisher.publish(state);
}

//void URServiceInterface::publish(const WrenchData& state) {
	//_wrenchDataPublisher.publish(state);
//}

void URServiceInterface::publish(const rw::math::Q& q) {


    std::stringstream ss;
    ss <<_service_name<<" "<<q;

    RobotState state;
    state.header.frame_id = _service_name;
	state.q = RwRos::toRosQ6(q);
    state.header.stamp = ros::Time::now();

    publish(state);


}

bool URServiceInterface::run() {

	  while (ros::ok())
	  {
		  loop();
		  ros::spinOnce();
		  _loopRate.sleep();
	  }
	  stopDriver();
	  return true;
}

double URServiceInterface::getLoopRate() {
	return _loopRate.expectedCycleTime ().toSec();
}
