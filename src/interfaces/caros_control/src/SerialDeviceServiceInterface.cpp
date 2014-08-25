/**/
#include <caros/SerialDeviceServiceInterface.hpp>
#include <caros/common.hpp>

#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>
#include <boost/foreach.hpp>

using namespace caros_control_msgs;
using namespace rw::common;
using namespace rw::math;

SerialDeviceServiceInterface::SerialDeviceServiceInterface(const std::string& service_name)
{
	_nodeHnd = ros::NodeHandle(service_name);
	initNodeHandle();
}

SerialDeviceServiceInterface::SerialDeviceServiceInterface(ros::NodeHandle nodeHnd):
		_nodeHnd(nodeHnd)
{
	initNodeHandle();
}

SerialDeviceServiceInterface::~SerialDeviceServiceInterface(){}

void SerialDeviceServiceInterface::initNodeHandle(){
    _deviceStatePublisher = _nodeHnd.advertise<RobotState>("RobotState", 10);

    _srvMovePTP = _nodeHnd.advertiseService("movePTP", &SerialDeviceServiceInterface::movePTP, this);
    _srvMovePTP_T = _nodeHnd.advertiseService("movePTP_T", &SerialDeviceServiceInterface::movePTP_T, this);
    _srvMoveLin = _nodeHnd.advertiseService("moveLin", &SerialDeviceServiceInterface::moveLin, this);
    _srvMoveLinFC = _nodeHnd.advertiseService("moveLinFC", &SerialDeviceServiceInterface::moveLinFC, this);
    _srvMoveVelQ = _nodeHnd.advertiseService("moveVelQ", &SerialDeviceServiceInterface::moveVelQ, this);
    _srvMoveVelT = _nodeHnd.advertiseService("moveVelT", &SerialDeviceServiceInterface::moveVelT, this);
    _srvServoQ = _nodeHnd.advertiseService("servoQ", &SerialDeviceServiceInterface::servoQ, this);
    _srvServoT = _nodeHnd.advertiseService("servoT", &SerialDeviceServiceInterface::servoT, this);

    _srvStop = _nodeHnd.advertiseService("stop", &SerialDeviceServiceInterface::stop, this);
    _srvPause = _nodeHnd.advertiseService("pause", &SerialDeviceServiceInterface::pause, this);
    _srvStart = _nodeHnd.advertiseService("start", &SerialDeviceServiceInterface::start, this);

    _srvSafe = _nodeHnd.advertiseService("setSafeModeEnabled", &SerialDeviceServiceInterface::setSafeModeEnabled, this);
}

void SerialDeviceServiceInterface::publish(const RobotState& state) {
	_deviceStatePublisher.publish(state);
}
