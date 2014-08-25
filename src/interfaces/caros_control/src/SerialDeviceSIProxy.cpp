/**/
#include <marvin_common_rw/SerialDeviceSIProxy.hpp>

#include <fstream>

#include <marvin_common/URServoQ.h>
#include <marvin_common/URServo.h>
#include <marvin_common_rw/SerialDeviceServiceInterface.hpp>
#include <marvin_common_rw/RwRos.hpp>


#include <rw/common/Ptr.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace std;

SerialDeviceSIProxy::SerialDeviceSIProxy(ros::NodeHandle nhandle, const std::string& devnameTmp):
		_nodeHnd(nhandle)
{
	std::string devname = devnameTmp;
	if(devname[0]!='/')
		devname = "/"+devnameTmp;
	// set up everything to control the robot
	// services
<<<<<<< .mine
	_srvStart = _nodeHnd.serviceClient<std_srvs::Empty> (devname + "/start");
	_srvStop = _nodeHnd.serviceClient<marvin_common::Stop> (devname + "/stop");
	_srvPause = _nodeHnd.serviceClient<marvin_common::Pause> (devname + "/pause");
=======
    bool PersistentConnection = true;
    _srvStart = _nodeHnd->serviceClient<std_srvs::Empty> (devname + "/start", PersistentConnection);
    _srvStop = _nodeHnd->serviceClient<marvin_common::Stop> (devname + "/stop", PersistentConnection);
    _srvPause = _nodeHnd->serviceClient<marvin_common::Pause> (devname + "/pause", PersistentConnection);
>>>>>>> .r1650

<<<<<<< .mine
	_srvMovePTP = _nodeHnd.serviceClient<marvin_common::SerialDeviceMovePTP> (devname + "/movePTP");
	_srvMovePTP_T = _nodeHnd.serviceClient<marvin_common::SerialDeviceMovePTP_T> (devname + "/movePTP_T");
	_srvMoveLin = _nodeHnd.serviceClient<marvin_common::SerialDeviceMoveLin> (devname + "/moveLin");
	_srvMoveLinFC = _nodeHnd.serviceClient<marvin_common::SerialDeviceMoveLinFC> (devname + "/moveLin_FC");
	_srvMoveVelQ = _nodeHnd.serviceClient<marvin_common::SerialDeviceMoveVelQ> (devname + "/moveVelQ");
	_srvMoveVelT = _nodeHnd.serviceClient<marvin_common::SerialDeviceMoveVelT> (devname + "/moveVelT");
=======
    _srvMovePTP = _nodeHnd->serviceClient<marvin_common::SerialDeviceMovePTP> (devname + "/movePTP", PersistentConnection);
    _srvMovePTP_T = _nodeHnd->serviceClient<marvin_common::SerialDeviceMovePTP_T> (devname + "/movePTP_T", PersistentConnection);
    _srvMoveLin = _nodeHnd->serviceClient<marvin_common::SerialDeviceMoveLin> (devname + "/moveLin", PersistentConnection);
    _srvMoveLinFC = _nodeHnd->serviceClient<marvin_common::SerialDeviceMoveLinFC> (devname + "/moveLin_FC", PersistentConnection);
    _srvMoveVelQ = _nodeHnd->serviceClient<marvin_common::SerialDeviceMoveVelQ> (devname + "/moveVelQ", PersistentConnection);
    _srvMoveVelT = _nodeHnd->serviceClient<marvin_common::SerialDeviceMoveVelT> (devname + "/moveVelT", PersistentConnection);
>>>>>>> .r1650

<<<<<<< .mine
	_srvServoQ = _nodeHnd.serviceClient<marvin_common::SerialDeviceMovePTP> (devname + "/servoQ");
	_srvServoT = _nodeHnd.serviceClient<marvin_common::SerialDeviceMovePTP_T> (devname + "/servoT");
=======
    _srvServoQ = _nodeHnd->serviceClient<marvin_common::SerialDeviceMovePTP> (devname + "/servoQ", PersistentConnection);
    _srvServoT = _nodeHnd->serviceClient<marvin_common::SerialDeviceMovePTP_T> (devname + "/servoT", PersistentConnection);
>>>>>>> .r1650


	// states
	_robotState = _nodeHnd.subscribe(devname + "/RobotState", 1, &SerialDeviceSIProxy::handleRobotState, this);

}

SerialDeviceSIProxy::~SerialDeviceSIProxy() {
}


//! @brief move robot in a linear Cartesian path
bool SerialDeviceSIProxy::moveLin(const rw::math::Transform3D<>& target, float speed, float blend)
{
	marvin_common::SerialDeviceMoveLin srv;
	srv.request.targets.push_back( RwRos::toRos(target) );
	srv.request.speeds.push_back( speed );
	srv.request.blends.push_back( blend );
	return _srvMovePTP_T.call(srv);
}

//! @brief move robot from point to point
bool SerialDeviceSIProxy::movePTP(const rw::math::Q& target, float speed, float blend)
{
	marvin_common::SerialDeviceMovePTP srv;
	srv.request.targets.push_back( RwRos::toRos(target) );
	srv.request.speeds.push_back( speed );
	srv.request.blends.push_back( blend );
	return _srvMovePTP.call(srv);
}


//! @brief move robot from point to point but using a pose as target (require invkin)
bool SerialDeviceSIProxy::movePTP_T(const rw::math::Transform3D<>& target, float speed, float blend)
{
	marvin_common::SerialDeviceMovePTP_T srv;
	srv.request.targets.push_back( RwRos::toRos(target) );
	srv.request.speeds.push_back( speed );
	srv.request.blends.push_back( blend );
	return _srvMovePTP_T.call(srv);
}

//! @brief move robot from point to point
bool SerialDeviceSIProxy::servoQ(const rw::math::Q& target, float speed)
{
	marvin_common::SerialDeviceMovePTP srv;
	srv.request.targets.push_back( RwRos::toRos(target) );
	srv.request.speeds.push_back( speed );
	return _srvServoQ.call(srv);
}


//! @brief move robot from point to point but using a pose as target (require invkin)
bool SerialDeviceSIProxy::servoT(const rw::math::Transform3D<>& target, float speed)
{
	marvin_common::SerialDeviceMovePTP_T srv;
	srv.request.targets.push_back( RwRos::toRos(target) );
	srv.request.speeds.push_back( speed );
	return _srvServoT.call(srv);
}


//! @brief move robot in a servoing fasion
bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
	marvin_common::SerialDeviceMoveVelQ srv;
	srv.request.q_vel =  RwRos::toRos(target) ;
	return _srvMoveVelQ.call(srv);
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
	marvin_common::SerialDeviceMoveVelT srv;
	srv.request.vel =  RwRos::toRos(target) ;
	return _srvMoveVelT.call(srv);
}

//! move robot with a hybrid position/force control
bool SerialDeviceSIProxy::moveLinFC(const rw::math::Transform3D<>& target,
						  rw::math::Wrench6D<>& wtarget,
						  float selection[6],
						  std::string refframe,
						  rw::math::Transform3D<> offset,
						  float speed,
						  float blend)
{
	//! TODO: need implementing
	ROS_ERROR("NOT IMPLEMENTED!");
	return false;
}

//! hard stop the robot,
bool SerialDeviceSIProxy::stop(){
	marvin_common::Stop srv;
	return _srvStop.call(srv);
}

//! pause the robot, should be able to continue trajectory
bool SerialDeviceSIProxy::pause(){
	marvin_common::Pause srv;
	return _srvPause.call(srv);
}

//! enable safe mode, so that robot stops when collisions are detected
bool SerialDeviceSIProxy::setSafeModeEnabled(bool enable){
	marvin_common::ConfigBool srv;
	srv.request.value = enable;
	return _srvStop.call(srv);
}

void SerialDeviceSIProxy::handleRobotState(const marvin_common::RobotState& state) {
	boost::mutex::scoped_lock lock(_mutex);
	_q = RwRos::toRw(state.q);
	_dq = RwRos::toRw(state.dq);
	_pRobotState = state;
}


rw::math::Q SerialDeviceSIProxy::getQ() {
	boost::mutex::scoped_lock lock(_mutex);
	return _q;
}

rw::math::Q SerialDeviceSIProxy::getQd() {
	boost::mutex::scoped_lock lock(_mutex);
	return _dq;
}

bool SerialDeviceSIProxy::isMoving() {
	boost::mutex::scoped_lock lock(_mutex);
	return _pRobotState.isMoving;
}

ros::Time SerialDeviceSIProxy::getTimeStamp() {
	boost::mutex::scoped_lock lock(_mutex);
	return _pRobotState.header.stamp;
}

