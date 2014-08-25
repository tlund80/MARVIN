/**/
#ifndef SerialDeviceSIProxy_HPP_
#define SerialDeviceSIProxy_HPP_
#include "marvin_common/RobotState.h"
#include <rw/common/Ptr.hpp>
#include <rw/math.hpp>
#include <rw/trajectory/Path.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include <queue>

namespace caros {


/**
 * @brief this class implements a cpp proxy to control and read data from
 * a SerialDeviceServiceInterface.
 *
 */
class SerialDeviceSIProxy {

public:
	typedef rw::common::Ptr<SerialDeviceSIProxy> Ptr;

	//! constructor - create with device name
	SerialDeviceSIProxy(ros::NodeHandle nhandle, const std::string& devname);

	//! destructor
	virtual ~SerialDeviceSIProxy();

	//! @brief move robot in a linear Cartesian path
	bool moveLin(const rw::math::Transform3D<>& target, float speed=100, float blend=0);

	//! @brief move robot from point to point
	bool movePTP(const rw::math::Q& target, float speed=100, float blend=0);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(const rw::math::Transform3D<>& target, float speed=100, float blend=0);

	//! @brief move robot in a servoing fasion
	virtual bool moveVelQ(const rw::math::Q& target);

	virtual bool moveVelT(const rw::math::VelocityScrew6D<>& target);

	//! @brief move robot in a servoing fasion
	virtual bool servoQ(const rw::math::Q& target, float speed = 100);

	virtual bool servoT(const rw::math::Transform3D<>& target, float speed=100);

	//! move robot with a hybrid position/force control
	virtual bool moveLinFC(const rw::math::Transform3D<>& target,
							  rw::math::Wrench6D<>& wtarget,
							  float selection[6],
							  std::string refframe,
							  rw::math::Transform3D<> offset,
							  float speed = 100,
							  float blend = 0);

	//! hard stop the robot,
	bool stop();

	//! pause the robot, should be able to continue trajectory
	bool pause();

	//! enable safe mode, so that robot stops when collisions are detected
	bool setSafeModeEnabled(bool enable);

	rw::math::Q getQ();
	rw::math::Q getQd();

	ros::Time getTimeStamp();

	bool isMoving();

protected:
	ros::NodeHandle _nodeHnd;
	ros::ServiceClient _servoService;

	// services
	ros::ServiceClient _srvStop;
	ros::ServiceClient _srvStart;
	ros::ServiceClient _srvPause;

	ros::ServiceClient _srvMovePTP;
	ros::ServiceClient _srvMovePTP_T;
	ros::ServiceClient _srvMoveLin;
	ros::ServiceClient _srvMoveLinFC;
	ros::ServiceClient _srvMoveVelQ;
	ros::ServiceClient _srvMoveVelT;

	// states
	ros::Subscriber _robotState;

	// old interfaces for ur...
	ros::ServiceClient _srvServoQ;
	ros::ServiceClient _srvServoT;


private:
	boost::mutex _mutex;

	// state variables
	rw::math::Q _q, _dq;
	bool _isRunning;
	bool _stopRobot;

	void handleRobotState(const marvin_common::RobotState& state);
	marvin_common::RobotState _pRobotState;
	double _zeroTime;
};

}

#endif //end include guard
