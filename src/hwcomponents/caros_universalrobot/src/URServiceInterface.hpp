/**/
#ifndef URSERVICEINTERFACE_HPP
#define URSERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include "marvin_common/URStop.h"
#include "marvin_common/URMoveL.h"
#include "marvin_common/URMoveQ.h"
#include "marvin_common/URServo.h"
#include "marvin_common/URServoQ.h"

#include "marvin_common/RobotState.h"
#include "marvin_common/WrenchData.h" 
#include "marvin_common/TimeSignal.h"
#include "marvin_common/MarvinUtils.hpp"

#include "ros/ros.h" 
#include <string>


class URServiceInterface {
public:
	URServiceInterface(const std::string& service_name, double loopRate, bool supportSafeMotions);

	bool run();

protected:
  //  MarvinTime _marvinTime;

	virtual bool moveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed) = 0;
	virtual bool moveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed) = 0;

	virtual bool safeMoveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed) = 0;
	virtual bool safeMoveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed) = 0;


    virtual bool servoQ(const rw::math::Q& qtarget) = 0;
	virtual bool servoT(const rw::math::Transform3D<>& target, const rw::math::VelocityScrew6D<>& velocity) = 0;

	virtual bool forceModeStart() = 0;
	virtual bool forceModeUpdate() = 0;
	virtual bool forceModeEnd() = 0;

	virtual bool stop() = 0;
	virtual bool pause() = 0;
	virtual bool start() = 0;
	virtual bool wait() = 0;


	virtual void loop() = 0;
	virtual void stopDriver() = 0;

	double getLoopRate();
	void publish(const rw::math::Q& q);
	void publish(const marvin_common::RobotState& state);
	void publish(const marvin_common::WrenchData& data);

	const std::string& name() { return _service_name; };

private:
	void timeSignal(marvin_common::TimeSignal::ConstPtr time);

	bool moveLHandle(marvin_common::URMoveL::Request& request, marvin_common::URMoveL::Response& response);
	bool moveQHandle(marvin_common::URMoveQ::Request& request, marvin_common::URMoveQ::Response& response);

	bool safeMoveLHandle(marvin_common::URMoveL::Request& request, marvin_common::URMoveL::Response& response);
	bool safeMoveQHandle(marvin_common::URMoveQ::Request& request, marvin_common::URMoveQ::Response& response);

	bool servoHandle(marvin_common::URServo::Request& request, marvin_common::URServo::Response& response);
	bool servoQHandle(marvin_common::URServoQ::Request& request, marvin_common::URServoQ::Response& response);

	bool stopHandle(marvin_common::URStop::Request& request, marvin_common::URStop::Response& response);
	bool pauseHandle();
	bool startHandle();
	bool waitHandle();


protected:
	ros::NodeHandle _nodeHnd;
	ros::Subscriber _subTimeSignal;

    ros::Publisher _deviceStatePublisher;
    ros::Publisher _wrenchDataPublisher;
    ros::ServiceServer _srvMoveQ;
    ros::ServiceServer _srvMoveL;
    ros::ServiceServer _srvServo;
    ros::ServiceServer _srvServoQ;
    ros::ServiceServer _srvStop;

    ros::ServiceServer _srvSafeMoveQ;
    ros::ServiceServer _srvSafeMoveL;

    std::string _service_name;
    ros::Rate _loopRate;


    /*ros::Duration _syncTimeOffset;
    ros::Duration _driverTimeOffset;
*/
};

#endif //#ifndef URSERVICEINTERFACE_HPP
