/**/
#ifndef UNIVERSALROBOTS_HPP
#define UNIVERSALROBOTS_HPP

#include "marvin_common/WrenchData.h"
#include <marvin_common_rw/SerialDeviceServiceInterface.hpp>
#include <marvin_common/URServo.h>
#include <marvin_common/URServoQ.h>
#include <marvin_common/SerialDeviceForceControlStart.h>
#include <marvin_common/SerialDeviceForceControlUpdate.h>
#include <marvin_common/SerialDeviceForceControlStop.h>
#include <rw/invkin/JacobianIKSolver.hpp>
#include "URServiceInterface.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Metric.hpp>
#include <rwlibs/algorithms/xqpcontroller/XQPController.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/netft/NetFTLogging.hpp>
#include <rwhw/netft/FTCompensation.hpp>


#include <boost/thread.hpp>
#include <vector>

class UniversalRobots: public SerialDeviceServiceInterface {
public:
	UniversalRobots(rw::models::WorkCell::Ptr workcell, const rw::common::PropertyMap& properties, int looprate, const std::string& wrenchTopic, rwhw::FTCompensation::Ptr pFTCompensation);

	bool run();

protected:
	virtual void loop();
	virtual void stopDriver();

	virtual bool moveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);
	virtual bool moveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	virtual bool safeMoveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	virtual bool safeMoveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);

    virtual bool servoQ(const rw::math::Q& qtarget);
	virtual bool servoT(const rw::math::Transform3D<>& target);


	//! @brief move robot in a linear Cartesian path
	virtual bool moveLin(marvin_common::SerialDeviceMoveLin::Request& request,
				  marvin_common::SerialDeviceMoveLin::Response& response);

	//! @brief move robot from point to point
	virtual bool movePTP(marvin_common::SerialDeviceMovePTP::Request& request,
				  marvin_common::SerialDeviceMovePTP::Response& response);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(marvin_common::SerialDeviceMovePTP_T::Request& request,
				  marvin_common::SerialDeviceMovePTP_T::Response& response);

	virtual bool servoQ(marvin_common::SerialDeviceMovePTP::Request& request,
				  marvin_common::SerialDeviceMovePTP::Response& response);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool servoT(marvin_common::SerialDeviceMovePTP_T::Request& request,
				  marvin_common::SerialDeviceMovePTP_T::Response& response);


	//! @brief move robot in a servoing fasion
	virtual bool moveVelQ(marvin_common::SerialDeviceMoveVelQ::Request& request,
				   marvin_common::SerialDeviceMoveVelQ::Response& response);

	virtual bool moveVelT(marvin_common::SerialDeviceMoveVelT::Request& request,
				   marvin_common::SerialDeviceMoveVelT::Response& response);

	//! move robot with a hybrid position/force control
	virtual bool moveLinFC(marvin_common::SerialDeviceMoveLinFC::Request& request,
				     	   marvin_common::SerialDeviceMoveLinFC::Response& response);

	//! hard stop the robot,
	virtual bool stop(marvin_common::Stop::Request& request,
					  marvin_common::Stop::Response& response);

	//! pause the robot, should be able to continue trajectory
	//virtual bool pause(marvin_common::Pause::Request& request,
	//					 marvin_common::Pause::Response& response){return false;};

	//! enable safe mode, so that robot stops when collisions are detected
	//virtual bool setSafeModeEnabled(marvin_common::ConfigBool::Request& request,
	//								    marvin_common::ConfigBool::Request& response){return false;};


	bool servoHandle(marvin_common::URServo::Request& request, marvin_common::URServo::Response& response);
	bool servoQHandle(marvin_common::URServoQ::Request& request, marvin_common::URServoQ::Response& response);

	bool forceControlStart(marvin_common::SerialDeviceForceControlStart::Request& request,
					    marvin_common::SerialDeviceForceControlStart::Response& response);

	bool forceControlUpdate(marvin_common::SerialDeviceForceControlUpdate::Request& request,
				 	 	 marvin_common::SerialDeviceForceControlUpdate::Response& response);

	bool forceControlStop(marvin_common::SerialDeviceForceControlStop::Request& request,
			 	 	   marvin_common::SerialDeviceForceControlStop::Response& response);



private:
    ros::Rate _loopRate;
	boost::mutex _mutex;
	rw::models::WorkCell::Ptr _workcell;
	rw::models::Device::Ptr _device;
	rw::math::Q _qcurrent;
	rw::kinematics::Frame* _ftFrame;

	rwhw::URCallBackInterface _ur;
	rwhw::UniversalRobotsRTLogging _urrt;
	rw::common::PropertyMap _properties;

//	rwhw::NetFTLogging::Ptr _pNetFT;
	rwhw::FTCompensation::Ptr _pFTCompensation;
	ros::Subscriber subFTData;
	std::queue<rwhw::Wrench3D> _wrenchDataQueue;

	rw::invkin::JacobianIKSolver::Ptr _iksolver;
	rw::kinematics::State _state;
    ros::ServiceServer _srvServo;
    ros::ServiceServer _srvServoQ;
    ros::ServiceServer _srvForceModeStart;
    ros::ServiceServer _srvForceModeUpdate;
    ros::ServiceServer _srvForceModeStop;

	bool _useFTCollisionDetection;
	double _driverTimeOffset;

//	bool _servoMode;
//	rw::math::Q _qservo;
//	rw::math::Q _dqservo;
	double _dt;
//	rwlibs::algorithms::XQPController::Ptr _xqp;
	//rw::math::Transform3D<> _servoTarget;
	//rw::math::VelocityScrew6D<> _servoVelocity;

	bool updateFTBias();
	void addFTData(const marvin_common::WrenchData::ConstPtr& state);
 
//	void servo();
    rw::math::QMetric::Ptr _q2cmetric;

	std::string _errorMsg;



};

#endif //#ifndef UNIVERSALROBOTS_HPP
