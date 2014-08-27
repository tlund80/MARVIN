/**/
#ifndef UNIVERSALROBOTS_HPP
#define UNIVERSALROBOTS_HPP

#include <caros_control_msgs/WrenchData.h>
#include <../../../MARVIN/src/interfaces/caros_control/include/caros/SerialDeviceServiceInterface.hpp>

//#include <marvin_common/URServo.h>
//#include <marvin_common/URServoQ.h>
#include <caros_common_msgs/Start.h>
#include <caros_common_msgs/Stop.h>
#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePTP.h>
#include <caros_control_msgs/SerialDeviceMovePTP_T.h>
#include <caros_control_msgs/SerialDeviceMoveVelQ.h>
#include <caros_control_msgs/SerialDeviceMoveVelT.h>
#include <caros_control_msgs/SerialDeviceMoveLinFC.h>
#include <caros_control_msgs/SerialDeviceForceControlStart.h>
#include <caros_control_msgs/SerialDeviceForceControlUpdate.h>
#include <caros_control_msgs/SerialDeviceForceControlStop.h>
#include <rw/invkin/JacobianIKSolver.hpp>
//#include "URServiceInterface.hpp"

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

/*	virtual bool moveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);
	virtual bool moveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	virtual bool safeMoveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	virtual bool safeMoveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);

	virtual bool servoQ(const rw::math::Q& qtarget);
	virtual bool servoT(const rw::math::Transform3D<>& target);
*/	
	bool moveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);
	bool moveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	bool safeMoveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	bool safeMoveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);

	virtual bool servoQ(const rw::math::Q& qtarget);
	virtual bool servoT(const rw::math::Transform3D<>& target);


	//! @brief move robot in a linear Cartesian path
	virtual bool moveLin(caros_control_msgs::SerialDeviceMoveLin::Request& request,
				  caros_control_msgs::SerialDeviceMoveLin::Response& response);

	//! @brief move robot from point to point
	virtual bool movePTP(caros_control_msgs::SerialDeviceMovePTP::Request& request,
				  caros_control_msgs::SerialDeviceMovePTP::Response& response);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(caros_control_msgs::SerialDeviceMovePTP_T::Request& request,
				  caros_control_msgs::SerialDeviceMovePTP_T::Response& response);

	virtual bool servoQ(caros_control_msgs::SerialDeviceMovePTP::Request& request,
				  caros_control_msgs::SerialDeviceMovePTP::Response& response);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool servoT(caros_control_msgs::SerialDeviceMovePTP_T::Request& request,
				  caros_control_msgs::SerialDeviceMovePTP_T::Response& response);


	//! @brief move robot in a servoing fasion
	virtual bool moveVelQ(caros_control_msgs::SerialDeviceMoveVelQ::Request& request,
				   caros_control_msgs::SerialDeviceMoveVelQ::Response& response);

	virtual bool moveVelT(caros_control_msgs::SerialDeviceMoveVelT::Request& request,
				   caros_control_msgs::SerialDeviceMoveVelT::Response& response);

	//! move robot with a hybrid position/force control
	virtual bool moveLinFC(caros_control_msgs::SerialDeviceMoveLinFC::Request& request,
				     	   caros_control_msgs::SerialDeviceMoveLinFC::Response& response);

	//! hard stop the robot,
	virtual bool stop(caros_common_msgs::Stop::Request& request,
					  caros_common_msgs::Stop::Response& response);

	//! pause the robot, should be able to continue trajectory
	//virtual bool pause(marvin_common::Pause::Request& request,
	//					 marvin_common::Pause::Response& response){return false;};

	//! enable safe mode, so that robot stops when collisions are detected
	//virtual bool setSafeModeEnabled(marvin_common::ConfigBool::Request& request,
	//								    marvin_common::ConfigBool::Request& response){return false;};


	bool servoHandle(caros_control_msgs::SerialDeviceMovePTP::Request& request, caros_control_msgs::SerialDeviceMovePTP::Response& response);
	bool servoQHandle(caros_control_msgs::SerialDeviceMovePTP::Request& request, caros_control_msgs::SerialDeviceMovePTP::Response& response);

	bool forceControlStart(caros_control_msgs::SerialDeviceForceControlStart::Request& request,
					    caros_control_msgs::SerialDeviceForceControlStart::Response& response);

	bool forceControlUpdate(caros_control_msgs::SerialDeviceForceControlUpdate::Request& request,
				 	 	 caros_control_msgs::SerialDeviceForceControlUpdate::Response& response);

	bool forceControlStop(caros_control_msgs::SerialDeviceForceControlStop::Request& request,
			 	 	   caros_control_msgs::SerialDeviceForceControlStop::Response& response);



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
	void addFTData(const caros_control_msgs::WrenchData::ConstPtr& state);
 
//	void servo();
    rw::math::QMetric::Ptr _q2cmetric;

	std::string _errorMsg;



};

#endif //#ifndef UNIVERSALROBOTS_HPP
