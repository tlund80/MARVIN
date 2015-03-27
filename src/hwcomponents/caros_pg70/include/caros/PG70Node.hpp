/**/
#ifndef PG70Node_HPP
#define PG70Node_HPP

#include <caros/CarosNodeServiceInterface.hpp>
#include <caros/GripperServiceInterface.hpp>

//#include "PG70ServiceInterface.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rwhw/schunkpg70/SchunkPG70.hpp>

#include <vector>
#include <ros/ros.h>

#define MIN_TIME_THRESHOLD_FOR_CALCULATING_VELOCITY 0.1
#define MOVE_DISTANCE_STOPPED_THRESHOLD 0.01

#define SUPPORTED_Q_LENGTH_FOR_PG70NODE 2

class PG70Node: public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface {
public:
	//PG70(const rw::common::PropertyMap& properties);
	PG70Node(const ros::NodeHandle& nodehandle);
	
	//! destructor
        virtual ~PG70Node();
	
	//! @copydoc caros::GripperServiceInterface::moveQ
	bool moveQ(const rw::math::Q& q);
	//! @copydoc caros::GripperServiceInterface::moveQ
	bool gripQ(const rw::math::Q& q);
	//! @copydoc caros::GripperServiceInterface::setForceQ
	bool setForceQ(const rw::math::Q& q);
	//! @copydoc caros::GripperServiceInterface::setVelocityQ
	bool setVelocityQ(const rw::math::Q& q);
	//! @copydoc caros::GripperServiceInterface::stopMovement
	bool stopMovement();
	
	//Extra functions
	bool home();
	bool open(float force);
	bool close(float force);
	
	/* TODO: Properly document the error codes */
	/* TODO: Consider better error codes for SDHNODE_INTERNAL_ERROR */
	/* The enum order should not be changed, as recorded ROS sessions would then be invalidated */ 
	enum PG70NODE_ERRORCODE { PG70NODE_SDH_DEVICE_ALREADY_ACTIVE = 1, PG70NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL, PG70NODE_UNSUPPORTED_INTERFACE_TYPE, PG70NODE_PG70_DEVICE_CONNECT_FAILED, PG70NODE_INTERNAL_ERROR, PG70NODE_PG70_DEVICE_NO_CONNECTION, PG70NODE_NO_PG70_DEVICE, PG70NODE_UNSUPPORTED_Q_LENGTH };


protected:
	// hooks implemented from CarosNodeServiceInterface base class
	bool configureHook();
	bool cleanupHook();
	bool startHook();
	bool stopHook();
	bool recoverHook();

	void initLoopHook();
	void stoppedLoopHook();
	void runLoopHook();
	void errorLoopHook();
	void fatalErrorLoopHook();
	
private:
	rwhw::SchunkPG70* _pg70;
	
	rw::common::Timer _moveStartTimer, _velUpdateTimer;
       rw::math::Q _moveQ, _velQ, _currentQ, _lastQ;
	
	ros::NodeHandle _nh;
	std::string _port;
	std::string _name;

};

#endif //#ifndef PG70Node_HPP
