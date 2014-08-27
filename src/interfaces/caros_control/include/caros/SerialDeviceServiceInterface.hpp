/**/
#ifndef SERIALDEVICESERVICEINTERFACE_HPP
#define SERIALDEVICESERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/common/Ptr.hpp>

#include <caros_control_msgs/RobotState.h>
#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePTP.h>
#include <caros_control_msgs/SerialDeviceMovePTP_T.h>
#include <caros_control_msgs/SerialDeviceMoveVelQ.h>
#include <caros_control_msgs/SerialDeviceMoveVelT.h>
#include <caros_control_msgs/SerialDeviceMoveLinFC.h>
#include <caros_common_msgs/ConfigBool.h>
#include <caros_common_msgs/Stop.h>
#include <caros_common_msgs/Start.h>
#include <caros_common_msgs/Pause.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <string>

/**
 * @brief this is the top level robot arm interface. It defines the
 * minimum interface that a joint based robotic arm device needs
 * to implement.
 *
 * The namespace of the nodehandle/service_name is used and it is important that
 * not two RobotArmServiceInterfaces are running in the same namespace.
 */
class SerialDeviceServiceInterface {
public:
    typedef rw::common::Ptr<SerialDeviceServiceInterface> Ptr;
	SerialDeviceServiceInterface(const std::string& service_name);

	SerialDeviceServiceInterface(ros::NodeHandle nodeHnd);

	virtual ~SerialDeviceServiceInterface();
protected:

	//! initialize services in the node handle
	void initNodeHandle();


	//! @brief move robot in a linear Cartesian path
	virtual bool moveLin(caros_control_msgs::SerialDeviceMoveLin::Request& request,
	                     caros_control_msgs::SerialDeviceMoveLin::Response& response) = 0;

	//! @brief move robot from point to point
	virtual bool movePTP(caros_control_msgs::SerialDeviceMovePTP::Request& request,
	                     caros_control_msgs::SerialDeviceMovePTP::Response& response) = 0;

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(caros_control_msgs::SerialDeviceMovePTP_T::Request& request,
	                       caros_control_msgs::SerialDeviceMovePTP_T::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying joint velocity targets
	virtual bool moveVelQ(caros_control_msgs::SerialDeviceMoveVelQ::Request& request,
	                      caros_control_msgs::SerialDeviceMoveVelQ::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying a velocity screw in tool coordinates
	virtual bool moveVelT(caros_control_msgs::SerialDeviceMoveVelT::Request& request,
	                      caros_control_msgs::SerialDeviceMoveVelT::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying joint velocity targets
	virtual bool servoQ(caros_control_msgs::SerialDeviceMovePTP::Request& request,
	                    caros_control_msgs::SerialDeviceMovePTP::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying a velocity screw in tool coordinates
	virtual bool servoT(caros_control_msgs::SerialDeviceMovePTP_T::Request& request,
	                    caros_control_msgs::SerialDeviceMovePTP_T::Response& response) = 0;


	//! move robot with a hybrid position/force control
	virtual bool moveLinFC(caros_control_msgs::SerialDeviceMoveLinFC::Request& request,
	                       caros_control_msgs::SerialDeviceMoveLinFC::Response& response) = 0;

	//! hard stop the robot,
	//virtual bool start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) = 0;
	//virtual bool start(caros_common_msgs::Start::Request& request, caros_common_msgs::Start::Response& response) = 0;

	//! hard stop the robot,
	//virtual bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) = 0;
	virtual bool stop(caros_common_msgs::Stop::Request& request, caros_common_msgs::Stop::Response& response) = 0;
	
	//! pause the robot, should be able to continue trajectory
	//virtual bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) = 0;
	//virtual bool pause(caros_common_msgs::Pause::Request& request, caros_common_msgs::Pause::Response& response) = 0;

	//! enable safe mode, so that robot stops when collisions are detected
	//virtual bool setSafeModeEnabled(caros_common_msgs::ConfigBool::Request& request,caros_common_msgs::ConfigBool::Response& response) = 0;

	//! publish robot state
	void publish(const caros_control_msgs::RobotState& state);

protected:
	std::string _service_name;
	ros::NodeHandle _nodeHnd;


    ros::Publisher _deviceStatePublisher;

    ros::ServiceServer _srvMovePTP;
    ros::ServiceServer _srvMovePTP_T;
    ros::ServiceServer _srvMoveLin;
    ros::ServiceServer _srvMoveLinFC;
    ros::ServiceServer _srvMoveVelQ;
    ros::ServiceServer _srvMoveVelT;
    ros::ServiceServer _srvServoQ;
    ros::ServiceServer _srvServoT;

    ros::ServiceServer _srvStop;
    ros::ServiceServer _srvPause;
    ros::ServiceServer _srvSafe;
    ros::ServiceServer _srvStart;

};

#endif //#ifndef URSERVICEINTERFACE_HPP
