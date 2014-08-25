/**/
#ifndef MOBILEDEVICESERVICEINTERFACE_HPP
#define MOBILEDEVICESERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <ros/ros.h>
#include <string>

/**
 * @brief describe the minimum interface of a Force/Torque sensing device.
 */
class MobileDeviceServiceInterface {
public:
	MobileDeviceServiceInterface(const std::string& service_name);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

private:
    //ros::Publisher _FTSensorStatePublisher;
    //ros::ServiceServer _srvMoveQ;

};

#endif //#ifndef SDHSERVICEINTERFACE_HPP
