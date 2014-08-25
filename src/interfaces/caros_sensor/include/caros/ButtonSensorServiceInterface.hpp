/**/
#ifndef BUTTONSENSORSERVICEINTERFACE_HPP
#define BUTTONSENSORSERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>
#include <string>

/**
 * @brief describe the minimum interface of a Force/Torque sensing device.
 */
class ButtonSensorServiceInterface {
public:
	ButtonSensorServiceInterface();

	ButtonSensorServiceInterface(const std::string& service_name);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

private:

};

#endif
