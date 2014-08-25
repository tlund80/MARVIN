/**/
#ifndef CAROS_ButtonSensorSIProxy_HPP_
#define CAROS_ButtonSensorSIProxy_HPP_

#include <caros_sensor_msgs/ButtonSensorState.h>

#include <rw/common/Ptr.hpp>
#include <rw/math.hpp>
#include <rw/trajectory/Path.hpp>
#include <boost/thread.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include <queue>

namespace caros {

/**
 * @brief this class implements a cpp proxy to control and read data from
 * a ButtonSensorServiceInterface.
 *
 */
class ButtonSensorSIProxy {

public:
	typedef rw::common::Ptr<ButtonSensorSIProxy> Ptr;

	//! constructor
	ButtonSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle);

	//! constructor
    ButtonSensorSIProxy(const std::string& devname);

	//! destructor
	virtual ~ButtonSensorSIProxy();

	struct ButtonData {
		float button;
		std::string id;
		bool isAnalog;
		ros::Time stamp;
	};

	std::vector<ButtonData> getButtons();

	ros::Time getTimeStamp();

protected:
	void handleButtonSensorState(const caros_sensor_msgs::ButtonSensorState& state);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

	// states
	ros::Subscriber _buttonSensorState;

private:
	boost::mutex _mutex;

	// state variables
	std::vector<ButtonData> _buttons;
	ros::Time _stamp;
};

}
#endif //end include guard
