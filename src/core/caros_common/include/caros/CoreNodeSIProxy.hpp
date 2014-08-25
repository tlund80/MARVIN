/**/
#ifndef ButtonSensorSIProxy_HPP_
#define ButtonSensorSIProxy_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math.hpp>
#include <rw/trajectory/Path.hpp>
#include <boost/thread.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include <queue>


/**
 * @brief this class implements a cpp proxy to control and read data from
 * a ButtonSensorServiceInterface.
 *
 */
class ButtonSensorSIProxy {

public:
	typedef rw::common::Ptr<ButtonSensorSIProxy> Ptr;

	//! constructor - create with device name
	ButtonSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle, const std::string& devname);

	//! destructor
	virtual ~ButtonSensorSIProxy();

	//! hard stop the robot,
	bool start();

	//! hard stop the robot,
	bool stop();

	//! pause the robot, should be able to continue trajectory
	bool pause();

	struct ButtonData {
		float button;
		std::string id;
		bool isAnalog;
		ros::Time stamp;
	};

	std::vector<ButtonData> getButtons();

	ros::Time getTimeStamp();


	void handleButtonSensorState(const marvin_common::ButtonSensorState& state);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
	// services
	ros::ServiceClient _srvStart;
	ros::ServiceClient _srvStop;
	ros::ServiceClient _srvPause;

	// states
	ros::Subscriber _buttonSensorState;


private:
	boost::mutex _mutex;

	// state variables
	std::vector<ButtonData> _buttons;
	ros::Time _stamp;
	double _zeroTime;
};

#endif //end include guard
