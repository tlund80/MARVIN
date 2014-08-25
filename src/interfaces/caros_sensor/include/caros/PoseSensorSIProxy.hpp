/**/
#ifndef PoseSensorSIProxy_HPP_
#define PoseSensorSIProxy_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math.hpp>
#include <rw/trajectory/Path.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <caros_sensor_msgs/PoseSensorState.h>
#include <queue>

namespace caros {

/**
 * @brief this class implements a cpp proxy to control and read data from
 * a PoseSensorServiceInterface.
 *
 */
class PoseSensorSIProxy {

public:
	typedef rw::common::Ptr<PoseSensorSIProxy> Ptr;

	//! constructor - create with device name
	PoseSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle);

	PoseSensorSIProxy(const std::string& devname);

	//! destructor
	virtual ~PoseSensorSIProxy();

	struct PoseData {
		rw::math::Transform3D<> pose;
		int id;
		float quality;
		ros::Time stamp;
		std::string frame;
	};

	std::vector<PoseData> getPoses();

	ros::Time getTimeStamp();

protected:

	void handlePoseSensorState(const caros_sensor_msgs::PoseSensorState& state);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

	// states
	ros::Subscriber _poseSensorState;


private:
	boost::mutex _mutex;

	// state variables
	std::vector<PoseData> _poses;
	ros::Time _stamp;
};

}

#endif //end include guard
