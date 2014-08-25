/**/
#ifndef POSESENSORSERVICEINTERFACE_HPP
#define POSESENSORSERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <caros_common_msgs/Stop.h>
#include <caros_common_msgs/Pause.h>

#include <string>

/**
 * @brief standard interface for pose sensor that can track a number of
 *  poses.
 */
class PoseSensorServiceInterface {
public:

    typedef rw::common::Ptr<PoseSensorServiceInterface> Ptr;

    //! constructor
	PoseSensorServiceInterface(const std::string& service_name);

	//! constructor
	PoseSensorServiceInterface(rw::common::Ptr<ros::NodeHandle> nodeHnd);


	void publishPoses(const std::vector<rw::math::Transform3D<> >& poses,
				   const std::vector<int>& ids,
				   const std::vector<float>& qualities);

	bool configurePoseSensor();
	bool cleanupPoseSensor();

private:
    PoseSensorServiceInterface(){};

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

private:
    ros::Publisher _posePublisher;

    ros::ServiceServer _srvStart;
    ros::ServiceServer _srvStop;
    ros::ServiceServer _srvPause;
};

#endif //#ifndef SDHSERVICEINTERFACE_HPP
