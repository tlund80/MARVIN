/*
 * trakstar.h
 *
 *  Created on: Jan 28, 2013
 *      Author: mtt
 */



#ifndef TRAKSTAR_NODELET_H_
#define TRAKSTAR_NODELET_H_

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

//#include <tf/transform_broadcaster.h>

#include "Trakstar.hpp"
//
#include "marvin_common/TrakStarState.h"
#include "marvin_common/Stop.h"
#include "marvin_common/Start.h"
#include "marvin_common_rw/ButtonSensorServiceInterface.hpp"
#include "marvin_common_rw/PoseSensorServiceInterface.hpp"

class TrakstarRawNodelet: public nodelet::Nodelet
{
public:

	TrakstarRawNodelet();

	virtual ~TrakstarRawNodelet();

	virtual void onInit();

	double getLoopRate();

	// PoseSensorServiceInterface
	bool start(marvin_common::Start::Request& request, marvin_common::Start::Response& response);
	bool stop(marvin_common::Stop::Request& request, marvin_common::Stop::Response& response);

	/*
	 * Publish
	 */
	void publish();

private:
	bool stopThread();

	bool startThread();

protected:
	ros::NodeHandle _node_handle;

private:
	ros::Publisher _state_publisher;
	ros::Publisher _posearray_publisher;
	ros::Publisher _button_publisher;
	std::string _service_name;

	// Services
	ros::ServiceServer _srvStop;
	ros::ServiceServer _srvStart;

	// Local
	Trakstar trakstar_driver_;

	boost::shared_ptr<boost::thread> _thread;
	boost::mutex _mutex;
	bool _stop, _isrunning;

	// Parameters
	double _maxPubFrequency;
	std::string _frameID;

};

#endif /* TRAKSTAR_NODELET_H_ */
