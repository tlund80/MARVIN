/*
 * trackstar_calibrated_nodelet.h
 *
 *  Created on: Jan 28, 2013
 *      Author: TRS
 */



#ifndef TRAKSTAR_CALIBRATION_NODELET_H_
#define TRAKSTAR_CALIBRATION_NODELET_H_

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "Trakstar.hpp"
//
#include "marvin_common/TrakStarState.h"
#include "TrakstarCalibration.h"


class TrakstarCalibratedNodelet : public nodelet::Nodelet {

public:

  TrakstarCalibratedNodelet();
	virtual ~TrakstarCalibratedNodelet();

	virtual void onInit();


	double getLoopRate();
        
        void stop();

        void start();

        void callback(const marvin_common::TrakStarState::ConstPtr& input);

	/*
	 * Publish
	 */
	//void publish();
protected:
	ros::NodeHandle _node_handle;


private:


//	void loop();

	ros::Publisher _state_publisher;
	ros::Subscriber _state_subscriber;
	std::string _service_name;

	TrakstarCalibration _trakstarCalib;

//	Trakstar trakstar_driver_;

//	boost::shared_ptr<boost::thread> deviceThread_;
        boost::mutex _mutex;
        bool _stop;
       // double _maxPubFrequency;

};

#endif /* TRAKSTAR_CALIBRATION_NODELET_H_ */
