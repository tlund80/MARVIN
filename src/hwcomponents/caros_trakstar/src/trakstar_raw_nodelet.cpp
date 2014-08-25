/*
 * trackstar.cpp
 *
 *  Created on: Jan 28, 2013
 *      Author: mtt
 */

#include "trakstar_raw_nodelet.h"

#include <pluginlib/class_list_macros.h>

#include <rw/math.hpp>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

using namespace rw::math;

//using namespace marvin_common;
// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(trakstar, TrakstarRawNodelet, TrakstarRawNodelet,
		nodelet::Nodelet)

TrakstarRawNodelet::TrakstarRawNodelet() {
	_stop = false;
	_isrunning = false;
}

TrakstarRawNodelet::~TrakstarRawNodelet() {
	if (_isrunning) {
		stopThread();
	}
}

void TrakstarRawNodelet::onInit() {

	_node_handle = getPrivateNodeHandle();
	_node_handle.param("rate", _maxPubFrequency, 100.0);
	_node_handle.param("frame", _frameID, std::string("TrakstarBase") );

	ROS_INFO_STREAM("rate: "<<_maxPubFrequency);
	ROS_INFO_STREAM("frame: "<<_frameID);

	_state_publisher = _node_handle.advertise<marvin_common::TrakStarState>("TrakstarState_raw", 5);
	_posearray_publisher = _node_handle.advertise<geometry_msgs::PoseArray>("poses_raw", 5);
	_button_publisher = _node_handle.advertise<std_msgs::Bool>("pushed", 5);

	_srvStop = _node_handle.advertiseService("stop", &TrakstarRawNodelet::stop,
			this);
	_srvStart = _node_handle.advertiseService("start",
			&TrakstarRawNodelet::start, this);

	ROS_INFO_STREAM("Sensors Initialised: start");
	trakstar_driver_.InitializeSystem(true);


	ROS_INFO_STREAM("Sensors Initialised: done");
	while (trakstar_driver_.getInitStatus() == TRAKSTAR_STATUS_INITIALIZING) {
		ROS_INFO_STREAM("Initialising.... ");
	}
	if (trakstar_driver_.getInitStatus() == TRAKSTAR_STATUS_STARTED) {
		trakstar_driver_.startPolling();
		ROS_INFO_STREAM(
				"Sensors Initialised: Is polling " << trakstar_driver_.isPolling());
		startThread();
	} else {
		ROS_FATAL("Unable to initialise trakStar");
		ROS_BREAK();
	}

}

bool TrakstarRawNodelet::stop(marvin_common::Stop::Request& request,
		marvin_common::Stop::Response& response) {
	response.success = stopThread();
	return true;
}

/* 
 * @brief Stop the periodic data publishing
 *  
 */
bool TrakstarRawNodelet::stopThread() {
	try {
		_mutex.lock();
		_stop = true;
		_mutex.unlock();
		_thread->join();
		_isrunning = false;
		return true;
	} catch (...) {
		ROS_WARN("TRAKSTAR STOP FAILED");
		return false;
	}
}

bool TrakstarRawNodelet::start(marvin_common::Start::Request& request,
		marvin_common::Start::Response& response) {
	startThread();
	return true;
}

/* 
 * @brief Starts the periodic data publishing
 *  
 * Starts a new thread that periodically publishes data until stopped.
 */
bool TrakstarRawNodelet::startThread() {
	try {
		_mutex.lock();
		_stop = false;
		_mutex.unlock();
		_thread = boost::shared_ptr<boost::thread>(
				new boost::thread(
						boost::bind(&TrakstarRawNodelet::publish, this)));
		_isrunning = true;
		return true;
	} catch (...) {
		ROS_WARN("TRAKSTAR START FAILED");
		return false;
	}
}

void TrakstarRawNodelet::publish() {

	bool _keepRunning;
	_mutex.lock();
	_keepRunning = !_stop;
	_mutex.unlock();

	ros::Rate loopRate(_maxPubFrequency);
	while (_keepRunning) {
		if (!trakstar_driver_.isPolling()) {
			loopRate.sleep();
			continue;
		}

		std::vector<Trakstar::PoseData> pd = trakstar_driver_.getData();
		geometry_msgs::PoseArray array;
		std_msgs::Bool buttonMsg;

		marvin_common::TrakStarStatePtr msgs(
				new marvin_common::TrakStarState);
		for (unsigned int i = 0; i < pd.size(); i++) {
			marvin_common::TrakStarPose tsp;
			tsp.pose.orientation.w = pd[i].rot.getQw();
			tsp.pose.orientation.x = -pd[i].rot.getQx();
			tsp.pose.orientation.y = -pd[i].rot.getQy();
			tsp.pose.orientation.z = -pd[i].rot.getQz();
			// convert to meters
			tsp.pose.position.x = pd[i].pos[0] * 0.001;
			tsp.pose.position.y = pd[i].pos[1] * 0.001;
			tsp.pose.position.z = pd[i].pos[2] * 0.001;
			tsp.status = pd[i].status;
			tsp.valid = pd[i].valid;
			tsp.button = pd[i].analogButtonOn;
			if (pd[i].analogButtonOn)
				buttonMsg.data = 1;
			else
				buttonMsg.data = 0;

			msgs->poses.push_back(tsp);

			array.poses.push_back(tsp.pose);

			// the posesensor stuff
			//poses.push_back( Transform3D<>(pd[i].pos, pd[i].rot.toRotation3D()) );
			//ids.push_back(i);
			//qualities.push_back(pd[i].quality);
			// the button sensor stuff

		}

		array.header.frame_id = _frameID;
		array.header.stamp = ros::Time::now();

		msgs->header.frame_id = _frameID;
		msgs->header.stamp = array.header.stamp;

		//ROS_INFO_STREAM("got " << msgs->poses.size() << "poses");
		_state_publisher.publish(msgs);
		_posearray_publisher.publish(array);
		_button_publisher.publish(buttonMsg);

		loopRate.sleep();

//     double cycletime_ms = 1000./_maxPubFrequency;
//     ROS_INFO_STREAM("TrakStar cycletime " << cycletime_ms << " ms");
// 
//     boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
//     double duration_us = (end - start).total_microseconds();
//     boost::this_thread::sleep(boost::posix_time::milliseconds(cycletime_ms-duration_us/1000.));
		_mutex.lock();
		_keepRunning = !_stop;
		_mutex.unlock();
	}

}

