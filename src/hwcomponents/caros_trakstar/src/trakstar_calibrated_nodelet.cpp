/*
 * trackstar_calibrated_nodelet.cpp
 *
 *  Created on: Jan 28, 2013
 *      Author: TRS
 */

#include "trakstar_calibrated_nodelet.h"

////#include "marvin_common_rw/RwRos.hpp"
//
//#include <boost/thread.hpp>
//
//#include <geometry_msgs/Quaternion.h>

#include <pluginlib/class_list_macros.h>

//using namespace marvin_common;
// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(trakstar, TrakstarCalibratedNodelet,
		TrakstarCalibratedNodelet, nodelet::Nodelet)

TrakstarCalibratedNodelet::TrakstarCalibratedNodelet() {
	_stop = false;
}

TrakstarCalibratedNodelet::~TrakstarCalibratedNodelet() {
	// TODO Auto-generated destructor stub
	stop();
	// deviceThread_->join();
}

void TrakstarCalibratedNodelet::onInit() {
	_node_handle = getPrivateNodeHandle();
	_state_publisher = _node_handle.advertise < marvin_common::TrakStarState
			> ("TrakstarState", 5);
	_state_subscriber = _node_handle.subscribe("in", 1,
			&TrakstarCalibratedNodelet::callback, this);
	ROS_INFO_STREAM("manager name:" << getMyArgv().size() << " "); //<<getMyArgv().at(0));
	/*
	 ROS_INFO_STREAM("Sensors Initialised: start");
	 trakstar_driver_.InitializeSystem(true);

	 ROS_INFO_STREAM("Sensors Initialised: done");
	 while (trakstar_driver_.getInitStatus() == TRAKSTAR_STATUS_INITIALIZING)
	 {
	 ROS_INFO_STREAM("Initialising.... ");
	 }
	 if (trakstar_driver_.getInitStatus() == TRAKSTAR_STATUS_STARTED)
	 {
	 trakstar_driver_.startPolling();
	 ROS_INFO_STREAM("Sensors Initialised: Is polling " << trakstar_driver_.isPolling());
	 deviceThread_ = boost::shared_ptr < boost::thread
	 > (new boost::thread(boost::bind(&TrakstarCalibratedNodelet::publish, this)));
	 }
	 else
	 {
	 ROS_ERROR("Unable to initialise trakStar");
	 }
	 */
}

void TrakstarCalibratedNodelet::callback(
		const marvin_common::TrakStarState::ConstPtr& input) {
	//ROS_INFO_STREAM("msg in ");

	double x(input->poses.at(2).pose.position.x);
	double y(input->poses.at(2).pose.position.y);
	double z(input->poses.at(2).pose.position.z);
	//ROS_INFO_STREAM("in x: " << x << "  y: " << y << "  z: " << z);
	_trakstarCalib.getCalibratedPosition(x, y, z);
	//ROS_INFO_STREAM("out x: " << x << "  y: " << y << "  z: " << z);
	//ROS_INFO_STREAM("msg in");
}

void TrakstarCalibratedNodelet::stop() {
	_mutex.lock();
	_stop = true;
	_mutex.unlock();
}
/*
 void TrakstarCalibratedNodelet::publish()
 {

 bool keeprunning;
 _mutex.lock();
 keeprunning = !_stop;
 _mutex.unlock();


 while (keeprunning)
 {
 boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
 if (trakstar_driver_.isPolling())
 {

 std::vector<Trakstar::PoseData> pd = trakstar_driver_.getData();

 marvin_common::TrakStarStatePtr msgs(new marvin_common::TrakStarState);
 for (unsigned int i = 0; i < pd.size(); i++)
 {
 marvin_common::TrakStarPose tsp;
 tsp.pose.orientation.w = pd[i].rot.getQw();
 tsp.pose.orientation.x = pd[i].rot.getQx();
 tsp.pose.orientation.y = pd[i].rot.getQy();
 tsp.pose.orientation.z = pd[i].rot.getQz();
 tsp.pose.position.x = pd[i].pos[0];
 tsp.pose.position.y = pd[i].pos[1];
 tsp.pose.position.z = pd[i].pos[2];
 tsp.status = pd[i].status;
 tsp.valid = pd[i].valid;

 msgs->poses.push_back(tsp);
 }
 ROS_INFO_STREAM("got " << msgs->poses.size() << "poses");
 _state_publisher.publish(msgs);
 }

 double cycletime_ms = 1000./_maxPubFrequency;

 boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
 double duration_us = (end - start).total_microseconds();
 boost::this_thread::sleep(boost::posix_time::milliseconds(cycletime_ms-duration_us/1000.));
 std::cerr << "took " << (end - start).total_microseconds() << std::endl;
 _mutex.lock();
 keeprunning = !_stop;
 _mutex.unlock();
 }

 }
 */
