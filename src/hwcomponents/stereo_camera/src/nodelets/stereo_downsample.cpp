// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/**
 \author Pascal Vanmechelen
 \file stereoCamera.cpp
 \brief
 */

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include "Mathematics/StereoCalibration.h"
#include "Mathematics/CameraCalibration.h"
#include <caros_common_msgs/StereoCalibration.h>

//copied from stereo_preprocess
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include <ros/console.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <caros/CovisRos.hpp>
#include "Mathematics/StereoCalibration.h"
#include "Image/ImageFunction/PrepAndEarlyVisionCV.h"
//#include <caros_common_msgs/StereoCalibration.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


using std::string;

using namespace cv;
using namespace sensor_msgs;

typedef boost::shared_ptr<caros_common_msgs::StereoCalibration const> StereoCalibrationConstPtr;
namespace nodelet_stereo_camera
{

class StereoDownsample : public nodelet::Nodelet
{
public:
	StereoDownsample()	{}
	~StereoDownsample()	{}

private:
	virtual void onInit();

	void connectCb();

	// Publications
	boost::mutex connect_mutex_;
	boost::mutex process_mutex_;
	boost::shared_ptr<image_transport::ImageTransport> it_;

	image_transport::Publisher pub_left_image_downsampled;
	image_transport::Publisher pub_right_image_downsampled;
	ros::Publisher pub_stereo_calib_downsampled;

	image_transport::SubscriberFilter sub_left_image_rect;
	image_transport::SubscriberFilter sub_right_image_rect;
	message_filters::Subscriber<caros_common_msgs::StereoCalibration> sub_calib_rect;

	int queue_size_;


	void imageCb(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image,
			const caros_common_msgs::StereoCalibrationConstPtr& msg_calib_raw);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
			caros_common_msgs::StereoCalibration> ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ApproximateSync> approximate_sync_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image,
			caros_common_msgs::StereoCalibration> ExactPolicy;
	typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
	boost::shared_ptr<ExactSync> exact_sync_;


};

void StereoDownsample::onInit()
{
	//  ROS_INFO_STREAM(" StereoDownsample::onInit()");
	ros::NodeHandle &nh = getNodeHandle();
	//ros::NodeHandle &private_nh = getPrivateNodeHandle();
	it_.reset(new image_transport::ImageTransport(nh));

	queue_size_ = 5;

	bool approx = false;
	if (approx)
	{
		approximate_sync_.reset(
				new ApproximateSync(ApproximatePolicy(queue_size_), sub_left_image_rect, sub_right_image_rect, sub_calib_rect));
		approximate_sync_->registerCallback(boost::bind(&StereoDownsample::imageCb, this, _1, _2, _3));
	}
	else
	{
		exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_), sub_left_image_rect, sub_right_image_rect, sub_calib_rect));
		exact_sync_->registerCallback(boost::bind(&StereoDownsample::imageCb, this, _1, _2, _3));
	}

	// Set up dynamic reconfiguration
	//  ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCb,
	//                                                  this, _1, _2);
	//  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
	//  reconfigure_server_->setCallback(f);

	//  count=0;

	image_transport::SubscriberStatusCallback connect_cb = boost::bind(&StereoDownsample::connectCb, this);
	// Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
	boost::lock_guard<boost::mutex> lock(connect_mutex_);



	pub_left_image_downsampled=it_->advertise("downsample/left/image_rect_color", 5, connect_cb);
	pub_right_image_downsampled=it_->advertise("downsample/right/image_rect_color", 5, connect_cb);
	pub_stereo_calib_downsampled=nh.advertise<caros_common_msgs::StereoCalibration>("downsample/stereo_calib_rect", 5);

}
void StereoDownsample::imageCb(const sensor_msgs::ImageConstPtr& msg_left_image,
		const sensor_msgs::ImageConstPtr& msg_right_image,
		const caros_common_msgs::StereoCalibrationConstPtr& msg_calib_info){
	ROS_INFO("Callback called");
	//msg_left_image->height;

	cv::Mat currentImageLeft,currentImageRight, downsampledImageLeft, downsampledImageRight;

	cv_bridge::CvImagePtr leftImageCVPtr, rightImageCVPtr;

	try {
		leftImageCVPtr = cv_bridge::toCvCopy( msg_left_image, "bgr8");
		rightImageCVPtr = cv_bridge::toCvCopy( msg_right_image, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from encodings. %s", e.what());
	}

	cv_bridge::CvImagePtr leftImageCVPtrDS(new cv_bridge::CvImage);
	cv_bridge::CvImagePtr rightImageCVPtrDS(new cv_bridge::CvImage);

	currentImageLeft=leftImageCVPtr->image;
	currentImageRight=rightImageCVPtr->image;

	//scalefactor hardcoded
	double scalefactor=0.5;
	cv::resize(currentImageLeft, downsampledImageLeft, cv::Size(), scalefactor, scalefactor, INTER_LINEAR);
	cv::resize(currentImageRight, downsampledImageRight, cv::Size(), scalefactor, scalefactor, INTER_LINEAR);
	ROS_INFO("Images downsampled");

	//transferring images to ROS
	cv_bridge::CvImage image_left_downsampled, image_right_downsampled;

	image_left_downsampled.header   = msg_left_image->header;
	image_left_downsampled.encoding = "bgr8";
	image_left_downsampled.image    = downsampledImageLeft; // Your cv::Mat
	image_right_downsampled.header   = msg_right_image->header;
	image_right_downsampled.encoding = "bgr8";
	image_right_downsampled.image    = downsampledImageRight;

	pub_left_image_downsampled.publish(image_left_downsampled.toImageMsg());
	pub_right_image_downsampled.publish(image_right_downsampled.toImageMsg());

	/////adjust calibration/////
	Calibration::StereoCalibration sc = CovisRos::toCovis(*msg_calib_info);

	Calibration::CameraCalibration ccLeft=sc.leftCamera();
	Calibration::CameraCalibration ccRight=sc.rightCamera();

	int original_height_left=msg_left_image->height;
	int original_width_left=msg_left_image->width;
	int original_height_right=msg_right_image->height;
	int original_width_right=msg_right_image->width;

	ccLeft.calculateProjectionMatrixForResize(original_width_left, original_height_left,original_width_left*scalefactor, original_height_left*scalefactor);
	ccRight.calculateProjectionMatrixForResize(original_width_right, original_height_right,original_width_right*scalefactor, original_height_right*scalefactor);
	Calibration::StereoCalibration scnew=Calibration::StereoCalibration(ccLeft, ccRight);

	ROS_INFO("Camera calibration adjusted");
	caros_common_msgs::StereoCalibration stereocalibmsg = CovisRos::toOwnRos(scnew);
	//??
	stereocalibmsg.header = msg_left_image->header;
	pub_stereo_calib_downsampled.publish(stereocalibmsg);
}
void StereoDownsample::connectCb()
{
	boost::lock_guard<boost::mutex> lock(connect_mutex_);

	if (pub_left_image_downsampled.getNumSubscribers() == 0 && pub_right_image_downsampled.getNumSubscribers() == 0
			&& pub_stereo_calib_downsampled.getNumSubscribers() == 0)
	{
		sub_left_image_rect.unsubscribe();
		sub_right_image_rect.unsubscribe();
		sub_calib_rect.unsubscribe();

	}
	else if (!sub_left_image_rect.getSubscriber())
	{
		ros::NodeHandle &nh = getNodeHandle();
		ROS_INFO_STREAM("StereoDownsample subscription is on");
		//image_transport::TransportHints hints("rect", ros::TransportHints(), getPrivateNodeHandle());
		sub_left_image_rect.subscribe(*it_, "left/image_rect_color", 5);
		sub_right_image_rect.subscribe(*it_, "right/image_rect_color", 5);
		sub_calib_rect.subscribe(nh, "stereo_calib_rect", 5);
		;
	}


}
PLUGINLIB_DECLARE_CLASS(stereo_camera, StereoDownsample, nodelet_stereo_camera::StereoDownsample, nodelet::Nodelet);
}


