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
 \author Wail Mustafa
 \file stereoCamera.cpp
 \brief
 */

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
#include <caros_common_msgs/StereoCalibration.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::string;

typedef boost::shared_ptr<caros_common_msgs::StereoCalibration const> StereoCalibrationConstPtr;
namespace nodelet_stereo_camera
{

class StereoPreprocess : public nodelet::Nodelet
{
public:
  StereoPreprocess()
  {

  }

  ~StereoPreprocess()
  {

    preprocessAndFilter.reset();
  }

private:
  virtual void onInit();

  void connectCb();

  // Publications
  boost::mutex connect_mutex_;
  boost::mutex process_mutex_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher pub_left_image_rect;
  image_transport::Publisher pub_right_image_rect;
  ros::Publisher pub_stereo_calibration_rect;

  image_transport::SubscriberFilter sub_left_image_raw;

  image_transport::SubscriberFilter sub_right_image_raw;
  message_filters::Subscriber<caros_common_msgs::StereoCalibration> sub_calib_raw;
  int subscriber_queue_size_;
  int publisher_queue_size_;

  void imageCb(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image,
               const caros_common_msgs::StereoCalibrationConstPtr& msg_calib_raw);
//  void imageCb(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image);

//sensor_msgs::CameraInfoPtr leftCamInfo;
//sensor_msgs::CameraInfoPtr rightCamInfo;
//  typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, caros_common_msgs::StereoCalibration> Sync;
//  boost::shared_ptr<Sync> sync_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
      caros_common_msgs::StereoCalibration> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image,
      caros_common_msgs::StereoCalibration> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  boost::shared_ptr<ExactSync> exact_sync_;

  boost::shared_ptr<PrepAndEarlyVisionCV> preprocessAndFilter;

  IplImage * imgColorL;
  IplImage * imgColorR;

  IplImage * imgGreyFloatDummyL;
  IplImage * imgGreyFloatDummyR;

  IplImage * imgGreyL;
  IplImage * imgGreyR;

//  Calibration::StereoCalibration* stereoCalibrationRaw;
//  Calibration::StereoCalibration stereoCalibrationRaw_;
  boost::array<double, 9> left_camera_R; // leftCamra rotation to track change in calibration
//  int count;
};

void StereoPreprocess::onInit()
{
//  ROS_INFO_STREAM(" StereoPreprocess::onInit()");
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  subscriber_queue_size_ = 5;
  publisher_queue_size_=1;
//  sync_.reset(new Sync(sub_left_image_raw, sub_right_image_raw, sub_calib_raw, subscriber_queue_size_));
//  sync_->registerCallback(boost::bind(&StereoPreprocess::imageCb, this, _1, _2, _3));
//  sync_.reset(new Sync(sub_left_image_raw, sub_right_image_raw, subscriber_queue_size_));
  //Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.

  bool approx = false;
  if (approx)
  {
    approximate_sync_.reset(
        new ApproximateSync(ApproximatePolicy(subscriber_queue_size_), sub_left_image_raw, sub_right_image_raw, sub_calib_raw));
    approximate_sync_->registerCallback(boost::bind(&StereoPreprocess::imageCb, this, _1, _2, _3));
  }
  else
  {
    exact_sync_.reset(new ExactSync(ExactPolicy(subscriber_queue_size_), sub_left_image_raw, sub_right_image_raw, sub_calib_raw));
    exact_sync_->registerCallback(boost::bind(&StereoPreprocess::imageCb, this, _1, _2, _3));
  }

  // Set up dynamic reconfiguration
//  ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCb,
//                                                  this, _1, _2);
//  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
//  reconfigure_server_->setCallback(f);

//  count=0;

  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&StereoPreprocess::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  pub_left_image_rect = it_->advertise("left/image_rect_color", publisher_queue_size_, connect_cb);
//  pub_right_image_rect=it_->advertiseCamera("right/image_rect_color", 1,connect_cb);
  pub_right_image_rect = it_->advertise("right/image_rect_color", publisher_queue_size_, connect_cb);
  pub_stereo_calibration_rect = nh.advertise<caros_common_msgs::StereoCalibration>("stereo_calib_rect", publisher_queue_size_);

}

void StereoPreprocess::imageCb(const sensor_msgs::ImageConstPtr& left_image_raw,
                               const sensor_msgs::ImageConstPtr& right_image_raw,
                               const caros_common_msgs::StereoCalibrationConstPtr& msg_calib_raw)
//void StereoPreprocess::imageCb(const sensor_msgs::ImageConstPtr& left_image_raw,
//                               const sensor_msgs::ImageConstPtr& right_image_raw)
{

  cv_bridge::CvImagePtr leftImageCVPtr, rightImageCVPtr;
  try
  {
    leftImageCVPtr = cv_bridge::toCvCopy(left_image_raw, "mono8");
    rightImageCVPtr = cv_bridge::toCvCopy(right_image_raw, "mono8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from encodings. %s", e.what());
  }

  IplImage leftIplImage = leftImageCVPtr->image;
  IplImage rightIplImage = rightImageCVPtr->image;
  ;

  const int width = left_image_raw->width;
  const int height = left_image_raw->height;

  const int downScaleFactor = 1;

  const bool rotate180 = false;

  imageType bayerType = PREP_BAYER_GRBG;
  double rectificsationScalingFactor = 0;
  //Set gain values for bayer demosaic
  const float rG = 1.0, gG = 1.0, bG = 1.0;

  IplImage * imgColorL = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  IplImage * imgColorR = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

  IplImage * imgGreyFloatDummyL = cvCreateImage(cvSize(width, height), IPL_DEPTH_64F, 1);
  IplImage * imgGreyFloatDummyR = cvCreateImage(cvSize(width, height), IPL_DEPTH_64F, 1);

  IplImage * imgGreyL = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  IplImage * imgGreyR = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

  if (preprocessAndFilter == NULL)
    preprocessAndFilter = boost::shared_ptr<PrepAndEarlyVisionCV>(
        new PrepAndEarlyVisionCV(width, height, downScaleFactor, rotate180, bayerType, 0, 0, MONOGENIC_FREQ0110, true));
//  if (count==0) {
  if (msg_calib_raw->left.R != left_camera_R)
  {
    left_camera_R = msg_calib_raw->left.R;
    Calibration::StereoCalibration stereoCalibrationRaw = CovisRos::toCovis(*msg_calib_raw);
    preprocessAndFilter->setStereoCalibration(stereoCalibrationRaw, rectificsationScalingFactor);
  }

  preprocessAndFilter->setRawImages(&leftIplImage, &rightIplImage);

  preprocessAndFilter->preprocessImages(rG, gG, bG, false);

  preprocessAndFilter->getPreprocessedData(imgColorL, imgGreyFloatDummyL, imgColorR, imgGreyFloatDummyR);

  cvCvtColor(imgColorL, imgGreyL, CV_BGR2GRAY);
  cvCvtColor(imgColorR, imgGreyR, CV_BGR2GRAY);

  cv_bridge::CvImage imgColorLMatCV, imgColorRMatCV, imgGreyLMatCV, imgGreyRMatCV;

  imgColorLMatCV.encoding = "bgr8";
  imgColorLMatCV.header.frame_id = left_image_raw->header.frame_id;
  imgColorLMatCV.header.stamp = left_image_raw->header.stamp;
  imgColorLMatCV.image = imgColorL;

  imgColorRMatCV.encoding = "bgr8";
  imgColorRMatCV.header.frame_id = right_image_raw->header.frame_id;
  imgColorRMatCV.header.stamp = right_image_raw->header.stamp;
  imgColorRMatCV.image = imgColorR;

  imgGreyLMatCV.encoding = "mono8";
  imgGreyLMatCV.header.frame_id = left_image_raw->header.frame_id;
  imgGreyLMatCV.header.stamp = left_image_raw->header.stamp;
  imgGreyLMatCV.image = imgGreyL;

  imgGreyRMatCV.encoding = "mono8";
  imgGreyRMatCV.header.frame_id = right_image_raw->header.frame_id;
  imgGreyRMatCV.header.stamp = right_image_raw->header.stamp;
  imgGreyRMatCV.image = imgGreyR;

  Calibration::StereoCalibration stereoCalibrationRect = preprocessAndFilter->getCalibrationAfterPreprocessing();
  caros_common_msgs::StereoCalibrationPtr stereoRectMsg(
      new caros_common_msgs::StereoCalibration(CovisRos::toOwnRos(stereoCalibrationRect)));
  stereoRectMsg->header.frame_id = msg_calib_raw->header.frame_id;
  stereoRectMsg->header.stamp = msg_calib_raw->header.stamp;

  pub_left_image_rect.publish(imgColorLMatCV.toImageMsg());
  pub_right_image_rect.publish(imgColorRMatCV.toImageMsg());
  pub_stereo_calibration_rect.publish(stereoRectMsg);

  cvReleaseImage(&imgColorL);
  cvReleaseImage(&imgColorR);
  cvReleaseImage(&imgGreyFloatDummyL);
  cvReleaseImage(&imgGreyFloatDummyR);
  cvReleaseImage(&imgGreyL);
  cvReleaseImage(&imgGreyR);

}

void StereoPreprocess::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
//  ros::NodeHandle &nh = getNodeHandle();
//  ros::NodeHandle &private_nh = getPrivateNodeHandle();
//  ROS_INFO_STREAM("StereoPreprocess subscription is on");
////  image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
//  sub_left_image_raw.subscribe(*it_, "left/image_raw", 5);
//  sub_right_image_raw.subscribe(*it_, "right/image_raw", 5);
//  sub_calib_raw.subscribe(nh,"stereo_calib_raw", 5);
  if (pub_left_image_rect.getNumSubscribers() == 0 && pub_right_image_rect.getNumSubscribers() == 0
      && pub_stereo_calibration_rect.getNumSubscribers() == 0)
  {

    sub_left_image_raw.unsubscribe();
//   // sub_l_info_ .unsubscribe();
    sub_right_image_raw.unsubscribe();
    sub_calib_raw.unsubscribe();
//   // sub_r_info_ .unsubscribe();
  }
  else if (!sub_left_image_raw.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    ROS_INFO_STREAM("StereoPreprocess subscription is on");
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_left_image_raw.subscribe(*it_, "left/image_raw", 5, hints);
    sub_right_image_raw.subscribe(*it_, "right/image_raw", 5, hints);
    sub_calib_raw.subscribe(nh, "stereo_calib_raw", 5);
    ;
  }
}
PLUGINLIB_DECLARE_CLASS(stereo_camera, StereoPreprocess, nodelet_stereo_camera::StereoPreprocess, nodelet::Nodelet);
}

