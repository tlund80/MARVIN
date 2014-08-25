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

#include "Mathematics/StereoCalibration.h"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <caros/CovisRos.hpp>
#include <caros_common_msgs/StereoCalibration.h>
#include <caros_common_msgs/CapturePoints2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h> 

//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 1

using std::string;
using namespace message_filters;

typedef boost::shared_ptr<caros_common_msgs::StereoCalibration const> StereoCalibrationConstPtr;
namespace nodelet_stereo_camera
{

class StereoPointCloud : public nodelet::Nodelet
{
public:
  StereoPointCloud()
  {

  }

  ~StereoPointCloud()
  {

  }

private:
  virtual void onInit();

  void connectCb();

  // Publications
  boost::mutex connect_mutex_;
  boost::mutex process_mutex_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
//  image_transport::Publisher pub_left_image_rect;
//  image_transport::Publisher pub_right_image_rect;
//  ros::Publisher pub_stereo_calibration_rect;

  image_transport::SubscriberFilter sub_left_image_rect;
  image_transport::SubscriberFilter sub_right_image_rect;
  Calibration::StereoCalibration stereoCalibrationRect;

  message_filters::Subscriber<caros_common_msgs::StereoCalibration> sub_calib_rect;
  int subscriber_queue_size_;
  int publisher_queue_size_;

  void imageCb(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image,
               const caros_common_msgs::StereoCalibrationConstPtr& msg_calib_rect);
  bool toPointsCB(caros_common_msgs::CapturePoints2::Request &req, caros_common_msgs::CapturePoints2::Response &res);

  bool toPoints();

  bool subscribeToImages();

  pcl::PointCloud<pcl::PointXYZRGB> toPCLpointcloud(const cv::Mat& mat, const cv::Mat& rgb);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
      caros_common_msgs::StereoCalibration> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image,
      caros_common_msgs::StereoCalibration> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  boost::shared_ptr<ExactSync> exact_sync_;

//  boost::shared_ptr<PrepAndEarlyVisionCV> preprocessAndFilter;

//  IplImage * imgColorL;
//  IplImage * imgColorR;

//  IplImage * imgGreyFloatDummyL;
//  IplImage * imgGreyFloatDummyR;

//  IplImage * imgGreyL;
//  IplImage * imgGreyR;

//  Calibration::StereoCalibration* stereoCalibrationRaw;
//  Calibration::StereoCalibration stereoCalibrationRaw_;
  cv::Mat imgL;
  cv::Mat imgR;
  int numberOfDisparities;
  int SADWindowSize;
  boost::array<double, 9> left_camera_R; // leftCamra rotation to track change in calibration
  bool publishPoints;
  sensor_msgs::PointCloud2 scene;
  ros::Time stamp;
  std::string frame_id;

  ros::Publisher pointsPub;

  // for calibration data
  cv::Rect roi1, roi2;
//  cv::Mat_<double> M1, D1, M2, D2, R, T, R1, P1, R2, P2;
//  cv::Mat_<double> M1, M2, T, R1;
  ros::ServiceServer service;
//  int count;

  image_transport::Publisher pub_dum;
};

void StereoPointCloud::onInit()
{
  ROS_DEBUG_STREAM(" StereoPointCloud::onInit()");
  ros::NodeHandle &nh = getNodeHandle();
//  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // optional parameters
  nh.param("max_disparities", numberOfDisparities, 192); // should be dividable with 16
  nh.param("block_size", SADWindowSize, 11); // should be odd number between 3 and 11

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  subscriber_queue_size_ = 5;
  publisher_queue_size_ = 1;
//  sync_.reset(new Sync(sub_left_image_raw, sub_right_image_raw, sub_calib_raw, subscriber_queue_size_));
//  sync_->registerCallback(boost::bind(&StereoPreprocess::imageCb, this, _1, _2, _3));
//  sync_.reset(new Sync(sub_left_image_raw, sub_right_image_raw, subscriber_queue_size_));
  //Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.

  bool approx = false;
  if (approx)
  {
    approximate_sync_.reset(
        new ApproximateSync(ApproximatePolicy(subscriber_queue_size_), sub_left_image_rect, sub_right_image_rect,
                            sub_calib_rect));
    approximate_sync_->registerCallback(boost::bind(&StereoPointCloud::imageCb, this, _1, _2, _3));
  }
  else
  {
    exact_sync_.reset(
        new ExactSync(ExactPolicy(subscriber_queue_size_), sub_left_image_rect, sub_right_image_rect, sub_calib_rect));
    exact_sync_->registerCallback(boost::bind(&StereoPointCloud::imageCb, this, _1, _2, _3));
  }

  // Set up dynamic reconfiguration
//  ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCb,
//                                                  this, _1, _2);
//  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
//  reconfigure_server_->setCallback(f);

//  count=0;

  ros::SubscriberStatusCallback connect_cb = boost::bind(&StereoPointCloud::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

//  pub_dum=it_->advertise("dum", 5, connect_cb);
  service = nh.advertiseService("grab_point_cloud", &StereoPointCloud::toPointsCB, this);
  pointsPub = nh.advertise<sensor_msgs::PointCloud2>("points", 1, connect_cb);

//  image_transport::TransportHints hints("rect_color", ros::TransportHints(), getPrivateNodeHandle());
//  sub_left_image_rect.subscribe(*it_, "left/image_rect_color", 5);
//  sub_right_image_rect.subscribe(*it_, "right/image_rect_color", 5);
//  sub_calib_rect.subscribe(nh, "stereo_calib_rect", 5);

}

void StereoPointCloud::imageCb(const sensor_msgs::ImageConstPtr& left_image,
                               const sensor_msgs::ImageConstPtr& right_image,
                               const caros_common_msgs::StereoCalibrationConstPtr& calib)
{
  ROS_DEBUG_STREAM("StereoPointCloud::imageCb");

  cv_bridge::CvImagePtr leftImageCVPtr, rightImageCVPtr;
  try
  {
    leftImageCVPtr = cv_bridge::toCvCopy(left_image, left_image->encoding);
    rightImageCVPtr = cv_bridge::toCvCopy(right_image, left_image->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from encodings. %s", e.what());
  }

  if (calib->left.R != left_camera_R)
  {
    left_camera_R = calib->left.R;
    stereoCalibrationRect = CovisRos::toCovis(*calib);
//    M1 = stereoCalibrationRect.leftCamera().getIntrinsic().toCvMat();
//    M2 = stereoCalibrationRect.rightCamera().getIntrinsic().toCvMat();
//    T = stereoCalibrationRect.rightCamera().getTranslation();

  }
  imgL = leftImageCVPtr->image;
  imgR = rightImageCVPtr->image;
  stamp = left_image->header.stamp;
  frame_id = left_image->header.frame_id;
  toPoints();
  if (publishPoints)
  {
//    scene.header=left_image->header;
    pointsPub.publish(scene);
  }

}
bool StereoPointCloud::toPointsCB(caros_common_msgs::CapturePoints2::Request &req,
                                  caros_common_msgs::CapturePoints2::Response &res)
{
  ROS_INFO("Get point cloud called");
  subscribeToImages();
  numberOfDisparities = req.max_disparity;
  SADWindowSize = req.blocksize;

  bool resLocal = toPoints();
  if (resLocal)
    res.outputScene = scene;
  return resLocal;
}

bool StereoPointCloud::toPoints()
{

//  ROS_INFO("StereoPointCloud::toPoints()");
  cv::Mat imgL_grey;
  cv::Mat imgR_grey;
  cvtColor(imgL, imgL_grey, CV_RGB2GRAY);
  cvtColor(imgR, imgR_grey, CV_RGB2GRAY);

  cv::StereoBM bm;
  bm.state->roi1 = roi1; //TODO: this was never assigned
  bm.state->roi2 = roi2; //TODO: this was never assigned
  bm.state->preFilterCap = 31;
  bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
  bm.state->minDisparity = 0;
  bm.state->numberOfDisparities = numberOfDisparities;
  bm.state->textureThreshold = 10;
  bm.state->uniquenessRatio = 10;
  bm.state->speckleWindowSize = 100;
  bm.state->speckleRange = 32;
  bm.state->disp12MaxDiff = -1;

  cv::Mat disp, disp8;
  bm(imgL_grey, imgR_grey, disp);

  //disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
  //imwrite("temp_disparity.png", disp8);

  // Get parameters for reconstruction
  cv::Mat_<double> M1 = stereoCalibrationRect.leftCamera().getIntrinsic().toCvMat();
  cv::Mat_<double> M2 = stereoCalibrationRect.rightCamera().getIntrinsic().toCvMat();
  float T = stereoCalibrationRect.rightCamera().getTranslation()[0];

  float f = M1(0, 0); // Focal length
  float B = T; // Baseline in the x direction
  float cx = M1(0, 2); // Center x coordinate
  float cy = M1(1, 2); // Center y coordinate
  float cx2 = M2(0, 2); // Center x coordinate of right image
  float dcx = cx - cx2; // Difference in center x coordinates
  int temp = disp.at<int16_t>(0, 0);
  for (int y = 0; y < disp.rows; ++y)
  {
    for (int x = 0; x < disp.cols; ++x)
    {
      if (temp > disp.at<int16_t>(y, x))
        temp = disp.at<int16_t>(y, x);
    }
  }

  cv::Mat_<cv::Vec3f> xyz(disp.rows, disp.cols, cv::Vec3f(0, 0, 0)); // Resulting point cloud, initialized to zero
  for (int y = 0; y < disp.rows; ++y)
  {
    for (int x = 0; x < disp.cols; ++x)
    {
      cv::Vec3f& p = xyz(y, x); // Point to write to

      // Avoid invalid disparities
      if (disp.at<int16_t>(y, x) == temp)
        continue;
      if (disp.at<int16_t>(y, x) == 0)
        continue;

      // Maths from here: http://answers.opencv.org/upfiles/13535653931527438.jpg
      float d = float(disp.at<int16_t>(y, x)) / 16.0f; // Disparity
      float W = B / (-d + dcx); // Weighting

      p[0] = (float(x) - cx) * W;
      p[1] = (float(y) - cy) * W;
      p[2] = f * W;
    }
  }
  
//  ros::Time stamp = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZRGB> cloud = toPCLpointcloud(xyz, imgL);
 // pcl::PointCloud2 cloud2; 

  pcl::toROSMsg<pcl::PointXYZRGB>(cloud, scene);
  scene.header.stamp = stamp;
  scene.header.frame_id = frame_id;
  return true;
}

pcl::PointCloud<pcl::PointXYZRGB> StereoPointCloud::toPCLpointcloud(const cv::Mat& mat, const cv::Mat& rgb)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud(mat.cols,mat.rows,pcl::PointXYZRGB(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));
  cloud.reserve(mat.cols * mat.rows);

  const double max_z = 2e3; // Disregard points farther away than 2 m
  //FILE* fp = fopen(filename, "wt");
  int counter=-1;
  for (int y = 0; y < mat.rows; y++)
  {
    for (int x = 0; x < mat.cols; x++)
    {
      counter++;
      cv::Vec3f point = mat.at<cv::Vec3f>(y, x);

      // This omits zero points
      if (point[0] == 0 && point[1] == 0 && point[2] == 0)
        continue;

      // This omits points equal to or larger than max_z
      if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
        continue;
      //fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);

      // Point to write to
      pcl::PointXYZRGB p;

      // Scale position from mm to m
      p.x = 0.001 * point[0];
      p.y = 0.001 * point[1];
      p.z = 0.001 * point[2];

      // OpenCV reads in images in BGR order, so we must switch to BGR for PCL
      cv::Vec3b pbgr = rgb.at<cv::Vec3b>(y, x);
      p.b = pbgr[0];
      p.g = pbgr[1];
      p.r = pbgr[2];


//      cloud.push_back(p);
      cloud.at(counter)=p;
    }
  }
  return cloud;
}

void StereoPointCloud::connectCb()
{
//  ROS_INFO_STREAM("StereoPointCloud::connectCb()..");
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  if (pointsPub.getNumSubscribers() == 0)
  {

    ROS_INFO_STREAM("publishing point clouds is off");
    publishPoints = false;
//    sub_left_image_rect.unsubscribe();
//    sub_right_image_rect.unsubscribe();
//    sub_calib_rect.unsubscribe();

  }
  else if (!sub_left_image_rect.getSubscriber())
  {
//    ros::NodeHandle &nh = getNodeHandle();
    ROS_INFO_STREAM("publishing point clouds is on");
    publishPoints = true;
    subscribeToImages();
//    image_transport::TransportHints hints("rect_color", ros::TransportHints(), getPrivateNodeHandle());
//    sub_left_image_rect.subscribe(*it_, "left/image_rect_color", 5);
//    sub_right_image_rect.subscribe(*it_, "right/image_rect_color", 5);
//    sub_calib_rect.subscribe(nh, "stereo_calib_rect", 5);
  }
}
bool StereoPointCloud::subscribeToImages()
{
  ros::NodeHandle &nh = getNodeHandle();
  image_transport::TransportHints hints("rect_color", ros::TransportHints(), getPrivateNodeHandle());
  sub_left_image_rect.subscribe(*it_, "left/image_rect_color", 5);
  sub_right_image_rect.subscribe(*it_, "right/image_rect_color", 5);
  sub_calib_rect.subscribe(nh, "stereo_calib_rect", 5);
  return true;
}

PLUGINLIB_DECLARE_CLASS(stereo_camera, StereoPointCloud, nodelet_stereo_camera::StereoPointCloud, nodelet::Nodelet);
}

