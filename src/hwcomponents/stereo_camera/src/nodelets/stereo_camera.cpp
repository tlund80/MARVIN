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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/transform_broadcaster.h>
#include <caros/CovisRos.hpp>
#include "Devices/Camera/StereoCamera.h"
#include "Devices/Camera/ContextManager.h"
#include "Mathematics/StereoCalibration.h"

using std::string;

namespace nodelet_stereo_camera
{

class StereoCamera : public nodelet::Nodelet
{
public:
  StereoCamera() :
      stereoCamera(NULL), running_(false),cameraSerialNumber(NULL),cameraSerialNumberL(NULL),cameraSerialNumberR(NULL)
  {

  }

  ~StereoCamera()
  {
    if (running_)
    {
      NODELET_INFO("shutting down driver thread");
      running_ = false;
      deviceThread_->join();
      NODELET_INFO("driver thread stopped");
    }
//    dvr_->shutdown();
  }

private:
  virtual void onInit();
  virtual void devicePoll();
  void loadParameters(ros::NodeHandle, const string);
  void connectCb();

  // Publications
  boost::mutex connect_mutex_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher leftImageRawPub;
  image_transport::CameraPublisher rightImageRawPub;
  ros::Publisher stereoCalibrationRawPub;

  tf::TransformBroadcaster br;
  tf::Transform transform;

// parameteres
  int* cameraSerialNumber; // BumbleBee Serial Number
  int* cameraSerialNumberL; // Pike Serial Number (left camera)
  int* cameraSerialNumberR; // Pike Serial Number (right camera)

  string calibrationRawFilePath;
  string cameraPostionFile;
  Calibration::StereoCalibration stereoCalibrationRaw;

  Camera::StereoCamera* stereoCamera;
  volatile bool running_; ///< device is running
  boost::shared_ptr<boost::thread> deviceThread_;

  sensor_msgs::CameraInfoPtr leftCamInfo;
  sensor_msgs::CameraInfoPtr rightCamInfo;

  std::string base_frame_name;
  std::string camera_frame_name;

  bool publish;
};
void StereoCamera::loadParameters(const ros::NodeHandle nh, const string camera_name)
{
  const std::string workingDir = ros::package::getPath("stereo_camera");
  if (nh.hasParam(camera_name))
  {
    ROS_INFO_STREAM( "reading parameters of "<< camera_name);
    string serial_path = camera_name;
    serial_path.append("/serial");
    string calibration_raw_file_path = camera_name;
    calibration_raw_file_path.append("/calibration_raw_file");
    string camera_postion_file_path = camera_name;
    camera_postion_file_path.append("/camera_postion_file");
    string serial_left_path = camera_name;
    serial_left_path.append("/serial_left");
    string serial_right_path = camera_name;
    serial_right_path.append("/serial_right");
    //      int serial;
    if (ros::param::has(serial_path))
    {
      cameraSerialNumber = new int();
      if (ros::param::get(serial_path, *cameraSerialNumber))
      {
        ROS_INFO_STREAM( camera_name<<"'s serial number is "<<*cameraSerialNumber);
      }
    }
    if (ros::param::has(serial_left_path))
    {
      cameraSerialNumberL = new int();
      if (ros::param::get(serial_left_path, *cameraSerialNumberL))
      {
        ROS_INFO_STREAM( camera_name<<"'s serial number (Left) is "<<*cameraSerialNumberL);
      }
    }
    if (ros::param::has(serial_right_path))
    {
      cameraSerialNumberR = new int();
      if (ros::param::get(serial_right_path, *cameraSerialNumberR))
      {
        ROS_INFO_STREAM( camera_name<<"'s serial number (Right) is "<<*cameraSerialNumberR);
      }
    }
    if (ros::param::get(calibration_raw_file_path, calibrationRawFilePath))
    {
      //        ROS_INFO_STREAM(calibrationRawFilePath);
    }
    if (ros::param::get(camera_postion_file_path, cameraPostionFile))
    {
      //        ROS_INFO_STREAM( camera_name<<"'s serial number is "<<*cameraSerialNumber);
    }
    //        ROS_INFO_STREAM( "NO ");
    calibrationRawFilePath = workingDir + "/" + calibrationRawFilePath;
    cameraPostionFile = workingDir + "/" + cameraPostionFile;
  }
  else
  {
    ROS_DEBUG_STREAM( "No parameters found for "<<camera_name);
  }
}
void StereoCamera::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  string nhgetNamespace = nh.getNamespace();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  string camera_name = private_nh.getNamespace();
  it_.reset(new image_transport::ImageTransport(nh));
  int  publisher_queue_size_=1;

//  ROS_INFO_STREAM("camera_name: "<<camera_name);

  // load parameters from files
  loadParameters(nh, camera_name);
  ROS_INFO_STREAM("camera parameters are loaded.");
  // Set up dynamic reconfiguration
//  ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCb,
//                                                  this, _1, _2);
//  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
//  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&StereoCamera::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
//  pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);}
  leftImageRawPub = it_->advertiseCamera("left/image_raw", publisher_queue_size_, connect_cb);
  rightImageRawPub = it_->advertiseCamera("right/image_raw", publisher_queue_size_, connect_cb);
  stereoCalibrationRawPub = nh.advertise<caros_common_msgs::StereoCalibration>("stereo_calib_raw", publisher_queue_size_);

  Camera::ContextManager* contextManager = new Camera::ContextManager();
  contextManager->init();
  if (cameraSerialNumber != NULL)
  {
    ROS_INFO_STREAM("trying to find bumblebee cameras ...");
    stereoCamera = contextManager->getStereoFromSerial(*cameraSerialNumber); // FOR BUMBLEBEE
    ROS_INFO_STREAM("allocated a Bumblebee2 stereo with serial number: "<<*cameraSerialNumber);

  }
  else if (cameraSerialNumberL != NULL && cameraSerialNumberR != NULL)
  {
    ROS_INFO_STREAM("trying to find pike cameras ...");
    stereoCamera = contextManager->getStereoFromSerial(*cameraSerialNumberL, *cameraSerialNumberR); // FOR PIKE CAMERAS
    ROS_INFO_STREAM(
        "allocated a Pike stereo with serial numbers: "<<*cameraSerialNumberL<<" (L) and "<<*cameraSerialNumberR<<" (R)");
  }
  else
  {
    ROS_INFO_STREAM("No camera parameters have been passed ...");
    abort();
    //TODO make possible to run the available camera
    //      ROS_DEBUG_STREAM("No camera parameters have been passed... trying to allocate the first available stereo");
    //      ROS_INFO_STREAM(contextManager->getAvailableGuids().size());
    //      stereoCamera = contextManager->getAvailableStereos().at(0); // getting the first available stereo on the firewire
    //      ROS_INFO_STREAM("allocated a"<<stereoCamera->);
    //      ROS_INFO_STREAM("No camera parameters have been passed and the first available stereo has been allocated");
    //     calibrationRawFilePath = "calibrationFiles/BumbleBee2-60-8511475_raw.txt"; // whatever-file for the program not to crash TODO revisit that
    //     cameraPostionFile = "calibrationFiles/BumbleBee2-60-8511475_position.txt"; // whatever-file for the program not to crash TODO revisit that
  }
  //
  if (!stereoCamera->init())
    ROS_DEBUG_STREAM("Could not initialize camera ");
  if (!stereoCamera->setupContinuousMode())
    ROS_DEBUG_STREAM("Could not setup continuous mode ");

  //Load the raw calibration information
  stereoCalibrationRaw = Calibration::loadStereoParametersFromOpenCV(calibrationRawFilePath.c_str());

  // TODO this needs a check if necessary (in the specific case) and utter a warning
  stereoCalibrationRaw.resetCoordinateSystem();

  //Load the position calibration information
  if (cameraPostionFile != "")
  {
    std::ifstream positionFileHandle;
    positionFileHandle.open(cameraPostionFile.c_str());
    ROS_ASSERT_MSG( positionFileHandle.is_open(), "Problem reading position file %s", cameraPostionFile.c_str());
    positionFileHandle >> base_frame_name;
    positionFileHandle >> camera_frame_name;
    KMatrix<double> kTransform(4, 4);
    positionFileHandle >> kTransform;
    transform.setOrigin(tf::Vector3(kTransform(0, 3), kTransform(1, 3), kTransform(2, 3)));
    KQuaternion rotQ = kTransform.rotationQuaternion();
    transform.setRotation(tf::Quaternion(rotQ[1], rotQ[2], rotQ[3], rotQ[0]));
    positionFileHandle.close();
  }
  else
  {
    camera_frame_name = camera_name;
    base_frame_name = "world";

    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    KMatrix<> identity(3, 3);
    identity.eye(1.0);
    KQuaternion rotQ = identity.rotationQuaternion();
    transform.setRotation(tf::Quaternion(rotQ[1], rotQ[2], rotQ[3], rotQ[0]));
  }

  std::pair<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> calibmsgs;
  double rectificsationScalingFactor = 0;
  calibmsgs = CovisRos::toRos(stereoCalibrationRaw, rectificsationScalingFactor);
//  leftCamInfo=calibmsgs.first;
  leftCamInfo = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(calibmsgs.first));
//  rightCamInfo=calibmsgs.second;
  rightCamInfo = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(calibmsgs.second));

  running_ = true;
  deviceThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&StereoCamera::devicePoll, this)));
  publish = false;
  //
  //  setCameraFeatureParameters();

}

void StereoCamera::devicePoll()
{

  uint64_t timestamp_cam;
  const int inputWidth = stereoCamera->getMaxImageWidth();
  const int inputHeight = stereoCamera->getMaxImageHeight();
  IplImage * imgInputL = cvCreateImage(cvSize(inputWidth, inputHeight), IPL_DEPTH_8U, 1);
  IplImage * imgInputR = cvCreateImage(cvSize(inputWidth, inputHeight), IPL_DEPTH_8U, 1);
  while (running_)
  {
    if (!stereoCamera->getRawImagePair(imgInputL, imgInputR, timestamp_cam, false))
    {
      ROS_DEBUG_STREAM("Unable to acquire image!");
      break;
    }

    ros::Time captureTime = ros::Time((double)timestamp_cam / 1000000.0);
    leftCamInfo->header.frame_id = camera_frame_name;

    rightCamInfo->header.frame_id = camera_frame_name;
    leftCamInfo->header.stamp = captureTime;
    rightCamInfo->header.stamp = captureTime;

    cv_bridge::CvImage imgInputLMatCV, imgInputRMatCV;

    imgInputLMatCV.encoding = "mono8";
    imgInputLMatCV.header.frame_id = camera_frame_name;
    imgInputLMatCV.header.stamp = captureTime;
    imgInputLMatCV.image = imgInputL;

    imgInputRMatCV.encoding = "mono8";
    imgInputRMatCV.header.frame_id = camera_frame_name;
    imgInputRMatCV.header.stamp = captureTime;
    imgInputRMatCV.image = imgInputR;

    caros_common_msgs::StereoCalibrationPtr stereoRawMsg(
        new caros_common_msgs::StereoCalibration(CovisRos::toOwnRos(stereoCalibrationRaw)));

    stereoRawMsg->header.frame_id = camera_frame_name;
    stereoRawMsg->header.stamp = captureTime;

    br.sendTransform(tf::StampedTransform(transform, captureTime, base_frame_name, camera_frame_name));

    if (publish)
    {
      leftImageRawPub.publish(imgInputLMatCV.toImageMsg(), leftCamInfo);
      rightImageRawPub.publish(imgInputRMatCV.toImageMsg(), rightCamInfo);
      stereoCalibrationRawPub.publish(stereoRawMsg);
    }
  }
}

void StereoCamera::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (leftImageRawPub.getNumSubscribers() == 0 && rightImageRawPub.getNumSubscribers() == 0
      && stereoCalibrationRawPub.getNumSubscribers() == 0)
  {

    ROS_INFO_STREAM("stereo nodelet publication shutdown");
    leftImageRawPub.shutdown();
    rightImageRawPub.shutdown();
    stereoCalibrationRawPub.shutdown();

  }
  else
  {
    ROS_INFO_STREAM("stereo nodelet publication is on");
    publish = true;
  }

}
PLUGINLIB_DECLARE_CLASS(stereo_camera, StereoCamera, nodelet_stereo_camera::StereoCamera, nodelet::Nodelet);
}

