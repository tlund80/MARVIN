#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"

#include "calibration_verify/stereo_chessboard_detector.h"
#include <opencv2/opencv.hpp>
//calibration
#include <Mathematics/StereoCalibration.h>
#include <caros/CovisRos.hpp>
#include <caros_common_msgs/StereoCalibration.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;
using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;
using Calibration::StereoCalibration;

typedef calibration_verify::stereo_chessboard_detector::Request ReqT;
typedef calibration_verify::stereo_chessboard_detector::Response ResT;

//vector<image_transport::SubscriberFilter *> subscribterVector;
//vector<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *> timeSync;
cv::Mat imgL;
cv::Mat imgR;
Calibration::StereoCalibration calib;

void stereoCallback(const sensor_msgs::ImageConstPtr& msg_left_image,
		const sensor_msgs::ImageConstPtr& msg_right_image,
		const caros_common_msgs::StereoCalibrationConstPtr& msg_calib_info) {

	cv_bridge::CvImagePtr leftImageCVPtr, rightImageCVPtr;

	try {
		leftImageCVPtr = cv_bridge::toCvCopy(msg_left_image,
				msg_left_image->encoding);
		rightImageCVPtr = cv_bridge::toCvCopy(msg_right_image,
				msg_right_image->encoding);

	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from encodings. %s", e.what());
	}
	imgL = leftImageCVPtr->image;
	imgR = rightImageCVPtr->image;
	calib = CovisRos::toCovis(*msg_calib_info);
}

bool locateChessboardCorners2D(const cv::Mat inputImage, const cv::Size boardsize, std::vector<cv::Point2f> & corners) {
   // try to find chessboard corners automatically
   if (!cv::findChessboardCorners(inputImage, boardsize, corners,
                                  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK)) {
      return false;
   }

   cv::cornerSubPix(inputImage, corners, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 300, 0.01));

   return true;
}

bool locateChessboardCorners3D(const cv::Mat leftImage, const cv::Mat rightImage, const cv::Size patternsize,
                               const StereoCalibration & calib, std::vector<KVector<> > & points3D,
                               cv::Mat & leftPatternImage, cv::Mat & rightPatternImage, cv::Mat & extremePointImage) {

   std::vector<cv::Point2f> cornersLeft;
   std::vector<cv::Point2f> cornersRight;

   cv::Mat leftImageGrey, rightImageGrey;

   cvtColor(leftImage, leftImageGrey, CV_BGR2GRAY);
   cvtColor(rightImage, rightImageGrey, CV_BGR2GRAY);

   if (!locateChessboardCorners2D(leftImageGrey, patternsize, cornersLeft)
         || !locateChessboardCorners2D(rightImageGrey, patternsize, cornersRight)) {
      std::cerr << "Could not detect chessboards automatically." << std::endl;
      return false;
   }

   leftPatternImage = leftImage.clone();
   rightPatternImage = rightImage.clone();
   extremePointImage = leftImage.clone();
   cv::drawChessboardCorners(leftPatternImage, patternsize, cv::Mat(cornersLeft), true);
   cv::drawChessboardCorners(rightPatternImage, patternsize, cv::Mat(cornersRight), true);

   cv::Mat corners3D;

   cv::triangulatePoints(calib.leftCamera().getProjectionMatrix().toCvMat(),
                         calib.rightCamera().getProjectionMatrix().toCvMat(), cornersLeft, cornersRight, corners3D);

   // Seems to be no good way to detect the mat type when accessing pixels
   // To make sure the float access below does not produce garbage we assert here
   assert((corners3D.type()& CV_MAT_DEPTH_MASK) == CV_32F);
   points3D.resize(corners3D.size().width, KVector<>(0.0, 0.0, 0.0, 0.0));
   for (int i = 0; i < corners3D.size().width; ++i) {
      for (int j = 0; j < 4; ++j) {
         points3D[i][j] = corners3D.at<float>(j, i);
      }
      points3D[i] = points3D[i].inhomog();
   }

   cv::circle(extremePointImage, cornersLeft[0], 50, cvScalar(0, 255, 255), 5);
   cv::circle(extremePointImage, cornersLeft[patternsize.width - 1], 50, cvScalar(255, 0, 0), 5);
   cv::circle(extremePointImage, cornersLeft[(patternsize.height - 1) * patternsize.width], 50, cvScalar(0, 255, 0), 5);
   cv::circle(extremePointImage, cornersLeft[(patternsize.height) * patternsize.width - 1], 50, cvScalar(0, 0, 255), 5);

   return true;
}

void toArray(KVector<>& P, boost::array<double, 3>& arr)
{
    arr[0] = P[0];
    arr[1] = P[1];
    arr[2] = P[2];
}


bool Grab_scene(ReqT &req, ResT &res)
{
   cv::Size patternsize(req.width, req.height); //interior number of corners
   //cv::Mat leftImage = cv::imread("left.png", 1);
   //cv::Mat rightImage = cv::imread("right.png", 1);
   cv::Mat leftImage = imgL;
   cv::Mat rightImage = imgR;
//    string work_dir = ros::package::getPath("calibration_verify");
//    
//    StereoCalibration calib = Calibration::loadStereoParametersFromOpenCV(work_dir + "/calib/bumblebeeRight.txt", false);

   if (leftImage.data == NULL || rightImage.data == NULL) {
      std::cerr << "Could not read images." << std::endl;
      return false;
   }

   std::vector<KVector<> > points3D;
   cv::Mat cornerImage;

   if (!locateChessboardCorners3D(leftImage, rightImage, patternsize, calib, points3D, leftImage, rightImage, cornerImage)) {
      std::cerr << "Problem detecting patterns." << std::endl;
      return false;
   }

   try {
      cv::imshow("leftPattern.png", leftImage);
      cv::imshow("rightPattern.png", rightImage);
      cv::imshow("cornerImage.png", cornerImage);
   } catch (...) {
      std::cerr << "Problem writing images!" << std::endl;
      return false;
   }

   std::cout << "Top    left  corner (yellow) is at: " << points3D[0] << std::endl;
   std::cout << "Top    right corner (blue)   is at: " << points3D[patternsize.width - 1] << std::endl;
   std::cout << "Bottom left  corner (green)  is at: " << points3D[(patternsize.height - 1) * patternsize.width] << std::endl;
   std::cout << "Bottom right corner (red)    is at: " << points3D[(patternsize.height) * patternsize.width - 1] << std::endl;
   
   toArray(points3D[0], res.left_top);
   toArray(points3D[patternsize.width - 1], res.right_top);
   toArray(points3D[(patternsize.height - 1) * patternsize.width], res.left_bottom);
   toArray(points3D[(patternsize.height) * patternsize.width - 1], res.right_bottom);
   
   cout<<"Grab Scene done!"<<"\n";
   return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_chessboard_detector");
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");
    image_transport::ImageTransport imageTransport(nodeHandle);
    
    std::string image_left, image_right, calib_rect;
    nodeHandle.getParam("image_left", image_left);
    nodeHandle.getParam("image_right", image_right);
    nodeHandle.getParam("calib_rect", calib_rect);
    
    if (image_left.empty()) ROS_FATAL("Please specify an intrinsic calibration file! use _image_left:=... ");
    if (image_right.empty()) ROS_FATAL("Please specify an intrinsic calibration file! use _image_right:=... ");
    if (calib_rect.empty()) ROS_FATAL("Please specify an intrinsic calibration file! use _calib_rect:=... ");
    
    image_transport::SubscriberFilter filter_left, filter_right;
    message_filters::Subscriber<caros_common_msgs::StereoCalibration> sub_calib_info;
    filter_left.subscribe(imageTransport, image_left, 1);
    filter_right.subscribe(imageTransport, image_right, 1);
    sub_calib_info.subscribe(nodeHandle, calib_rect, 1);

    typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, caros_common_msgs::StereoCalibration> StereoImageSyncMsgFilter;
    StereoImageSyncMsgFilter timeSync(1);
    timeSync.connectInput(filter_left, filter_right, sub_calib_info);
    timeSync.registerCallback( boost::bind(&stereoCallback, _1, _2, _3) );
    
    ros::ServiceServer service = nodeHandle.advertiseService("getcorner", Grab_scene);
    
    ros::spin();
    
    return 0;
}
