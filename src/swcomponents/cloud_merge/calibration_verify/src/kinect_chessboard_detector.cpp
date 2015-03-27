/**
 * From an aligned RGB-D image containing a chessboard,
 * find the 4 outer chessboard in the RGB image,
 * and compute their 3D positions
 * 
 * @param rgb RGB image
 * @param depth aligned depth image, assumed given in [mm]
 * @param intrinsics common intrinsics used for 3D reconstruction, assumed given in [mm]
 * @param numCornersHorizontal number of horizontal chessboard corners to detect
 * @param numCornersVertical number of vertical chessboard corners to detect
 * @param winSize window size in the depth image used for smoothing
 * @return if operation succeeded, a vector with 4 entries of reconstructed corners in [mm], otherwise an empty vector
 */

// STL
#include <iostream>
#include "ros/ros.h"

// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include "calibration_verify/kinect_chessboard_detector.h"

//grab image from kinect
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>


typedef calibration_verify::kinect_chessboard_detector::Request ReqT;
typedef calibration_verify::kinect_chessboard_detector::Response ResT;

using namespace std;
using namespace cv;
using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;

// image_transport::SubscriberFilter sub_image_rgb, sub_image_depth;
// message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info;

cv::Mat_<cv::Vec3b> rgb;
cv::Mat_<ushort> depth;
cv::Mat_<float> intrinsics;
int numCornersHorizontal;
int numCornersVertical;
int winSize = 5;

cv::Mat temp_rgb;
cv::Mat temp_depth;


std::vector<cv::Vec3f> find3DCorners(
      const cv::Mat_<cv::Vec3b>& rgb, const cv::Mat_<ushort>& depth,
      const cv::Mat_<float>& intrinsics,
      int numCornersHorizontal, int numCornersVertical,
      int winSize = 5) {
   /*
    * Sanity checks
    */
   if(rgb.empty() || depth.empty()) {
      std::cerr << "Empty RGB or depth image!" << std::endl;
      return std::vector<cv::Vec3f>();
   }
   
   if(rgb.rows != depth.rows || rgb.cols != depth.cols) {
      std::cerr << "RGB/depth image dimension mismatch!" << std::endl;
      return std::vector<cv::Vec3f>();
   }
   
   if(intrinsics.rows != 3 || intrinsics.cols != 3) {
      std::cerr << "Invalid intrinsics (must be 3x3):" << std::endl << intrinsics << std::endl;
      return std::vector<cv::Vec3f>();
   }
   
   if(numCornersHorizontal < 1 || numCornersVertical < 1) { 
      std::cerr << "Number of chessboard corners must be >= 1!" << std::endl;
      return std::vector<cv::Vec3f>();
   }
   
   if(winSize < 0) { 
      std::cerr << "Window size must be >= 0!" << std::endl;
      return std::vector<cv::Vec3f>();
   }
   
   /*
    * 1) Find all chessboard corners in RGB image
    */
   std::vector<cv::Vec2f> cornersAll;
   
   const bool found = cv::findChessboardCorners(
         rgb,
         cv::Size(numCornersHorizontal,numCornersVertical),
         cornersAll,
         cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK
         );
   
   if(!found) {
      std::cerr << "Chessboard corners not found in RGB image!" << std::endl;
      return std::vector<cv::Vec3f>();
   }
   
   // Take out the 4 outer corners of interest
   std::vector<cv::Vec2f> corners(4);
   corners[0] = cornersAll[0];
   corners[1] = cornersAll[numCornersHorizontal-1];
   corners[2] = cornersAll[numCornersHorizontal*numCornersVertical-numCornersHorizontal];
   corners[3] = cornersAll[numCornersHorizontal*numCornersVertical-1];
   
   /*
    * 2) Take the mean depth value around the 4 corners
    */
   std::vector<float> meanDepths(4);
   
   // Loop over corners
   for(int i = 0; i < 4; ++i) {
      // Round the corner coordinates
      const cv::Vec2f& c = corners[i];
      const int x(c[0] + 0.5f);
      const int y(c[1] + 0.5f);
      // Get start coordinates of window around the corner
      const int xx = std::max(0,x-winSize);
      const int yy = std::max(0,y-winSize);
      // Get maximum allowed size (do not exceed image border)
      int winSizeX = 2*winSize;
      if(xx+winSizeX > depth.cols)
        winSizeX -= xx + winSizeX - depth.cols;
      int winSizeY = 2*winSize;
      if(yy+winSizeY > depth.rows)
        winSizeY -= yy + winSizeY - depth.rows;
      cv::Rect win(
            xx, yy,
            winSizeX, winSizeY
            );
      // Take out the depth values of the window
      cv::Mat_<ushort> window = depth(win);
      // Convert to float
      cv::Mat_<float> winf(window);
      // Take the mean of all depths not 0 or NaN
      meanDepths[i] = 0.0f;
      int numValids = 0;
      for(cv::Mat_<float>::const_iterator it = winf.begin(); it != winf.end(); ++it) {
         if (std::isnan(*it))
            continue;
         
         if (*it > 0.0f) {
            meanDepths[i] += *it;
            ++numValids;
         }
      }
      // If depth map is too corrupted, we find zero valids here
      if(numValids == 0) {
         std::cerr << "Unable to find valid depths around corner " << i+1 << "/4!" << std::endl;
         return std::vector<cv::Vec3f>();
      }
      meanDepths[i] /= float(numValids);
   }
   
   /*
    * 3) Finally reconstruct the four depth values using calibration
    */
   std::vector<cv::Vec3f> result(4);
   
   const float fx = intrinsics(0,0);
   const float cx = intrinsics(0,2);
   const float fy = intrinsics(1,1);
   const float cy = intrinsics(1,2);
   for(int i = 0; i < 4; ++i) {
      result[i][0] = (corners[i][0] - cx) * meanDepths[i] / fx;
      result[i][1] = (corners[i][1] - cy) * meanDepths[i] / fy;
      result[i][2] = meanDepths[i];
   }
   
   return result;
}

void boostarrayTintrinsics(cv::Mat_<float>& intrinsics, boost::array<double,9>& array)
{
    intrinsics(0,0) = float(array[0]);
    intrinsics(0,1) = float(array[1]);
    intrinsics(0,2) = float(array[2]);
    intrinsics(1,0) = float(array[3]);
    intrinsics(1,1) = float(array[4]);
    intrinsics(1,2) = float(array[5]);
    intrinsics(2,0) = float(array[6]);
    intrinsics(2,1) = float(array[7]);
    intrinsics(2,2) = float(array[8]);
}

void vec3fTboostarray(cv::Vec3f& P, boost::array<double, 3>& arr)
{
    arr[0] = P[0];
    arr[1] = P[1];
    arr[2] = P[2];
}

void kinectCallback(const sensor_msgs::ImageConstPtr& msg_rgb_image, 
		    const sensor_msgs::ImageConstPtr& msg_depth_image)
{
  //ROS_INFO("Called kinect callback.");
  cv_bridge::CvImagePtr rgbImageCVPtr, depthImageCVPtr;

  try
  {
    rgbImageCVPtr = cv_bridge::toCvCopy(msg_rgb_image, msg_rgb_image->encoding);
    depthImageCVPtr = cv_bridge::toCvCopy(msg_depth_image, msg_depth_image->encoding);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from encodings. %s", e.what());
  }

  temp_rgb = rgbImageCVPtr->image;
  temp_depth = depthImageCVPtr->image;
  
  cv::Mat_<cv::Vec3b> _rgb(temp_rgb.rows, temp_rgb.cols, cv::Vec3b(0, 0, 0));
  cv::Mat_<ushort> _depth(temp_depth.rows, temp_depth.cols);
  
  
  for (int y = 0; y < temp_rgb.rows; ++y)
  {
    for (int x = 0; x < temp_rgb.cols; ++x)
    {
	Vec3b pbgr = temp_rgb.at<Vec3b>(y,x);
	_rgb.at<Vec3b>(y,x) = pbgr;
    }
  }
  
  for (int y = 0; y < temp_depth.rows; ++y)
  {
    for (int x = 0; x < temp_depth.cols; ++x)
    {
	ushort& p = _depth(y,x);
	p = ushort(temp_depth.at<int16_t>(y,x));
    }
  }
  
  rgb = _rgb;
  depth = _depth;
}

bool CornerDetectionCB(ReqT &req, ResT &res) 
{    
    numCornersHorizontal = req.width;
    numCornersVertical = req.height;
    
    cout<<"RGB image size: "<< rgb.size() <<endl;
    cout<<"Detpth image size : "<< depth.size() <<endl;
    
    std::vector<cv::Vec3f> result = find3DCorners(rgb, depth, intrinsics, numCornersHorizontal, numCornersVertical, winSize);
    
    if(result.empty()) {
       std::cerr << "Corner extraction failed!" << std::endl;
       return false;
    }
    
    std::cout << "Top    left  corner is at: " << result[0] << std::endl;
    std::cout << "Top    right corner is at: " << result[1] << std::endl;
    std::cout << "Bottom left  corner is at: " << result[2] << std::endl;
    std::cout << "Bottom right corner is at: " << result[3] << std::endl;
    
    vec3fTboostarray(result[0], res.left_top);
    vec3fTboostarray(result[1], res.right_top);
    vec3fTboostarray(result[2], res.left_bottom);
    vec3fTboostarray(result[3], res.right_bottom);
    
    return true;
}

int main(int argc, char** argv) {
   
    ros::init(argc, argv, "kinect_chessboard_detector");
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");
    image_transport::ImageTransport imageTransport(nodeHandle);
    
    std::string image_rgb, image_depth, camera_info;
    nodeHandle.getParam("image_rgb", image_rgb);
    nodeHandle.getParam("image_depth", image_depth);
    nodeHandle.getParam("camera_info", camera_info);
    
    if (image_rgb.empty()) ROS_FATAL("Please specify an intrinsic calibration file! use _image_rgb:=... ");
    if (image_depth.empty()) ROS_FATAL("Please specify an intrinsic calibration file! use _image_depth:=... ");
    if (camera_info.empty()) ROS_FATAL("Please specify an intrinsic calibration file! use _camera_info:=... ");
    
    image_transport::SubscriberFilter sub_image_rgb, sub_image_depth;
//     message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info;
    sub_image_rgb.subscribe(imageTransport, image_rgb, 1);
    sub_image_depth.subscribe(imageTransport, image_depth, 1);
    //sub_camera_info.subscribe(nodeHandle, camera_info, 1);
    
    //ros::Subscriber sub_camera_info = nodeHandle.subscribe<sensor_msgs::CameraInfo>(camera_info, 10);
    sensor_msgs::CameraInfo::ConstPtr msg_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info,ros::Duration(5.0));
    boost::array<double, 9> temp_K = msg_camera_info->K;
    cv::Mat_<float> temp_intrinsics(3,3);
    boostarrayTintrinsics(temp_intrinsics, temp_K);
    intrinsics = temp_intrinsics;
     //~ typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> KinectImageSyncMsgFilter;
     //~ KinectImageSyncMsgFilter timeSync(1);
     //~ timeSync.connectInput(sub_image_rgb, sub_image_depth);
     //~ timeSync.registerCallback( boost::bind(&kinectCallback, _1, _2) );   //TODO remember this mehod is not working for kinect.
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyT;
    message_filters::Synchronizer<SyncPolicyT> timeSync(SyncPolicyT(5), sub_image_rgb, sub_image_depth);
    timeSync.registerCallback(boost::bind(&kinectCallback, _1, _2));
    //timeSync.registerCallback(kinectCallback); also works
    
    ros::ServiceServer service = nodeHandle.advertiseService("getcorner", CornerDetectionCB);
   
    ros::spin();
   
    return 0;
}
