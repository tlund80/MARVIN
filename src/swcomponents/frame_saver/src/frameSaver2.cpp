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
 \file frameSaver.cpp
 \brief
 */

#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>

#include <tr1/memory>
#include <opencv/highgui.h>

#include <sstream>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/norms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/common/common.h>


#include <pcl/ros/conversions.h>
//#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace tr1;
using namespace message_filters;
using namespace sensor_msgs;
// using namespace Modules;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;

int counter;
int objectID;
std::string folder;
std::string packagePath;

vector<image_transport::SubscriberFilter *> subscribterVector;

vector<ros::Subscriber*> point_cloud_vector;

image_transport::SubscriberFilter* sub_temp_1;
image_transport::SubscriberFilter* sub_temp_2;

vector<message_filters::TimeSynchronizer<Image, Image> *> timeSync;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyT;
vector<message_filters::Synchronizer<SyncPolicyT> *> timeSyncApprox;

void stereoSaver(image_transport::ImageTransport imageTransport, string name);
void kinectSaver(image_transport::ImageTransport imageTransport, string name);
void kinectPointCloudSaver(ros::NodeHandle, string name);

const int kinect_width = 640;
const int kinect_height = 480;

string KinectRawCalibrationFileName[3] = {"calibration_A00361800375049A.yml", "calibration_A00366802050045A.yml",
                                          "calibration_A00366920910047A.yml"};
string KinectPositionCalibrationFileName[3] = {"position_A00361800375049A.txt", "position_A00366802050045A.txt",
                                               "position_A00366920910047A.txt"};

void fromPointCloudToRGBDepth(pcl::PointCloud<pcl::PointXYZRGB> cloud, CameraCalibrationCV calib,cv::Mat_<cv::Vec3b>& rgb ,cv::Mat_<ushort>& depth) {



   // Projected RGB-D frame, using the RGB part (right) of the Kinect calibration
  int w = kinect_width;
  int h = kinect_height;


//     cv::Mat_<cv::Vec3b> rgb(h, w, cv::Vec3b(0, 0, 0));
//     cv::Mat_<ushort> depth(h, w, ushort(0));
   rgb= cv::Mat_<cv::Vec3b> (h, w, cv::Vec3b(0, 0, 0));
   depth= cv::Mat_<ushort>(h, w, ushort(0));

   // Get RGB part of the Kinect calibration
   const cv::Mat_<double> intrinsics = calib.getRight(w, h);
   const double& fx = intrinsics(0, 0);
   const double& fy = intrinsics(1, 1);
   const double& cx = intrinsics(0, 2);
   const double& cy = intrinsics(1, 2);

   // Generate projection
   for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = cloud.begin(); it != cloud.end(); ++it) {

      // 3D RGB point
      pcl::PointXYZRGB p = *it;

      // Pixel coordinates, rounded
      const int x(double(p.x) * fx / double(p.z) + cx + 0.5);
      const int y(double(p.y) * fy / double(p.z) + cy + 0.5);
      if (y >= 0 && y < h && x >= 0 && x < w) { // Projection valid, i.e. inside image borders
          uint32_t _rgb = *reinterpret_cast<int*>(&p.rgb);
          uint8_t r = (_rgb >> 16) & 0x0000ff;
          uint8_t g = (_rgb >> 8)  & 0x0000ff;
          uint8_t b = (_rgb)       & 0x0000ff;
         rgb(y, x) = cv::Vec3b(p.r,p.g, p.b); // RGB

         depth(y, x) = ushort(p.z*1000); // Depth
      }

   }

//   cv::imwrite("test.png",rgb);


}

void fromOrganizedPointCloudToRGBDepth(pcl::PointCloud<pcl::PointXYZRGB> cloud,cv::Mat_<cv::Vec3b>& rgb ,cv::Mat_<ushort>& depth) {


   rgb= cv::Mat_<cv::Vec3b> (cloud.height, cloud.width, cv::Vec3b(0, 0, 0));
   depth= cv::Mat_<ushort>(cloud.height, cloud.width, ushort(0));

   // Generate projection


   for(int h=0; h<cloud.height;h++) {
          for(int w=0; w<cloud.width;w++) {
              const pcl::PointXYZRGB& p = cloud.at(w,h);
              rgb(h, w) = cv::Vec3b(p.r,p.g, p.b); // RGB

              depth(h, w) = ushort(p.z*1000); // Depth

              }
           }

}


void stereoCallback(const sensor_msgs::ImageConstPtr& msg_left_image, const sensor_msgs::ImageConstPtr& msg_right_image)
{

  cv_bridge::CvImagePtr leftImageCVPtr, rightImageCVPtr;

  try
  {
    leftImageCVPtr = cv_bridge::toCvCopy(msg_left_image, msg_left_image->encoding);
    rightImageCVPtr = cv_bridge::toCvCopy(msg_right_image, msg_right_image->encoding);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from encodings. %s", e.what());
  }

  std::stringstream LeftPath;
  LeftPath << folder << "/" << msg_left_image.get()->header.frame_id << "_left_" << objectID << "_" << counter
      << ".png";

  std::stringstream RightPath;
  RightPath << folder << "/" << msg_right_image.get()->header.frame_id << "_right_" << objectID << "_" << counter
      << ".png";

  cv::imwrite(LeftPath.str(), leftImageCVPtr->image);
  cout << "Files saved: " << LeftPath.str() << endl;
  cv::imwrite(RightPath.str(), rightImageCVPtr->image);
  cout << "Files saved: " << RightPath.str() << endl;
}

void kinectCallback(const sensor_msgs::ImageConstPtr& msg_rgb_image, const sensor_msgs::ImageConstPtr& msg_depth_image)
{

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

  std::stringstream rgbPath;
  rgbPath << folder << msg_rgb_image.get()->header.frame_id << "_RGB_" << objectID << "_" << counter << ".png";

  std::stringstream depthPath;
  depthPath << folder << msg_depth_image.get()->header.frame_id << "_depth_" << objectID << "_" << counter << ".png";
//    std::stringstream depthPathYml;
//    depthPathYml <<folder<<msg_depth_image.get()->header.frame_id<<"_depth_"<<counter<< ".yml";

//   cv::Mat colorImage_BGR;
//   cv::cvtColor(rgbImageCVPtr->image, colorImage_BGR, CV_RGB2BGR);

//   cv::imwrite(rgbPath.str(),colorImage_BGR);
  cv::imwrite(rgbPath.str(), rgbImageCVPtr->image);
  cout << "Files saved: " << rgbPath.str() << endl;
  cv::imwrite(depthPath.str(), depthImageCVPtr->image);
  cout << "Files saved: " << depthPath.str() << endl;

//    cv::FileStorage fs;
//    fs.open(depthPathYml.str(), cv::FileStorage::WRITE);
//    fs << "data3D" << depthImageCVPtr->image;
//    fs.release();
//    depthPathYml.str("");
//    cout << "Files saved: " << depthPathYml.str() << endl;
  //cv::Mat test;
  //test=cv::imread(depthPath.str(),0);
  //cout << "format: " << test.type() << endl;
  //cv_bridge::CvImage cviDepthRGB;
  //cviDepthRGB.image=test;
  //cout << "image type: " <<  cv_bridge::getCvType(msg_depth_image->encoding) << endl;

}

void leftRight(const CloudT& cloud, CloudT& cloud_segmented) {
   pcl::PassThrough<PointT> filter;
   filter.setInputCloud(cloud.makeShared());
   filter.setFilterFieldName("x");
   filter.setFilterLimits(-0.3, 0.3); // TODO: Hard-coded
   filter.setFilterLimitsNegative(false);
   CloudT result;
   filter.filter(result);

   cloud_segmented = result;
}

void nearFar(const CloudT& cloud, CloudT& cloud_segmented) {
   pcl::PassThrough<PointT> filter;
   filter.setInputCloud(cloud.makeShared());
   filter.setFilterFieldName("z");
   filter.setFilterLimits(0.8, 1.2); // TODO: Hard-coded
   filter.setFilterLimitsNegative(false);
   CloudT result;
   filter.filter(result);

   cloud_segmented = result;
}

pcl::ModelCoefficients::ConstPtr tableRemoval(const CloudT& cloud, CloudT& cloud_segmented) {
   // Initialize stuff
   pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
   // Create the segmentation object
   pcl::SACSegmentation<PointT> seg;
   seg.setInputCloud(cloud.makeShared());
   // Optional
   seg.setMaxIterations(500); // TODO: Hard-coded
   seg.setOptimizeCoefficients(true);
   // Mandatory
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setDistanceThreshold (0.01); // TODO: Hard-coded
   // Segment out inlier indices
   seg.segment(*inliers, *coeff);
   pcl::ExtractIndices<PointT> extract;
   extract.setInputCloud(cloud.makeShared());
   extract.setIndices(inliers);
   extract.setNegative(true);
   CloudT result;
   extract.filter(result);
   cloud_segmented = result;

   return coeff;
}

void underTableRemoval(const CloudT& cloud, CloudT& cloud_segmented, pcl::ModelCoefficients::ConstPtr coeff) {
   // Project all points to the table plane
   pcl::ProjectInliers<PointT> proj;
   proj.setModelType(pcl::SACMODEL_PLANE);
   proj.setInputCloud(cloud.makeShared());
   proj.setModelCoefficients(coeff);
   CloudT projected;
   proj.filter(projected);
   // Store only points which are closer to view than their projected counterparts
   CloudT result;
   const int size = cloud.size();
   for(int i = 0; i < size; ++i) {
      const PointT& p = cloud[i];
      const PointT& pc = projected[i];
      if( (p.x*p.x+p.y*p.y+p.z*p.z) < (pc.x*pc.x+pc.y*pc.y+pc.z*pc.z) )
         result.push_back(p);
   }
   cloud_segmented = result;
}

void aboveTableRemoval(const CloudT& cloud, CloudT& cloud_segmented, pcl::ModelCoefficients::ConstPtr coeff) {
   // Project all points to the table plane
   pcl::ProjectInliers<PointT> proj;
   proj.setModelType(pcl::SACMODEL_PLANE);
   proj.setInputCloud(cloud.makeShared());
   proj.setModelCoefficients(coeff);
   CloudT projected;
   proj.filter(projected);
   // TODO: Store only points which are closer to table than hard-coded value
   const float disttolSquared = 0.5f*0.5f; // 50 cm
   const int size = cloud.size();
   CloudT result;
   for(int i = 0; i < size; ++i) {
      const PointT& p = cloud[i];
      const PointT& pc = projected[i];
      const float dx = pc.x - p.x;
      const float dy = pc.y - p.y;
      const float dz = pc.z - p.z;
      if( (dx*dx + dy*dy + dz*dz) < disttolSquared ) // Distance between original and projected point
         result.push_back(p);
   }
   cloud_segmented = result;
}

void outlierRemoval(const CloudT& cloud, CloudT& cloud_segmented) {
   pcl::StatisticalOutlierRemoval<PointT> filter;
   filter.setInputCloud(cloud.makeShared());
   filter.setMeanK(50); // TODO: Hard-coded
   filter.setStddevMulThresh(1.0); // TODO: Hard-coded
   CloudT result;
   filter.filter(result);
   cloud_segmented = result;
}


void cloudProcessing(const CloudT& cloud, CloudT& cloud_segmented) {

  ROS_DEBUG("\tPerforming near/far clipping...");
  int size = cloud_segmented.size();
  nearFar(cloud_segmented, cloud_segmented);
  leftRight(cloud_segmented, cloud_segmented);
//  ROS_DEBUG_STREAM("\t" << size << " --> " << cloud_segmented.size());
//  ROS_DEBUG("\tPerforming table removal...");
  size = cloud_segmented.size();
  pcl::ModelCoefficients::ConstPtr coeff = tableRemoval(cloud_segmented, cloud_segmented);
  underTableRemoval(cloud_segmented, cloud_segmented, coeff);
  aboveTableRemoval(cloud_segmented, cloud_segmented, coeff);
//  ROS_DEBUG_STREAM("\t" << size << " --> " << cloud_segmented.size());
//  ROS_DEBUG("\tPerforming outlier removal...");
//  size = cloud_segmented.size();
  outlierRemoval(cloud_segmented, cloud_segmented);
  ROS_DEBUG_STREAM("\t" << size << " --> " << cloud_segmented.size());
   }

bool loadCalibration(std::string kinectName,CameraCalibrationCV& calib) {
   int kinectCameraNo=0;
   if (kinectName=="/kinect_left")
        kinectCameraNo=0;
   else if (kinectName=="/kinect_right")
     kinectCameraNo=1;
   else if (kinectName=="/kinect_center")
     kinectCameraNo=2;
   else {
     ROS_ERROR_STREAM("does not recognize camera with name: "<<kinectName);
     return false;
   }
   std::stringstream KinectRawCalibrationPath;
      std::stringstream KinectPositionCalibrationPath;
      KinectRawCalibrationPath <<packagePath<< "/"<<"CalibrationFiles" << "/" << KinectRawCalibrationFileName[kinectCameraNo];
      KinectPositionCalibrationPath <<packagePath<< "/"<<"CalibrationFiles" << "/" << KinectPositionCalibrationFileName[kinectCameraNo];
//      ROS_INFO_STREAM("Loading Kinect calibration from file: \"" << KinectRawCalibrationPath.str() << "\"");
      //std::cout << "KinectPositionCalibrationPath " << KinectPositionCalibrationPath.str() << std::endl;
      CameraCalibrationCV* kinectCalibrationRawCV = new CameraCalibrationCV();
      kinectCalibrationRawCV->read(KinectRawCalibrationPath.str().c_str());;
//      ROS_INFO_STREAM("Loading external calibration from file: \"" << KinectPositionCalibrationPath.str() << "\"...");
      std::ifstream positionFileHandle;
      positionFileHandle.open(KinectPositionCalibrationPath.str().c_str());
      KMatrix<double> kTransform(4, 4);
      std::string base_frame_name;
      std::string camera_frame_name;
      positionFileHandle >> base_frame_name;
      positionFileHandle >> camera_frame_name;
      positionFileHandle >> kTransform;
      positionFileHandle.close();
  ////       std::cout<<"base_frame_name "<<base_frame_name<<std::endl;
  ////       std::cout<<"camera_frame_name "<<camera_frame_name<<std::endl;
//      std::cout << "kTransform " << kTransform << std::endl;

//      CameraCalibrationCV* kinectCalib = new CameraCalibrationCV();
      cv::Mat intrinsicsCV = kinectCalibrationRawCV->getRight(kinect_width, kinect_height);
//      std::cout << "intrinsicsCV: " << intrinsicsCV << std::endl;
      calib.setRightIntrinsics( intrinsicsCV);
  ////     std::cout<<"kinectCalibrationRawCV.getRight(cv_RGB_image.cols, cv_RGB_image.rows) "<<kinectCalibrationRawCV.getRight()<<std::endl;
      calib.setGlobalTransformation( kTransform );
  ////    kinectCalibrationRawCV.setGlobalTransformation(kTransform);
      calib.setImageHeight(kinect_height);
      calib.setImageWidth(kinect_width);

      delete kinectCalibrationRawCV;

      return true;
}

void kinectPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& PointCloudROS)
{

//  sensor_msgs::PointCloud2 PointCloud=*input;

  std::stringstream pcdFilePath;
  pcdFilePath<<packagePath<<"/"<<folder<<PointCloudROS->header.frame_id<<"_cloud_"<<objectID<<"_"<<counter<<".pcd";
  if (true) {
  pcl::io::savePCDFile(pcdFilePath.str().c_str(), *PointCloudROS);
  cout << "Files saved: " << pcdFilePath.str() << endl;
  }

  pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
  pcl::fromROSMsg<pcl::PointXYZRGB>(*PointCloudROS, pointCloud);

  if (true) {
  cv::Mat_<cv::Vec3b> rgb_original;
  cv::Mat_<ushort> depth_original;
  fromOrganizedPointCloudToRGBDepth(pointCloud, rgb_original ,depth_original);

  std::stringstream rgbOrigianlPath;
  rgbOrigianlPath <<packagePath<<"/"<< folder <<PointCloudROS->header.frame_id <<"_original"<< "_RGB_" << objectID << "_" << counter << ".png";

  std::stringstream depthOrigianlPath;
  depthOrigianlPath <<packagePath<<"/"<< folder << PointCloudROS->header.frame_id<<"_original" << "_depth_" << objectID << "_" << counter << ".png";

  cv::cvtColor(rgb_original, rgb_original, CV_RGB2BGR);
  cv::imwrite(rgbOrigianlPath.str(), rgb_original);
  cout << "Files saved: " << rgbOrigianlPath.str() << endl;

  cv::imwrite(depthOrigianlPath.str(), depth_original);
  cout << "Files saved: " << depthOrigianlPath.str() << endl;
  }

  cloudProcessing(pointCloud, pointCloud);

  cv::Mat_<cv::Vec3b> rgb;
  cv::Mat_<ushort> depth;
  if (pointCloud.isOrganized()) {
    fromOrganizedPointCloudToRGBDepth(pointCloud, rgb ,depth);
  }
  else {
    CameraCalibrationCV calib;
    loadCalibration(PointCloudROS->header.frame_id,calib);
    fromPointCloudToRGBDepth(pointCloud, calib,rgb ,depth);
  }
//
  std::stringstream rgbPath;
  rgbPath <<packagePath<<"/"<< folder <<PointCloudROS->header.frame_id << "_RGB_" << objectID << "_" << counter << ".png";
//
  std::stringstream depthPath;
  depthPath <<packagePath<<"/"<< folder << PointCloudROS->header.frame_id << "_depth_" << objectID << "_" << counter << ".png";

//
////   cv::Mat colorImage_BGR;
   cv::cvtColor(rgb, rgb, CV_RGB2BGR);
//
////   cv::imwrite(rgbPath.str(),colorImage_BGR);
  cv::imwrite(rgbPath.str(), rgb);
  cout << "Files saved: " << rgbPath.str() << endl;
  cv::imwrite(depthPath.str(), depth);
  cout << "Files saved: " << depthPath.str() << endl;
//
////    cv::FileStorage fs;
////    fs.open(depthPathYml.str(), cv::FileStorage::WRITE);
////    fs << "data3D" << depthImageCVPtr->image;
////    fs.release();
////    depthPathYml.str("");
////    cout << "Files saved: " << depthPathYml.str() << endl;
//  //cv::Mat test;
//  //test=cv::imread(depthPath.str(),0);
//  //cout << "format: " << test.type() << endl;
//  //cv_bridge::CvImage cviDepthRGB;
//  //cviDepthRGB.image=test;
//  //cout << "image type: " <<  cv_bridge::getCvType(msg_depth_image->encoding) << endl;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "frame_saver");

  ros::NodeHandle nodeHandle = ros::NodeHandle("~");

  counter = 0;
  folder = "images";
  char choice;

  image_transport::ImageTransport imageTransport(nodeHandle);

//  stereoSaver(imageTransport, "/bumblebeeLeft");
//  stereoSaver(imageTransport, "/bumblebeeRight");
//  stereoSaver(imageTransport, "/bumblebeeCenter");
//  kinectSaver(imageTransport, "/kinect_left");
//  kinectSaver(imageTransport, "/kinect_right");
//  kinectSaver(imageTransport, "/kinect_center");
//  stereoSaver(imageTransport, "/bikeBack");
//  stereoSaver(imageTransport, "/crop");
//  stereoSaver(imageTransport, "/downSample");
  kinectPointCloudSaver(nodeHandle, "/kinect_left");
  kinectPointCloudSaver(nodeHandle, "/kinect_right");
  kinectPointCloudSaver(nodeHandle, "/kinect_center");

  packagePath = ros::package::getPath("frame_saver");

  cout << "object number : " << endl;
  cin >> objectID;
  while (ros::ok())
  {
    cout << "instance number : " << endl;

    cin >> counter;
//       if (choice == 's'){ 
// 	counter++; 
    ros::spinOnce();
//       } 
  }

  return 0;
}

void stereoSaver(image_transport::ImageTransport imageTransport, string name)
{

  std::stringstream LeftPath;
  LeftPath << name << "/left/image_rect_color";

  std::stringstream RightPath;
  RightPath << name << "/right/image_rect_color";

  subscribterVector.push_back(new image_transport::SubscriberFilter());
  sub_temp_1 = subscribterVector.back();
  sub_temp_1->subscribe(imageTransport, LeftPath.str(), 1);

  subscribterVector.push_back(new image_transport::SubscriberFilter());
  sub_temp_2 = subscribterVector.back();
  sub_temp_2->subscribe(imageTransport, RightPath.str(), 1);

  timeSync.push_back(new message_filters::TimeSynchronizer<Image, Image>(1));
  timeSync.back()->connectInput(*sub_temp_1, *sub_temp_2);
  timeSync.back()->registerCallback(boost::bind(&stereoCallback, _1, _2));

}

void kinectSaver(image_transport::ImageTransport imageTransport, string name)
{

  std::stringstream rgbPath;
  rgbPath << name << "/rgb/image_color";

  std::stringstream DepthPath;
  DepthPath << name << "/depth_registered/image_raw";

  subscribterVector.push_back(new image_transport::SubscriberFilter());
  sub_temp_1 = subscribterVector.back();

  sub_temp_1->subscribe(imageTransport, rgbPath.str(), 1);

  subscribterVector.push_back(new image_transport::SubscriberFilter());
  sub_temp_2 = subscribterVector.back();

  sub_temp_2->subscribe(imageTransport, DepthPath.str(), 1);

//   timeSync.push_back(new message_filters::TimeSynchronizer< Image, Image>(1) );
//
//   timeSync.back()->connectInput(*sub_temp_1,*sub_temp_2);
//
//   timeSync.back()->registerCallback(boost::bind(&kinectCallback, _1, _2));
  timeSyncApprox.push_back(new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *sub_temp_1, *sub_temp_2));
  timeSyncApprox.back()->registerCallback(kinectCallback);

}

void kinectPointCloudSaver(ros::NodeHandle nh, string name)
{


  std::stringstream PointCloudPath;
  PointCloudPath << name << "/depth_registered/points";


  point_cloud_vector.push_back(new ros::Subscriber());
  *point_cloud_vector.back() = nh.subscribe (PointCloudPath.str(), 1, kinectPointCloudCallback);
//  subscribterVector.push_back(new image_transport::SubscriberFilter());
//  sub_temp_1 = subscribterVector.back();
//
//  sub_temp_1->subscribe(imageTransport, rgbPath.str(), 1);
//
//  subscribterVector.push_back(new image_transport::SubscriberFilter());
//  sub_temp_2 = subscribterVector.back();
//
//  sub_temp_2->subscribe(imageTransport, DepthPath.str(), 1);

//   timeSync.push_back(new message_filters::TimeSynchronizer< Image, Image>(1) );
//
//   timeSync.back()->connectInput(*sub_temp_1,*sub_temp_2);
//
//   timeSync.back()->registerCallback(boost::bind(&kinectCallback, _1, _2));
//  timeSyncApprox.push_back(new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *sub_temp_1, *sub_temp_2));
//  timeSyncApprox.back()->registerCallback(kinectCallback);

}

