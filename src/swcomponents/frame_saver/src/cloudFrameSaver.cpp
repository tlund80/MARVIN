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
#include <pcl_ros/transforms.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
//#include <caros/CovisRos.hpp>

#include <tr1/memory>
#include "cv.h"

#include <sstream>

#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/common/norms.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/ros/conversions.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/common/common.h>
//
//#include <pcl/ros/conversions.h>
////#include <pcl_ros/point_cloud.h>
////#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <sys/types.h>
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <errno.h>
#include <libgen.h>
#include <string.h>

#include <tf/transform_listener.h>

#include <boost/numeric/ublas/matrix.hpp>

using namespace std;
using namespace tr1;
using namespace message_filters;
using namespace sensor_msgs;
// using namespace Modules;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;

std::string counter;

std::string objectName;
std::string objectCategory;
std::string objectLightCond;

std::string folder;
std::string packagePath;

vector<image_transport::SubscriberFilter *> subscribterVector;

vector<ros::Subscriber*> point_cloud_vector;

image_transport::SubscriberFilter* sub_temp_1;
image_transport::SubscriberFilter* sub_temp_2;

vector<message_filters::TimeSynchronizer<Image, Image> *> timeSync;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyT;
vector<message_filters::Synchronizer<SyncPolicyT> *> timeSyncApprox;

void kinectPointCloudSaver(ros::NodeHandle, string name);
void stereoPointCloudSaver(ros::NodeHandle, string name);

bool USES_CATEGORY = false;
bool USES_LIGHT = false;
bool USES_SEGMENT = false;

tf::TransformListener * tf_listener;

pcl::PointCloud<pcl::PointXYZRGBA> PointCloudPCLCombined;
pcl::PointCloud<pcl::PointXYZRGBA> KinectPointCloudPCLCombined;
pcl::PointCloud<pcl::PointXYZRGBA> StereoPointCloudPCLCombined;

pcl::PCDWriter pcdWriter;

tf::StampedTransform gettransformationMatrix(const std::string frame_id)
{
//  cv::Mat_<float> global;
  tf::StampedTransform transform;
  try
  {
    tf_listener->lookupTransform("/world", frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

//  tf::transformAsMatrix(transform);
//  boost::numeric::ublas::matrix<double> transformAsMatrix      (       const Transform &       bt       )
  return transform;
}

//bool applyGlobalTranformation(pcl::PointCloud<pcl::PointXYZRGBA>& PointCloudTranformed, const tf::StampedTransform& transform)
//{
////  pcl::PointCloud<pcl::PointXYZRGBA> PointCloudTranformed;
//  cv::Mat<float> global;
//  boost::numeric::ublas::matrix<double> matrix= transformAsMatrix(  Transform );
//  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = PointCloudTranformed.begin(); it != PointCloudTranformed.end();
//      ++it)
//  {
//    const float xx = global(0, 0) * it->x + global(0, 1) * it->y + global(0, 2) * it->z + global(0, 3);
//    const float yy = global(1, 0) * it->x + global(1, 1) * it->y + global(1, 2) * it->z + global(1, 3);
//    const float zz = global(2, 0) * it->x + global(2, 1) * it->y + global(2, 2) * it->z + global(2, 3);
//    it->x = xx;
//    it->y = yy;
//    it->z = zz;
//  }
//
//  return true;
//}

//bool toMeter(pcl::PointCloud<pcl::PointXYZRGBA>& PointCloudTranformed)
//{
//
//  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = PointCloudTranformed.begin(); it != PointCloudTranformed.end();
//      ++it)
//  {
//    it->x /= 1000;
//    it->y /= 1000;
//    it->z /= 1000;
//  }
//
//  return true;
//}

bool saveCombined()
{
  if (!PointCloudPCLCombined.empty())
  {
    std::stringstream pcdFilePathCombined;
    pcdFilePathCombined << packagePath << "/" << folder << "/" << counter << "/" << "combined_cloud_" << objectName
        << "_" << counter << ".pcd";

    pcdWriter.writeBinaryCompressed(pcdFilePathCombined.str().c_str(), PointCloudPCLCombined);
    cout << "Files saved: " << pcdFilePathCombined.str() << endl;
  }

  if (!KinectPointCloudPCLCombined.empty())
  {
    std::stringstream pcdFilePathCombinedKinect;
    pcdFilePathCombinedKinect << packagePath << "/" << folder << "/" << counter << "/" << "combined_cloud_kinect_"
        << objectName << "_" << counter << ".pcd";
    pcdWriter.writeBinaryCompressed(pcdFilePathCombinedKinect.str().c_str(), KinectPointCloudPCLCombined);
    cout << "Files saved: " << pcdFilePathCombinedKinect.str() << endl;
  }
  if (!StereoPointCloudPCLCombined.empty())
  {
    std::stringstream pcdFilePathCombinedStereo;
    pcdFilePathCombinedStereo << packagePath << "/" << folder << "/" << counter << "/" << "combined_cloud_stereo_"
        << objectName << "_" << counter << ".pcd";
    pcdWriter.writeBinaryCompressed(pcdFilePathCombinedStereo.str().c_str(), StereoPointCloudPCLCombined);
    cout << "Files saved: " << pcdFilePathCombinedStereo.str() << endl;
  }
  return true;
}

void pointCloudCallback(sensor_msgs::PointCloud2 & PointCloudROS)
{

  std::string frame_id = PointCloudROS.header.frame_id;
  if (true)
  {
    pcl::PointCloud < pcl::PointXYZRGBA > PointCloudPCL;
    pcl::fromROSMsg < pcl::PointXYZRGBA > (PointCloudROS, PointCloudPCL);
    std::stringstream pcdFilePath;
    pcdFilePath << packagePath << "/" << folder << "/" << counter << "/" << frame_id << "_cloud_local_" << objectName
        << "_" << counter << ".pcd";
    pcdWriter.writeBinaryCompressed(pcdFilePath.str().c_str(), PointCloudPCL);
    cout << "Files saved: " << pcdFilePath.str() << endl;

  }
//  sensor_msgs::PointCloud2 PointCloud = *PointCloudROS;
  pcl_ros::transformPointCloud("/world", PointCloudROS, PointCloudROS, *tf_listener);
  pcl::PointCloud < pcl::PointXYZRGBA > PointCloudPCL;
  pcl::fromROSMsg < pcl::PointXYZRGBA > (PointCloudROS, PointCloudPCL);

  std::stringstream pcdFilePath;
  pcdFilePath << packagePath << "/" << folder << "/" << counter << "/" << frame_id << "_cloud_" << objectName << "_"
      << counter << ".pcd";

  pcdWriter.writeBinaryCompressed(pcdFilePath.str().c_str(), PointCloudPCL);
  cout << "Files saved: " << pcdFilePath.str() << endl;

  PointCloudPCLCombined += PointCloudPCL;

}

void kinectPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& PointCloudROS)
{

  sensor_msgs::PointCloud2 PointCloud = *PointCloudROS;
  pointCloudCallback(PointCloud);
  pcl::PointCloud < pcl::PointXYZRGBA > PointCloudPCL;
 
  // Convert input message to pointcloud
  pcl::PCLPointCloud2 out;
  pcl_conversions::toPCL(PointCloud, out);
  pcl::fromPCLPointCloud2(out, PointCloudPCL);
 // pcl::fromROSMsg < pcl::PointXYZRGBA > (PointCloud, PointCloudPCL);
  
  KinectPointCloudPCLCombined += PointCloudPCL;

}

void stereoPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& PointCloudROS)
{

  sensor_msgs::PointCloud2 PointCloud = *PointCloudROS;
  pointCloudCallback(PointCloud);
  pcl::PointCloud < pcl::PointXYZRGBA > PointCloudPCL;
  
   // Convert input message to pointcloud
  pcl::PCLPointCloud2 out;
  pcl_conversions::toPCL(PointCloud, out);
  pcl::fromPCLPointCloud2(out, PointCloudPCL);
  
  //pcl::fromROSMsg < pcl::PointXYZRGBA > (PointCloud, PointCloudPCL);
  StereoPointCloudPCLCombined += PointCloudPCL;
}

bool checkIfQuit(std::string& input)
{

  if (!input.compare("quit"))
    exit(0);
  else if (input.compare("back"))
    return false;

  unsigned found = folder.find_last_of("/\\");
  folder = folder.substr(0, found);
  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "frame_saver");

  if (argc > 1 && argv[1][0] == '-')
  {

    int i = strlen(argv[1]);

    while (--i)
    {

      switch (argv[1][i])
      {

        case 'c':
          USES_CATEGORY = true;
          break;

        case 'l':
          USES_LIGHT = true;
          break;

        case 's':
          USES_SEGMENT = true;
          break;
      }
    }
  }

  ros::NodeHandle nodeHandle = ros::NodeHandle("~");

  if (argc > 2)
    folder = "images/" + std::string(argv[2]);
  else
    folder = "images";

  kinectPointCloudSaver(nodeHandle, "/kinect_left");
  kinectPointCloudSaver(nodeHandle, "/kinect_right");
  kinectPointCloudSaver(nodeHandle, "/kinect_center");
  stereoPointCloudSaver(nodeHandle, "/bumblebeeRight");
  stereoPointCloudSaver(nodeHandle, "/bumblebeeLeft");
  stereoPointCloudSaver(nodeHandle, "/bumblebeeCenter");

  kinectPointCloudSaver(nodeHandle, "/carmine1");
  kinectPointCloudSaver(nodeHandle, "/carmine2");

  packagePath = ros::package::getPath("frame_saver");

  tf_listener = new tf::TransformListener();

  cout << "=====================================" << endl;
  cout << "        Type 'back' to go up         " << endl;
  cout << "        Type 'quit' to close         " << endl;
  cout << "=====================================" << endl;

  while (ros::ok())
  {

    if (USES_CATEGORY)
    {
      cout << "${frame_saver}/" << folder + "/> " << "Category Name : ";
      cin >> objectCategory;

      if (checkIfQuit(objectCategory))
        break;

      folder = folder + "/" + objectCategory;
    }

    while (ros::ok())
    {
      cout << "${frame_saver}/" << folder + "/> " + "Object Name : ";
      cin >> objectName;

      if (checkIfQuit(objectName))
        break;

      folder = folder + "/" + objectName;
      while (ros::ok())
      {
        if (USES_LIGHT)
        {
          cout << "${frame_saver}/" << folder + "/> " + "Lightning Condition : ";
          cin >> objectLightCond;

          if (checkIfQuit(objectLightCond))
            break;

          folder = folder + "/" + objectLightCond;
        }

        while (ros::ok())
        {
          cout << "${frame_saver}/" << folder + "/> " + "Instance Number : ";
          cin >> counter;

          if (checkIfQuit(counter))
            break;

          boost::filesystem::create_directories(packagePath + "/" + folder + "/" + counter);
//          if (true){
//            std::stringstream pcdFilePath;
//              pcdFilePath << packagePath << "/" << folder << "/" << counter<<"/" << "combined_cloud_"
//                  << objectName << "_" << counter << ".pcd";
//            pcl::io::savePCDFile(pcdFilePath.str().c_str(), PointCloudPCLCombined);
//            cout << "Files saved: " << pcdFilePath.str() << endl;

//          }

          ros::spinOnce();
          saveCombined();// save combined point clouds
        }

        if (!USES_LIGHT)
          break; //go_back
      }
    }

    if (!USES_CATEGORY)
      break; //go_back
  }

  return 0;
}

void kinectPointCloudSaver(ros::NodeHandle nh, string name)
{

  std::stringstream PointCloudPath;
  PointCloudPath << name << "/depth_registered/points";

  point_cloud_vector.push_back(new ros::Subscriber());
  *point_cloud_vector.back() = nh.subscribe(PointCloudPath.str(), 1, kinectPointCloudCallback);

}

void stereoPointCloudSaver(ros::NodeHandle nh, string name)
{

  std::stringstream PointCloudPath;
  PointCloudPath << name << "/points";

  point_cloud_vector.push_back(new ros::Subscriber());
  *point_cloud_vector.back() = nh.subscribe(PointCloudPath.str(), 1, stereoPointCloudCallback);

}

