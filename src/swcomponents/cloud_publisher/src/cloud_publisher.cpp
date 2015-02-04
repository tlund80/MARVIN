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
 \author Thomas Sølund
 \file cloud_publisher.cpp
 \brief
 */

// STL
#include <sstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/io/pcd_io.h>

#include <pcl/io/pcd_io.h>
// Eigen
#include <Eigen/Eigen>

// Variables
bool hasExtrinsic;
Eigen::Matrix4f Teig = Eigen::Matrix4f::Identity(); // Scaled to [m]

ros::Publisher pub; // Publisher for aligned point cloud

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_publisher");
  ros::NodeHandle n("~");

  // Namespace name is expected to be equal to the target frame name
  const std::string& ns = n.getNamespace();
  
  // Get arguments
  std::string source;
  int rate;
  if(!n.getParam(ns+"/source", source))
      source = "/world";
   if(!n.getParam(ns+"/rate", rate))
      rate = 30;

  // Setup publisher of aligned point clouds
  const int queuesize = 10;
//  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/"+ns+"/depth_registered/points", queuesize, callback);
  sensor_msgs::PointCloud2 _cloud;
  pub = n.advertise<sensor_msgs::PointCloud2>("/"+ns+"/depth_registered/points", queuesize);
  
   // Load extrinsics if available
   hasExtrinsic = n.hasParam(ns+"/position/data");
   if(hasExtrinsic) {
      XmlRpc::XmlRpcValue xmldata;
      n.getParam(ns+"/position/data", xmldata);
      ROS_ASSERT(xmldata.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(xmldata.size() == 1);
      const std::string sdata = xmldata[0];
   
      // Read rotation and translation
      std::istringstream iss(sdata);
      tf::Matrix3x3 R;
      tf::Vector3 t;
      // Row-wise
      for(int i = 0; i < 3; ++i) {
         tf::Vector3& Ri = R[i];
         iss >> Ri.m_floats[0] >> Ri.m_floats[1] >> Ri.m_floats[2] >> t[i];
      }
      
      // Create transformation
      tf::Transform T;
      T.setBasis(R);
      T.setOrigin(t);
            
      // Convert extrinsics to Eigen format [m]
      Teig << R[0][0],R[0][1],R[0][2],0.001f*t[0],\
            R[1][0],R[1][1],R[1][2],0.001f*t[1],\
            R[2][0],R[2][1],R[2][2],0.001f*t[2],\
            0,0,0,1;
      
      // Create broadcaster
      tf::TransformBroadcaster br;
      
      // Setup rate
      ros::Rate r(rate);
      
      //Load pointcloud
      if(n.hasParam(ns+"/cloud")){
	std::string path;
	if(!n.getParam(ns+"/cloud", path))
	    ROS_ERROR_STREAM("Could not get " << ns+"/cloud" << " from parameter server");
	if(pcl::io::loadPCDFile(path,_cloud))
	     ROS_ERROR_STREAM("Could not get lod pointCloud from: " << path.c_str());
      }
	
       _cloud.header.frame_id = ns;
      // Start
      while(ros::ok()) {
         br.sendTransform(tf::StampedTransform(T, ros::Time::now(), source, ns));  
         ros::spinOnce();
	 _cloud.header.stamp = ros::Time::now();
	pub.publish(_cloud);
         r.sleep();
      }
   } else {
      ROS_ERROR_STREAM("No extrinsics available for \"" << ns << "\"!");
      ros::spin();
   }   
	 
  return 0;
}


