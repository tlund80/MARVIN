// STL
#include <sstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>

// Eigen
#include <Eigen/Eigen>

// Variables
bool hasExtrinsic;
Eigen::Matrix4f Teig = Eigen::Matrix4f::Identity(); // Scaled to [m]
ros::Publisher pub; // Publisher for aligned point cloud

/*
 * Callback function does the following:
 *  - Reads the unaligned point cloud from depth_registered/points
 *  - Knows the extrinsic calibration transformation matrix Teig
 *  - Applies Teig to the points and publishes them through depth_registered/points_extrinsic
 */
void callback(sensor_msgs::PointCloud2::ConstPtr cloud) {
   if(hasExtrinsic) {
      sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2);
      pcl_ros::transformPointCloud(Teig, *cloud, *output);
      pub.publish(output);
   } else {
      pub.publish(cloud);
   }
}

/*
 * Main entry point
 */
int main(int argc, char** argv){
   // Init
   ros::init(argc, argv, "");
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
   ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/"+ns+"/depth_registered/points", queuesize, callback);
   pub = n.advertise<sensor_msgs::PointCloud2>("/"+ns+"/depth_registered/points_extrinsic", queuesize);
   
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
      /*
      Teig << R[0][0],R[0][1],R[0][2],0.001f*t[0],\
            R[1][0],R[1][1],R[1][2],0.001f*t[1],\
            R[2][0],R[2][1],R[2][2],0.001f*t[2],\
            0,0,0,1;
      */
      Teig << R[0][0],R[0][1],R[0][2],t[0],\
            R[1][0],R[1][1],R[1][2],t[1],\
            R[2][0],R[2][1],R[2][2],t[2],\
            0,0,0,1;
      
      // Create broadcaster
      tf::TransformBroadcaster br;
      
      // Setup rate
      ros::Rate r(rate);
      
      // Start
      while(ros::ok()) {
         br.sendTransform(tf::StampedTransform(T, ros::Time::now(), source, ns));      
         ros::spinOnce();
         r.sleep();
      }
   } else {
      ROS_ERROR_STREAM("No extrinsics available for \"" << ns << "\"!");
      ros::spin();
   }   
   
   
   return 0;
};
