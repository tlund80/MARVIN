/*
 * test_node.cpp
 *
 *  Created on: Oct 1, 2013
 *      Author: thomas
 */
// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

// Services
#include <pose_estimation_covis/estimate.h>
#include <pose_estimation_covis/prepareEstimation.h>


int main(int argc, char** argv) {
   // Initialize
   ros::init(argc, argv, "test_pose_estimation_pcl");
   ros::NodeHandle n("~");

   ROS_INFO("Loading model from %s", argv[1]);
   sensor_msgs::PointCloud2 model;
   if(pcl::io::loadPCDFile(argv[1], model) < 0) {
         ROS_ERROR("Failed to load test data!");
      return 1;
   }

   // Subscribe to prepare estimation service to create a surface model
   ROS_INFO("Subscribing to prepare pose estimation service...");
   ros::service::waitForService("/pose_estimation_covis/prepareEstimation");
   ros::ServiceClient prepare = n.serviceClient<pose_estimation_covis::prepareEstimation>("/pose_estimation_covis/prepareEstimation");


   pose_estimation_covis::prepareEstimation pMsg;
   pMsg.request.model.push_back(model);
   pMsg.request.model_name.push_back("salt");

  if(!prepare.call(pMsg)){
	  ROS_ERROR("Something went wrong when calling prepare pose estimation service");
	  return 1;
  }
  std::vector<std::string> id_vec;
  for(size_t i = 0; i<= pMsg.response.model_id.size()-1; i++)
  {
	  std::string model_id = pMsg.response.model_id[i];
	  ROS_INFO("Model id: %s", model_id.c_str());
	  id_vec.push_back(model_id);
  }


   // Subscribe to global estimation service
   ROS_INFO("Subscribing to pose estimation service...");
   ros::service::waitForService("/pose_estimation_covis/estimate");
   ros::ServiceClient estimate = n.serviceClient<pose_estimation_covis::estimate>("/pose_estimation_covis/estimate");

   pose_estimation_covis::estimate eMsg;
   //Adding models to recognize
   for(size_t j = 0; j <= id_vec.size()-1; j++){
	   eMsg.request.model_id.push_back(id_vec[j]);
   }
   eMsg.request.print = true;
   eMsg.request.local_refinement = false;

   if(!estimate.call(eMsg)){
   	  ROS_ERROR("Something went wrong when calling pose estimation service");
   	  return 1;
    }

   for(size_t k = 0; k<= eMsg.response.detected_obj_id.size()-1; k++)
   {
	 ROS_INFO("===================== %s model detected with id %s =================",eMsg.response.detected_obj_name[k].c_str(), eMsg.response.detected_obj_id[k].c_str());
	 ROS_INFO("Error: %f",eMsg.response.error[k]);
	 ROS_INFO("Inlier Fraction: %f",eMsg.response.inlier_fraction[k]);
	 ROS_INFO("Object Name: %s",eMsg.response.detected_obj_name[k].c_str());
	 ROS_INFO("Object Id: %s",eMsg.response.detected_obj_id[k].c_str());


	 geometry_msgs::Transform t = eMsg.response.poses.back();
	 
	 
	 ROS_INFO("Pose:\n\t X: %f Y: %f Z: %f",t.translation.x,t.translation.y,t.translation.z);
	 ROS_INFO("\n\t w: %f x: %f y: %f z: %f",t.rotation.w,t.rotation.x,t.rotation.y, t.rotation.z);


   }



   return 0;
}
