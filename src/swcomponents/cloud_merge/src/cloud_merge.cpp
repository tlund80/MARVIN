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
 \file object_modeller.cpp
 \brief
 */

//--------------PCL Includes-------------------
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/lccp_segmentation.h>

#include <pcl/common/pca.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/project_inliers.h>

//--------------ROS Includes -------------------
#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/publisher.h>

//#include <Eigen/Geometry>
//#include <Eigen/Dense>
#include "cloud_merge/CloudMerge.h"

#include <caros_common_msgs/CreateObjectModel.h>

#include <tr1/memory>
#include <sstream>
#include <map>
#include <ode/odeconfig.h>


using namespace std;
using namespace tr1;
using namespace sensor_msgs;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;

vector<ros::Subscriber*> point_cloud_vector;
//std::vector<double> plane_coeff;
pcl::ModelCoefficients::Ptr plane_coeff;

CloudT PointCloudPCLCombined;
CloudT KinectPointCloudPCLCombined;
CloudT StereoPointCloudPCLCombined;

visualization_msgs::Marker _crop_box_marker;
 ros::Publisher _crop_box_marker_publisher;
struct kinectClouds
{
  std::map<int, CloudT::Ptr > Clouds;
  std::vector<Eigen::Matrix4f> transform;
  bool kinect_received[3];
} kinectCloudStruct;

struct stereoClouds
{
//  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > Clouds;
  std::map<int, CloudT::Ptr > Clouds;
  std::vector<Eigen::Matrix4f> transform;
  bool stereo_received[3];
} stereoCloudStruct;

pcl_ros::Publisher<PointT> _pubAlignKinect;
pcl_ros::Publisher<PointT> _pubAlignStereo;
pcl_ros::Publisher<PointT> _pubAlignFull;

pcl_ros::Publisher<PointT> _pubKinectLeft;
pcl_ros::Publisher<PointT> _pubKinectRight;
pcl_ros::Publisher<PointT> _pubKinectCenter;

pcl_ros::Publisher<PointT> _pubBBLeft;
pcl_ros::Publisher<PointT> _pubBBRight;
pcl_ros::Publisher<PointT> _pubBBCenter;

ros::ServiceServer _model_srv;

void removePlane(CloudT::Ptr &src_cloud, CloudT::Ptr &target_cloud, double dist_threads)
{
   //*********************************************************************//
   //	Plane fitting
   /**********************************************************************/
    
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

   // Create the segmentation object
   pcl::SACSegmentation<PointT> seg;
   // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setDistanceThreshold (dist_threads);

   seg.setInputCloud (src_cloud);
   seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
   {
     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     //return (-1);
   }
   
   plane_coeff = coefficients;
  // plane_coeff.clear();
 //  for(int i = 0; i < coefficients->values.size(); i++)
  //   plane_coeff.push_back(coefficients->values.at(i));

 /*  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                       << coefficients->values[1] << " "
                                       << coefficients->values[2] << " "
                                       << coefficients->values[3] << std::endl;

*/
   //*********************************************************************//
   //	Extract Indices
   /**********************************************************************/

   CloudT::Ptr cloud_f (new CloudT);
   CloudT::Ptr cloud_p (new CloudT);
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers
  extract.setInputCloud (src_cloud);
  extract.setIndices(inliers);
  extract.setNegative (false);
  extract.setKeepOrganized(false);
  extract.filter (*cloud_p);
 // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl; 
 /*  try{
	pcl::io::savePCDFile("plane_fitting_indicies.pcd", *cloud_p);
  }catch(pcl::IOException &e){
	std::cerr << e.what() << std::endl; 
  }
*/
  // Create the filtering object
  extract.setNegative (true);
  extract.setKeepOrganized(false);
  extract.filter (*cloud_f);

  pcl::copyPointCloud(*cloud_f, *target_cloud);

}

bool extractClusters(CloudT::Ptr &src_cloud,
		     std::vector<CloudT::Ptr, Eigen::aligned_allocator_indirection<CloudT::Ptr> > &clusters,
		     unsigned int minClusterSize, unsigned int maxClusterSize, unsigned int clusterThreadshold
		    )
{
    
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
   
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (minClusterSize); //100
    ec.setMaxClusterSize (maxClusterSize); //50000
    ec.setSearchMethod (tree);
    ec.setInputCloud(src_cloud);
    ec.extract (cluster_indices); 
    
     std::cout<<"Found "  << cluster_indices.size() << " clusters" << std::endl;
     
    if ( cluster_indices.size() < 1) return false;
    
 //    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
     
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
       
	CloudT::Ptr cloud_cluster (new CloudT);
   
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back (src_cloud->points[*pit]); //*

	    if(cloud_cluster->points.size () > clusterThreadshold) //clusterThreadshold = 500
	     {
	     	 cloud_cluster->width = cloud_cluster->points.size ();
	     	 cloud_cluster->height = 1;
	     	 cloud_cluster->is_dense = true;
		 
	/*	 Eigen::Vector4f centroid;
		 pcl::compute3DCentroid(*cloud_cluster,centroid);
		 
		 pcl::PCA<pcl::PointXYZRGBA> _pca; 
		 pcl::PointXYZRGBA projected; 
		 pcl::PointXYZRGBA reconstructed;
		 pcl::PointCloud<pcl::PointXYZRGBA > cloudi = *cloud_cluster;
		 pcl::PointCloud<pcl::PointXYZRGBA> finalCloud;
		 
		 try{
		      //Do PCA for each point to preserve color information
		      //Add point cloud to force PCL to init_compute else a exception is thrown!!!HACK
		      _pca.setInputCloud(cloud_cluster);
		      for(int i = 0; i < (int)cloud_cluster->size(); i++)
		      {
			_pca.project(cloudi[i],projected);
			_pca.reconstruct (projected, reconstructed);

			//assign colors
			projected.r = cloudi[i].r;
			projected.g = cloudi[i].g;
			projected.b = cloudi[i].b;

			//add point to cloud
			finalCloud.push_back(projected);
		}
		 }catch(pcl::InitFailedException &e)
		 {
		    PCL_ERROR(e.what());
		 }

*/
	     	 std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		 //clusters.push_back(finalCloud.makeShared());
		  clusters.push_back(cloud_cluster);
	     }
	     
	     
	  //    pcl::copyPointCloud(*cloud_cluster, *model);
      }
    
    return true;
  
}

void toSpherical(pcl::PointNormal normal, float &radius, float &theta, float &phi ){
  
  radius = std::sqrt((normal.normal_x*normal.normal_x) + (normal.normal_y*normal.normal_y) + (normal.normal_z*normal.normal_z));
  theta = std::acos((normal.normal_z/radius));
  phi = std::atan(normal.normal_y/normal.normal_x);
  
}

void label_cloud(CloudT::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &label_cloud, uint32_t label){
  
   CloudT::iterator cloud_iter;
   for (cloud_iter = src_cloud->begin(); cloud_iter != src_cloud->end(); cloud_iter++) {
	pcl::PointXYZL p;
	p.x = cloud_iter->x; p.y = cloud_iter->y; p.z = cloud_iter->z;
	p.label = label;
	label_cloud->points.push_back(p);
   } 
}

void computePrincipalCurvature(CloudT::Ptr &src_cloud){
 
  // Compute the normals
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (src_cloud);

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  normal_estimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

  normal_estimation.setRadiusSearch (0.03);

  normal_estimation.compute (*cloud_with_normals);

  
  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<PointT, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

  // Provide the original point cloud (without normals)
  principal_curvatures_estimation.setInputCloud (src_cloud);

  // Provide the point cloud with normals
  principal_curvatures_estimation.setInputNormals (cloud_with_normals);

  // Use the same KdTree from the normal estimation
  principal_curvatures_estimation.setSearchMethod (tree);
  principal_curvatures_estimation.setRadiusSearch (1.0);

  // Actually compute the principal curvatures
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
  principal_curvatures_estimation.compute (*principal_curvatures);

  std::cout << "output points.size (): " << principal_curvatures->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];
  std::cout << descriptor << std::endl;
}

void LCCPSegmentation(CloudT::Ptr &src_cloud,pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr,
		      std::vector<CloudT, Eigen::aligned_allocator_indirection<CloudT> > &clusters,
		      bool use_supervoxel_refinement, bool has_normals ){
  
  ///------------------------------------------------- Supervoxel computation ------------------------------------------------------
  
   if(!has_normals)
     PCL_WARN ("Could not find normals point cloud. Normals will be calculated. This only works for single-camera-view pointclouds.\n");
  /// Supervoxel Stuff
  float voxel_resolution = 0.0045f;
  float seed_resolution = 0.03f;
  float color_importance = 2.0f;
  float spatial_importance = 1.0f;
  float normal_importance = 5.0f;
  bool use_single_cam_transform = false; //Kinect = true
 // bool use_supervoxel_refinement = false;
  
  /// Preparation of Input: Supervoxel Oversegmentation
  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, use_single_cam_transform);
  super.setInputCloud (src_cloud);
  if (has_normals)
    super.setNormalCloud (input_normals_ptr);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

  PCL_INFO ("Extracting supervoxels\n");
  super.extract (supervoxel_clusters);

  if (use_supervoxel_refinement)
  {
    PCL_INFO ("Refining supervoxels\n");
    super.refineSupervoxels (2, supervoxel_clusters);
  }
  std::stringstream temp;
  temp << "  Nr. Supervoxels: " << supervoxel_clusters.size () << "\n";
  PCL_INFO (temp.str ().c_str ());

  PCL_INFO ("Getting supervoxel adjacency\n");
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  
  /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);

  ///------------------------------------------------- Segmentation step ------------------------------------------------------
  
  /// LCCPSegmentation Variables
  float concavity_tolerance_threshold = 30; //30
  float smoothness_threshold = 0.1;
  uint32_t min_segment_size = 2;
  bool use_extended_convexity = true;
  bool use_sanity_criterion = false;
  
  uint k_factor = 0;
  if (use_extended_convexity)
    k_factor = 1;
  
  /// The Main Step: Perform LCCPSegmentation
  PCL_INFO ("Starting Segmentation\n");
  pcl::LCCPSegmentation<PointT> lccp;
  lccp.setConcavityToleranceThreshold (concavity_tolerance_threshold);
  lccp.setSanityCheck (use_sanity_criterion);
  lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
  lccp.setKFactor (k_factor);
  lccp.segment (supervoxel_clusters, supervoxel_adjacency);

  if (min_segment_size > 0)
  {
    PCL_INFO ("Removing small segments\n");
    lccp.removeSmallSegments (min_segment_size);
  }

  PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared ();
  lccp.relabelCloud (*lccp_labeled_cloud);
 // pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList sv_adjacency_list;
 // lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization
  
   /// Creating Colored Clouds and Output
  if (lccp_labeled_cloud->size () == src_cloud->size ()){
    
   //  if (pcl::getFieldIndex (*src_cloud, "label") >= 0)
   //       PCL_WARN ("Input cloud already has a label field. It will be overwritten by the lccp segmentation output.\n");
  
      ///Get all LCCP segments
      std::map<uint32_t, std::vector<uint32_t> > segment_supervoxel_map;   
      lccp.getSegmentSupervoxelMap(segment_supervoxel_map);
      uint32_t _num_of_segment = segment_supervoxel_map.size();
      std::cout << "Number of segment : " << _num_of_segment << std::endl;
      ///Initialize map to store inliers(segments)
      std::map<uint32_t, pcl::PointIndices::Ptr> segment_indice;
      for(uint32_t i = 0; i< _num_of_segment; i++){
	 pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	 segment_indice.insert(std::pair<uint32_t, pcl::PointIndices::Ptr>(i,inliers));
      }
      
      ///Iterate through LCCP labled cloud to copy label to inlier map
      pcl::PointCloud<pcl::PointXYZL>::iterator cloud_iter;
       std::map<uint32_t, uint32_t> pointcloud_label_to_new_label;
       int count = 0;
      for (cloud_iter = lccp_labeled_cloud->begin(); cloud_iter != lccp_labeled_cloud->end(); cloud_iter++) {
	//TODO: Kig på om label er ny. hvis: lig gamle lable i map på 1 til antal segmenters plads
	  uint32_t label =cloud_iter->label; 
	  if(pointcloud_label_to_new_label.count(label) == 0){ //The label dosen't exist
	      std::cout << "add label :" << count << std::endl;
             pointcloud_label_to_new_label.insert(std::pair<uint32_t,uint32_t>(label, count));
	     count++;
	  }else{ //label already exist. get the label position from pointcloud_label_to_new_label
	   uint32_t index = pointcloud_label_to_new_label.find(label)->second;
	 //  std::cout << "Save label :" << pointcloud_label_to_new_label.find(label)->first << " at index: " << index << std::endl;
	   if(index >= _num_of_segment ){
	     pcl::console::print_error("error: map index cannot exceed number of segments in cloud");
	     break;
	  }
	    //Map label to point
	    segment_indice.at(index)->indices.push_back(cloud_iter - lccp_labeled_cloud->begin());
	    
	  }
	
      }
      std::cout << "Number of labels : " << segment_indice.size() << std::endl;
      
      CloudT::Ptr segment_cloud (new CloudT);
      pcl::PointCloud<pcl::PointXYZL>::Ptr labled_cloud (new pcl::PointCloud<pcl::PointXYZL>);
      /// Extract full resolution point cloud for each segment 
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud (src_cloud);
      
      pcl::PointIndices::Ptr inliers_sac (new pcl::PointIndices);

      // Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.015);
      
      ///Segment iterator
   //   for(uint32_t i = 0; i<= segment_indice.size()-1; i++){
	  //Get all supervoxels in one segment
	  std::map<uint32_t, std::vector<uint32_t> >::iterator seg_iter;
	//  std::cout << "segment_supervoxel_map size: " << segment_supervoxel_map.size() << std::endl;
	  for(seg_iter = segment_supervoxel_map.begin(); seg_iter != segment_supervoxel_map.end(); seg_iter++) {
		///First get the label
		uint32_t segment_label = seg_iter->first;
		std::cout << "-------------------VoxelID's for segment --------------------------" << std::endl;
		std::vector<uint32_t> supervoxels_in_segment = seg_iter->second;
	        std::cout << "Number of supervoxels in segment: " << supervoxels_in_segment.size() << std::endl;
		  ///Supervoxel iterator in one segment
		  float theta_avg = 0;
		  for(uint32_t j = 0; j< supervoxels_in_segment.size(); j++){
		    uint32_t id = supervoxels_in_segment.at(j);
		    pcl::PointNormal normal;
		    supervoxel_clusters.at(id)->getCentroidPointNormal(normal);
		    float radius, theta, phi = 0;
		    toSpherical(normal, radius, theta, phi); 
		    theta_avg += theta;
		    //std::cout << "Normal_x: " << normal.normal_x << " Normal_y: " << normal.normal_y << " Normal_z: " << normal.normal_z << std::endl;
		  //  std::cout << " theta: " << theta << " phi: " << phi << std::endl;
		  }
		  float samples = (float(supervoxels_in_segment.size())); 
		 // std::cout << " samples: " << samples << std::endl;
		 // std::cout << " theta_acc: " << theta_avg << std::endl;
		  std::cout << " theta_avg: " << theta_avg/samples << std::endl;
		  theta_avg = theta_avg/samples;
		  
		  int label = std::distance(segment_supervoxel_map.begin(), seg_iter);
		  //std::cout << " label inliers: " << label << std::endl;
		  pcl::PointIndices::Ptr inliers =segment_indice.at(label);
		  if (inliers->indices.size () != 0){
		      extract.setIndices (inliers);
		      extract.setNegative (false);
		      extract.filter (*segment_cloud);
		      if(segment_cloud->points.size() > 10){ //only fitting plane for clouds larger than 10 points
			 //Estimate the plane coefficients
			 pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			 seg.setInputCloud (segment_cloud);
			 seg.segment (*inliers_sac, *coefficients);
			 
			 if (inliers_sac->indices.size () == 0)
				PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	      
			 std::cerr << "Model coefficients: "  << coefficients->values[0] << " "
								  << coefficients->values[1] << " "
								  << coefficients->values[2] << " "
								  << coefficients->values[3] << std::endl;
			 //coefficients.reset();
			 pcl::PointNormal plane_normal;
			 plane_normal.normal_x = coefficients->values[0];
			 plane_normal.normal_y = coefficients->values[1];
			 plane_normal.normal_z = coefficients->values[2];
			 float radius, theta,  phi = 0;
		         toSpherical(plane_normal, radius, theta, phi); 
			 std::cout <<"radius: " << radius << " theta: " << theta << " phi: " << phi << std::endl;
			   
			  //Compute centroid 
			  Eigen::Vector4d centroid;
			  pcl::compute3DCentroid(*segment_cloud,centroid);
			//  computePrincipalCurvature(segment_cloud);
			  
			  std::cout << "Centroid: " << centroid << std::endl;
			    
			 pcl::PointCloud<pcl::PointXYZL>::Ptr temp_label_cloud (new pcl::PointCloud<pcl::PointXYZL> );
			 //Only add segments smaller than t= 5000
			 //if(centroid[2] < 1.14f && centroid[2] > 1.05f && theta < 2.20f && (theta_avg < 2.20f)){//(theta < 1.50f || centroid[2] > 1.10) && (theta_avg > 2.00f)){
			 if(theta < 1.00f && (theta_avg < 2.20f)){//(theta < 1.50f || centroid[2] > 1.10) && (theta_avg > 2.00f)){
			   
			   CloudT::Ptr temp (new CloudT);
			            
			      //Deep copy
			      *temp = *segment_cloud; 
			      label_cloud(segment_cloud,temp_label_cloud, 1);
			      *labled_cloud += *temp_label_cloud;
			      ROS_ERROR("Valid cluster found!");
			     // std::stringstream ss;
			     // std::cout << "Saving segment " << label <<  "..."<< std::endl;
			     // ss << "/home/thso/segment_" << label; ss << ".pcd";
			     // pcl::io::savePCDFile(ss.str(),*segment_cloud);
			
			      clusters.push_back(*temp);
			 }else{
			    label_cloud(segment_cloud,temp_label_cloud, 2);
			   *labled_cloud += *temp_label_cloud;
			 }
		      }
		  }else{
		      std::cerr << "No inliers!!!!" << std::endl;
		  }
		
	
	 segment_cloud->clear();
	 pcl::io::savePCDFile("/home/thso/labled_cloud.pcd",*labled_cloud);	  
		  
		  
	  }
	  
  //    }
     
  }else{
    PCL_ERROR ("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
  }
  
   PCL_INFO ("Done .....\n");
}


void smooth(const std::map<int, CloudT::Ptr > &src_cloud, 
	    CloudT::Ptr &target_cloud, const std::vector<Eigen::Matrix4f> & transforms,
	    float radius)
{
    CloudT::Ptr filtered (new CloudT);
  
    std::vector<CloudT > clouds;
    for(unsigned int i = 0; i<= src_cloud.size()-1; i++){
      clouds.push_back(*src_cloud.find(i)->second);
    }
    CloudMerge* cm = new CloudMerge(clouds,transforms);
 
    cm->SetRadius(radius);
    //filtered = cm->MergeSmoothing();// MergeSmoothing();
      filtered = cm->MergeCombined();
    std::cout << "Finish .... " << std::endl;
    pcl::copyPointCloud(*filtered, *target_cloud);
     
    delete cm;
}

void checkForNaN(CloudT::Ptr& cloud)
{
	unsigned int count = 0; 
	
	CloudT::iterator it;
	for (it = cloud->begin(); it != cloud->end(); ++it){
	  //float _x = it->x;
	  //float _y = it->y;
	  //float _z = it->z;
	     // if(_x != _x || _y != _y || _z != _z){
		if(!pcl::isFinite(*it)){
			std::cout << count << " hej" << std::endl;
			cloud->erase(it);
			count++;
		}
	}
      std::cout << count << " NaN's removed from the point cloud" << std::endl;
}

void radiusOutlierRemoval(CloudT::Ptr &src_cloud, CloudT::Ptr &target_cloud, double radius)
{
	CloudT::Ptr cloud_filtered (new CloudT);

	if(src_cloud->size() > 0)
	{
	    try{
	      // Create the filtering object
	      pcl::RadiusOutlierRemoval<PointT> ror;
	      ror.setInputCloud(src_cloud);
	      ror.setRadiusSearch(radius);
	      ror.setMinNeighborsInRadius(800);
	      ror.filter (*cloud_filtered);
	      ror.setKeepOrganized(true);

	      pcl::copyPointCloud(*cloud_filtered, *target_cloud);
	    }catch(...)
	    {
	      PCL_ERROR("Somthing went wrong in object_modeller::radiusOutlierRemoval()");
	    }
	}
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
bool pairAlign (CloudT::Ptr &src_cloud, CloudT::Ptr &target_cloud, CloudT::Ptr &output, Eigen::Matrix4f &final_transform, bool downsample = true)
{

  bool result = true;
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  CloudT::Ptr src (new CloudT);
  CloudT::Ptr tgt (new CloudT);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.001, 0.001, 0.001);
    grid.setInputCloud (src_cloud);
    grid.filter (*src);

    grid.setInputCloud (target_cloud);
    grid.filter (*tgt);

    std::cout << "Filtered cloud contains " << src->size () << " data points" << std::endl;
  }
  else
  {
    src = src_cloud;
    tgt = target_cloud;
  }


  // Compute surface normals and curvature
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);

  
  pcl::NormalEstimation<PointT, pcl::PointNormal> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
/*  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
*/
  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  reg.setTransformationEpsilon (1e-9);
  reg.setEuclideanFitnessEpsilon(1e-9);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  //reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource(points_with_normals_src);//setInputCloud (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);


  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);

  double conv;
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);//setInputCloud (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    std::cout << "sum: " << (reg.getLastIncrementalTransformation () - prev).sum () << std::endl;
    
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();
    conv = reg.getFitnessScore();
    std::cout << "Registration has converged: " << reg.hasConverged() << " with fitness score: " << reg.getFitnessScore() << std::endl;

  }

  if(conv > 0.0001)
  {
	  PCL_ERROR("Registration Error!!");
	 // result = false;
  }
	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*target_cloud, *output, targetToSource);
  
  //add the source to the transformed target
  *output += *src_cloud;
  
  final_transform = targetToSource;
  
  return result;
 }

void MLSApproximation(CloudT::Ptr &cloud, CloudT::Ptr &target)
{
	 using namespace pcl::console;
	 
	 /// Create a KD-Tree
	 pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

	 /// Output has the PointNormal type in order to store the normals calculated by MLS
	 CloudT::Ptr mls_points (new CloudT);
	 CloudT::Ptr mls_points_f (new CloudT);

	 /// Init object (second point type is for the normals, even if unused)
	 pcl::MovingLeastSquares<PointT, PointT> mls;

	 /// Set parameters
	 mls.setInputCloud (cloud);
	 mls.setPolynomialFit (true);
	 mls.setSearchMethod (tree);
	 mls.setSearchRadius (0.01); //0.03
	 
	 /// Reconstruct
	 TicToc tt;
	 tt.tic ();
	 print_highlight("Computing smoothed point cloud using MLS algorithm....");
	 mls.process (*mls_points);
	 print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
	  
	     pcl::io::savePCDFile("/home/thso/with_nan.pcd", *mls_points);
	 //    checkForNaN(mls_points);
	  std::vector<int> index;
	  pcl::removeNaNFromPointCloud(*mls_points,*mls_points_f, index);
	    //   std::cout << "index size: " << index.size() << std::endl; 
	  pcl::io::savePCDFile("/home/thso/without_nan.pcd", *mls_points_f);
	  mls.setInputCloud (mls_points_f);
	  std::cout << "point size before: " << mls_points->size() << std::endl; 
	   
	   mls.setDilationVoxelSize(0.0005); 
	   mls.setSearchRadius (0.01); //0.03
	  // mls.setPointDensity(70);
	  // mls.setUpsamplingMethod(pcl::MovingLeastSquares< pcl::PointXYZRGBA, pcl::PointXYZRGBA >::RANDOM_UNIFORM_DENSITY);
	    mls.setUpsamplingMethod(pcl::MovingLeastSquares< PointT, PointT >::VOXEL_GRID_DILATION);
	   mls.process(*target);
	  
	   std::cout << "point size after: " << target->size() << std::endl; 
	  //pcl::copyPointCloud(*mls_points, *target);
	

}

void CropBox(CloudT::Ptr &src_cloud, CloudT::Ptr &target_cloud,
             float rx, float ry, float minz, float maxz, Eigen::Matrix4f mat, bool publish_box_marker)
{
  
   Eigen::Matrix3f rot_center = Eigen::Matrix3f::Identity();
   Eigen::Affine3f transform(Eigen::Translation3f(0.0,-0.50,0)); //move the base frame to the middle of the table 
   Eigen::Matrix4f m = transform.matrix();
   Eigen::Matrix4f mat_inv = m;//mat.inverse() * m;
   Eigen::Affine3f matrix;
    
   matrix(0,0) = mat_inv(0,0); matrix(0,1) = mat_inv(0,1); matrix(0,2) = mat_inv(0,2); matrix(0,3) =  mat_inv(0,3);
   matrix(1,0) = mat_inv(1,0); matrix(1,1) = mat_inv(1,1); matrix(1,2) = mat_inv(1,2); matrix(1,3) = mat_inv(1,3); 
   matrix(2,0) = mat_inv(2,0); matrix(2,1) = mat_inv(2,1); matrix(2,2) = mat_inv(2,2); matrix(2,3) = mat_inv(2,3);
     
   
   float x, y, z,roll, pitch, yaw;
   pcl::getTranslationAndEulerAngles(matrix,x,y,z,roll,pitch, yaw);
   Eigen::Vector3f boxTranslatation; boxTranslatation[0] = x; boxTranslatation[1] = y; boxTranslatation[2] = z;
   Eigen::Vector3f boxRotation; boxRotation[0] = roll; boxRotation[1] = pitch; boxRotation[2] = yaw; 
  
  
  CloudT::Ptr filtered (new CloudT);
  
  //Plcae frame in the middle of the box
 // boxTranslatation[0] += rx/2;
 // boxTranslatation[1] -= ry/2;
  pcl::CropBox<PointT> cropFilter; 
  cropFilter.setInputCloud (src_cloud); 
  cropFilter.setMin(Eigen::Vector4f(-rx, -ry, minz, 1.0f));  
  cropFilter.setMax(Eigen::Vector4f(rx, ry, maxz, 1.0f)); 
  cropFilter.setTranslation(boxTranslatation); 
  cropFilter.setRotation(boxRotation); 
  cropFilter.setKeepOrganized(false);
  cropFilter.setNegative(false);


  cropFilter.filter (*filtered); 
  
  pcl::copyPointCloud(*filtered, *target_cloud);
  
  
  if(publish_box_marker){
    
  _crop_box_marker.header.frame_id = "/world";
  _crop_box_marker.header.stamp = ros::Time::now();
  
  _crop_box_marker.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  _crop_box_marker.action = visualization_msgs::Marker::ADD;
   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
   tf::Quaternion q;
   q.setRPY(roll,pitch,yaw);
    
  
   tf::TransformBroadcaster br;
   tf::Transform tf_transform;
   tf_transform.setOrigin( tf::Vector3(x,y,z) );
   tf_transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), _crop_box_marker.header.frame_id, "cropbox_frame"));

   _crop_box_marker.pose.position.x = double(matrix(0,3));
   _crop_box_marker.pose.position.y = double(matrix(1,3));
   _crop_box_marker.pose.position.z = double(matrix(2,3));
   _crop_box_marker.pose.orientation.x = double(q.getX());
   _crop_box_marker.pose.orientation.y = double(q.getY());
   _crop_box_marker.pose.orientation.z = double(q.getZ());
   _crop_box_marker.pose.orientation.w = double(q.getW());
   
   // Set the scale of the marker -- 1x1x1 here means 1m on a side
   _crop_box_marker.scale.x = 2*rx;
   _crop_box_marker.scale.y = 2*ry;
   _crop_box_marker.scale.z = std::abs(maxz - minz);
   
   // Set the color -- be sure to set alpha to something non-zero!
   _crop_box_marker.color.r = 0.0f;
   _crop_box_marker.color.g = 1.0f;
   _crop_box_marker.color.b = 0.0f;
   _crop_box_marker.color.a = 0.7f;
   
   _crop_box_marker.lifetime = ros::Duration(120);
   
   _crop_box_marker_publisher.publish(_crop_box_marker);
    
  }
  
}

void publish_marker(double rx, double ry, double minz, double maxz,  Eigen::Matrix4f mat){  
  _crop_box_marker.header.frame_id = "/world";
  _crop_box_marker.header.stamp = ros::Time::now();
  
  _crop_box_marker.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  _crop_box_marker.action = visualization_msgs::Marker::ADD;
   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

   Eigen::Matrix3f rot_center = Eigen::Matrix3f::Identity();
   Eigen::Affine3f transform(Eigen::Translation3f(0.0,-0.55,0)); //move the base frame to the middle of the table 
   Eigen::Matrix4f m = transform.matrix();
   Eigen::Matrix4f mat_inv = mat.inverse() * m;
   Eigen::Affine3f matrix;
    
   matrix(0,0) = mat_inv(0,0); matrix(0,1) = mat_inv(0,1); matrix(0,2) = mat_inv(0,2); matrix(0,3) =  mat_inv(0,3);
   matrix(1,0) = mat_inv(1,0); matrix(1,1) = mat_inv(1,1); matrix(1,2) = mat_inv(1,2); matrix(1,3) = mat_inv(1,3); 
   matrix(2,0) = mat_inv(2,0); matrix(2,1) = mat_inv(2,1); matrix(2,2) = mat_inv(2,2); matrix(2,3) = mat_inv(2,3);
     
    float x, y, z,roll, pitch, yaw;
   pcl::getTranslationAndEulerAngles(matrix,x,y,z,roll,pitch, yaw);
   // Matrix3f mat;
    //Eigen::Quaternionf q(matrix.rotation());
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    
  
   tf::TransformBroadcaster br;
   tf::Transform tf_transform;
   tf_transform.setOrigin( tf::Vector3(x,y,z) );
   tf_transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), _crop_box_marker.header.frame_id, "cropbox_frame"));

   _crop_box_marker.pose.position.x = double(matrix(0,3));
   _crop_box_marker.pose.position.y = double(matrix(1,3));
   _crop_box_marker.pose.position.z = double(matrix(2,3));
   _crop_box_marker.pose.orientation.x = double(q.getX());
   _crop_box_marker.pose.orientation.y = double(q.getY());
   _crop_box_marker.pose.orientation.z = double(q.getZ());
   _crop_box_marker.pose.orientation.w = double(q.getW());
   
   // Set the scale of the marker -- 1x1x1 here means 1m on a side
   _crop_box_marker.scale.x = 2*rx;
   _crop_box_marker.scale.y = 2*ry;
   _crop_box_marker.scale.z = std::abs(maxz - minz);
   
   // Set the color -- be sure to set alpha to something non-zero!
   _crop_box_marker.color.r = 0.0f;
   _crop_box_marker.color.g = 1.0f;
   _crop_box_marker.color.b = 0.0f;
   _crop_box_marker.color.a = 0.7f;
   
   _crop_box_marker.lifetime = ros::Duration(120);
   
   _crop_box_marker_publisher.publish(_crop_box_marker);
}

void PassThroughFilter(CloudT::Ptr &src_cloud, CloudT::Ptr &target_cloud,
		       double min_depth, double max_depth, std::string direction)
{
       CloudT::Ptr filtered (new CloudT);
	// Preprocess the cloud by...
	// ...removing distant points
	//const float depth_limit = 1.0;

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (src_cloud);
	pass.setFilterFieldName (direction);
	pass.setFilterLimits (min_depth, max_depth);
	pass.filter (*filtered);

	 pcl::copyPointCloud(*filtered, *target_cloud);
}

bool process_cloud(caros_common_msgs::CreateObjectModel::Request &req, caros_common_msgs::CreateObjectModel::Response &res){

  /// Allocate point clouds
  CloudT::Ptr final_cloud (new CloudT);
  
  std::vector<CloudT,  Eigen::aligned_allocator_indirection<CloudT> > clusters;
	
   ///Now do pre-processing for each cloud
   for(int i = 0; i< 3; i++){
       CloudT::Ptr filtered (new CloudT);
       CloudT::Ptr out (new CloudT);
    // int i=0;
      ROS_INFO("Cropping cloud from view %i", i);
     Eigen::Matrix4f mat;
     if(req.sensor == 0){
	filtered = kinectCloudStruct.Clouds.find(i)->second;
	mat = kinectCloudStruct.transform.at(i);  

	//CropBox(filtered,filtered,0.65,0.3, -0.2, 0.8,mat);
	// PassThroughFilter(filtered,filtered,0.0,2.0,"z");
     }else if(req.sensor == 1){
        filtered = stereoCloudStruct.Clouds.find(i)->second;
	mat = stereoCloudStruct.transform.at(i); 
	
	// PassThroughFilter(filtered,filtered,0.0,2.0,"z");
//		 _pubBBLeft.publish(*filtered);
     }else{
      return false;      
    }
    
     double rx, ry, minz, maxz;
     rx = 0.65; ry = 0.3; minz = -0.2; maxz = 0.8;
     CropBox(filtered,filtered,rx,ry, minz, maxz,mat, true);
  //   publish_marker( rx,ry,minz,maxz, mat);
	
     ROS_INFO("Saving view %i to hard drive!!", i);
     std::stringstream ss;
     ss << "/home/thso/view_" << i; ss << ".pcd";
     if(filtered->points.size() > 0) pcl::io::savePCDFile(ss.str(), *filtered  );
    
/*	 
     pcl::PointCloud<pcl::Normal>::Ptr dummy_ptr;
     LCCPSegmentation(filtered, dummy_ptr, clusters,true,false);
     
      //Find the correct models
     PCL_INFO("Selecting cluster for view %i ......\n", i);
     for(int j = 0; j < clusters.size(); j++){
       //Filtering by simple point size criterion
       std::cout << clusters[j].points.size() << " points in cluster" << std::endl;
       if(clusters.at(j).points.size() < 50000 && clusters.at(j).points.size() > 500){
	 ROS_ERROR("Selecting cluster!!");
         *out += clusters.at(j);
	
       }
     }
     clusters.clear();
     
     std::vector<int> index;
     pcl::removeNaNFromPointCloud(*out,*final_cloud,index);
     
     if(req.sensor == 0) KinectPointCloudPCLCombined += *final_cloud;
     else if(req.sensor == 1) StereoPointCloudPCLCombined += *final_cloud;
     
     
     ss.str("");
     ss << "/home/thso/selected_cluster_" << i; ss << ".pcd";
     pcl::io::savePCDFile(ss.str(), *final_cloud  );       
     out->clear();
     filtered->clear();
 */
//     removePlane(filtered,filtered,0.005); //0.015 gredy 
//     radiusOutlierRemoval(filtered,filtered,0.08); 

//     pcl::io::savePCDFile("/home/thso/mls_filtered.pcd", *filtered  );
   //  MLSApproximation(filtered, filtered);
  
 /*    size_t erased = kinectCloudStruct.Clouds.erase(i);
     if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(i, final_cloud)).second){
	ROS_ERROR("Could not add cloud to map");
     }
   */
     
  }
   
  
  
  
  
  
  /*   if(i > 0){
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _src =  kinectCloudStruct.Clouds.find(i-1)->second; 
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _tar =  kinectCloudStruct.Clouds.find(i)->second; 
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _output;
     pairAlign(_src,_tar,_output)
       
     }
    */ 
  //   pcl::transformPointCloud(*filtered,*out,mat);
  //   KinectPointCloudPCLCombined += *out;

     //pcl::io::savePCDFile("/home/thso/scene_w_plane.pcd", KinectPointCloudPCLCombined  );
     //smooth(kinectCloudStruct.Clouds,final_cloud,kinectCloudStruct.transform,0.5);
   //  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model = KinectPointCloudPCLCombined.makeShared();

     //std::vector<int> index;
     //pcl::removeNaNFromPointCloud(*final_cloud,*final_cloud,index);
    // extractClusters(final_cloud, clusters,100, 50000, 500); 
     
      // Create a Concave Hull representation of the projected inliers
 /*     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
      chull.setInputCloud (clusters[0]);
      chull.setDimension(3);
      chull.setAlpha(0.006);
      chull.setKeepInformation(true);
      chull.reconstruct (*cloud_hull);
*/
 
/*	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
     //checkForNaN(clusters[0]);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(clusters[0]);
	proj.setModelCoefficients (coefficients);
	proj.filter (*proj_cloud);
*/	
	//pcl::io::savePCDFile("proj_cloud.pcd", *proj_cloud);
      //  pcl::io::savePCDFile("model.pcd", *clusters[0]);
	//*clusters[0]+= *proj_cloud;
	
  //   result->header.frame_id = "/world";
  //  _pubAlignKinect.publish(clusters[0]);
   
   
   
   sensor_msgs::PointCloud2 cloud_model;
   
   try{
	  if(req.sensor == 0){
	    pcl::io::savePCDFile("/home/thso/final_kinect_model.pcd", KinectPointCloudPCLCombined);
	    pcl::toROSMsg(KinectPointCloudPCLCombined,cloud_model);
	  }else if(req.sensor == 1){
	    pcl::io::savePCDFile("/home/thso/final_stereo_model.pcd", StereoPointCloudPCLCombined);
	    pcl::toROSMsg(StereoPointCloudPCLCombined,cloud_model);
	  }
   }catch(pcl::IOException &e){
	std::cerr << e.what() << std::endl; 
   }
  
  KinectPointCloudPCLCombined.clear();
  StereoPointCloudPCLCombined.clear();
  
  ///Send result
  res.model = cloud_model;
  res.success = true;
  
  return true;  
}

bool create_model_callback(caros_common_msgs::CreateObjectModel::Request &req, caros_common_msgs::CreateObjectModel::Response &res){
  
  sensor_msgs::PointCloud2::ConstPtr cloud_left;
  sensor_msgs::PointCloud2::ConstPtr cloud_right;
  sensor_msgs::PointCloud2::ConstPtr cloud_center;
  
  Eigen::Matrix4f mat;
  CloudT PointCloudPCL;

  switch(req.sensor)
  {      
    case 0: //Kinect sensor
   {   
	cloud_left = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_left/depth_registered/points", ros::Duration(5.0));
	pcl::fromROSMsg < PointT > (*cloud_left, PointCloudPCL);
	if(!cloud_left) {
	  ROS_WARN("Retrieval of /kinect_left/depth_registered/points point cloud failed!");
	  return false;
	}
	mat = kinectCloudStruct.transform.at(0);
	pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	if(kinectCloudStruct.Clouds.erase(0) < 1) ROS_ERROR("Could not erase /kinect_left cloud from map!");
	if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(0,PointCloudPCL.makeShared())).second){
	    ROS_ERROR("Could not insert /kinect_left cloud into map!");
	}
	
	cloud_right = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_right/depth_registered/points", ros::Duration(5.0));
	pcl::fromROSMsg < PointT > (*cloud_right, PointCloudPCL);
	if(!cloud_right) {
	  ROS_WARN("Retrieval of /kinect_right/depth_registered/points point cloud failed!");
	  return false;
	}
        mat = kinectCloudStruct.transform.at(1);
	pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	if(kinectCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /kinect_right cloud from map!");
	if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(1,PointCloudPCL.makeShared())).second){
	    ROS_ERROR("Could not insert /kinect_right cloud into map!");  
	}
	
	cloud_center = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_center/depth_registered/points", ros::Duration(5.0));
	pcl::fromROSMsg <PointT > (*cloud_center, PointCloudPCL);
	if(!cloud_center) {
	  ROS_WARN("Retrieval of /kinect_center/depth_registered/points point cloud failed!");
	  return false;
	}
	mat = kinectCloudStruct.transform.at(2);
	pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	if(kinectCloudStruct.Clouds.erase(2) < 1) ROS_ERROR("Could not erase /kinect_center cloud from map!");
	if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(2,PointCloudPCL.makeShared())).second){
          ROS_ERROR("Could not insert /kinect_center cloud into map!"); 
      }
    
     /// All Kinect clouds are received. Lets process the clouds
      process_cloud(req,res);
      
      std::cout << "Extracted model has " << res.model.data.size() << " points"<< std::endl;
      
      break;
   }  
      
    case 1: //Stereo sensor
   {
	  cloud_left = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/bumblebeeLeft/depth_registered/points", ros::Duration(5.0));
	  std::cout << "points received: " << cloud_left->width * cloud_left->height << std::endl;
	pcl::fromROSMsg < PointT > (*cloud_left, PointCloudPCL);
	if(!cloud_left) {
	  ROS_WARN("Retrieval of /bumblebeeLeft/points point cloud failed!");
	  return false;
	}
	mat = stereoCloudStruct.transform.at(0);
	pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	if(stereoCloudStruct.Clouds.erase(0) < 1) ROS_ERROR("Could not erase /bumblebeeLeft cloud from map!");
	if(!stereoCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(0,PointCloudPCL.makeShared())).second){
	    ROS_ERROR("Could not insert /bumblebeeLeft cloud into map!");
	}
	
	cloud_right = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/bumblebeeRight/depth_registered/points", ros::Duration(5.0));
	 std::cout << "points received: " << cloud_right->width * cloud_right->height << std::endl;
	pcl::fromROSMsg < PointT > (*cloud_right, PointCloudPCL);
	if(!cloud_right) {
	  ROS_WARN("Retrieval of /bumblebeeRight/points point cloud failed!");
	  return false;
	}
	mat = stereoCloudStruct.transform.at(1);
	pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);

	if(stereoCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /bumblebeeRight cloud from map!");
	if(!stereoCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(1,PointCloudPCL.makeShared())).second){
	    ROS_ERROR("Could not insert /bumblebeeRight cloud into map!");  
	}
	
	cloud_center = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/bumblebeeCenter/depth_registered/points", ros::Duration(5.0));
		 std::cout << "points received: " << cloud_center->width * cloud_center->height << std::endl;
	pcl::fromROSMsg <PointT > (*cloud_center, PointCloudPCL);
	if(!cloud_center) {
	  ROS_WARN("Retrieval of /bumblebeeCenter/points point cloud failed!");
	  return false;
	}
	mat = stereoCloudStruct.transform.at(2);
	pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);

	if(stereoCloudStruct.Clouds.erase(2) < 1) ROS_ERROR("Could not erase /bumblebeeCenter cloud from map!");
	if(!stereoCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(2,PointCloudPCL.makeShared())).second){
          ROS_ERROR("Could not insert /bumblebeeCenter cloud into map!"); 
      }
        /// All stereo clouds are received. Lets process the clouds
	process_cloud(req,res);
	std::cout << "Extracted model has " << res.model.data.size() << " points"<< std::endl;
      
      break;
   }
      
    case 2: //Both sensors
	  ROS_WARN("cloud_merge not implemented yet for stereo+kinect!");
      break;
      
    default:
      
      break; 
  }
  return true;
}


void pointCloudCallback(sensor_msgs::PointCloud2 & PointCloudROS)
{

//    pcl_ros::transformPointCloud("/world", PointCloudROS, PointCloudROS, *tf_listener);
    CloudT PointCloudPCL;
    pcl::fromROSMsg < PointT > (PointCloudROS, PointCloudPCL);

    PointCloudPCLCombined += PointCloudPCL;

}

void kinectPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& PointCloudROS){
  std::string received_frame = PointCloudROS->header.frame_id;

  CloudT::Ptr out (new CloudT);
  CloudT::Ptr filtered (new CloudT);
  CloudT::Ptr result (new CloudT);
  CloudT::Ptr final_cloud (new CloudT);
  
  std::vector<CloudT::Ptr,  Eigen::aligned_allocator_indirection<CloudT::Ptr> > clusters;
  
  sensor_msgs::PointCloud2 PointCloud = *PointCloudROS;
  pcl::PointCloud < PointT > PointCloudPCL;
  pcl::fromROSMsg <PointT > (PointCloud, PointCloudPCL);
  
  filtered = PointCloudPCL.makeShared();
  PassThroughFilter(filtered,filtered,0.0,2.0,"z");
  
  if(received_frame.compare("/kinect_left") == 0){
    //std::cout << "Recived /kinect_left" << std::endl;
    kinectCloudStruct.kinect_received[0] = true;
    Eigen::Matrix4f mat = kinectCloudStruct.transform.at(0);
    pcl::transformPointCloud(*filtered,*filtered,mat);
    CropBox(filtered,filtered,0.65,0.3, -0.2, 0.8,mat, false);
 //   removePlane(filtered,filtered,0.020); //0.015 gredy 
    _pubKinectLeft.publish(*filtered);
    KinectPointCloudPCLCombined += *filtered;
    if(kinectCloudStruct.Clouds.erase(0) < 1) ROS_ERROR("Could not erase /kinect_left cloud from map!");
    if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(0,PointCloudPCL.makeShared())).second){
      ROS_ERROR("Could not insert /kinect_left cloud into map!");
    }
    
  }else if(received_frame.compare("/kinect_right") == 0){
    //std::cout << "Recived /kinect_right" << std::endl;
    kinectCloudStruct.kinect_received[1] = true;
    Eigen::Matrix4f mat = kinectCloudStruct.transform.at(1);
    pcl::transformPointCloud(*filtered,*filtered,mat);
    CropBox(filtered,filtered,0.65,0.3, -0.2, 0.8,mat, false);
 //   removePlane(filtered,filtered,0.020); //0.015 gredy 
    KinectPointCloudPCLCombined += *filtered;
    _pubKinectRight.publish(*filtered);
    if(kinectCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /kinect_right cloud from map!");
    if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(1,PointCloudPCL.makeShared())).second){
     ROS_ERROR("Could not insert /kinect_right cloud into map!");  
    }

  }else if(received_frame.compare("/kinect_center") == 0){
     //std::cout << "Recived /kinect_center" << std::endl;
     kinectCloudStruct.kinect_received[2] = true;
     Eigen::Matrix4f mat = kinectCloudStruct.transform.at(2);
     pcl::transformPointCloud(*filtered,*filtered,mat);
     CropBox(filtered,filtered,0.65,0.3, -0.2, 0.8,mat,false);
   //  removePlane(filtered,filtered,0.020); //0.015 gredy 
     _pubKinectCenter.publish(*filtered);
     KinectPointCloudPCLCombined += *filtered;
     if(kinectCloudStruct.Clouds.erase(2) < 1) ROS_ERROR("Could not erase /kinect_center cloud from map!");
     if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(2,PointCloudPCL.makeShared())).second){
          ROS_ERROR("Could not insert /kinect_center cloud into map!"); 
    }
  }
 
 //All Kinect clouds are received.
 if(kinectCloudStruct.kinect_received[0] == true && kinectCloudStruct.kinect_received[1] == true && kinectCloudStruct.kinect_received[2] == true){
    //std::cout << "Publish merge cloud" << std::endl;
    _pubAlignKinect.publish(KinectPointCloudPCLCombined);
     KinectPointCloudPCLCombined.clear();
     
     kinectCloudStruct.kinect_received[0] = false;
     kinectCloudStruct.kinect_received[1] = false;
     kinectCloudStruct.kinect_received[2] = false;
  
   }
}


void stereoPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& PointCloudROS)
{
  std::string received_frame = PointCloudROS->header.frame_id;

  CloudT::Ptr filtered (new CloudT);
  
  sensor_msgs::PointCloud2 PointCloud = *PointCloudROS;
 // pointCloudCallback(PointCloud);
  pcl::PointCloud < PointT > PointCloudPCL;
  pcl::fromROSMsg < PointT > (PointCloud, PointCloudPCL);
  
   if(received_frame.compare("/bumblebeeLeft") == 0){
    stereoCloudStruct.stereo_received[0] = true;
    Eigen::Matrix4f mat = stereoCloudStruct.transform.at(0);
    pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
   //  CropBox(filtered,filtered,0.65,0.3, -0.2, 0.8,mat);
//    if(stereoCloudStruct.Clouds.erase(0) < 1) ROS_ERROR("Could not erase /bumblebeeLeft cloud from map!");
//    if(!stereoCloudStruct.Clouds.insert(std::pair<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >(0,PointCloudPCL.makeShared())).second){
//      ROS_ERROR("Could not insert /bumblebeeLeft cloud into map!");
//    }   
    StereoPointCloudPCLCombined += PointCloudPCL;
  }
  else if(received_frame.compare("/bumblebeeRight") == 0){
    stereoCloudStruct.stereo_received[1] = true;
    Eigen::Matrix4f mat = stereoCloudStruct.transform.at(1);
    pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
//    if(stereoCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /bumblebeeRight cloud from map!");
//    if(!stereoCloudStruct.Clouds.insert(std::pair<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >(1,PointCloudPCL.makeShared())).second){
//      ROS_ERROR("Could not insert /bumblebeeRight cloud into map!");
//    }  
    StereoPointCloudPCLCombined += PointCloudPCL;
  }
  else if(received_frame.compare("/bumblebeeCenter") == 0){
    stereoCloudStruct.stereo_received[2] = true;
    Eigen::Matrix4f mat = stereoCloudStruct.transform.at(2);
    pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
//    if(stereoCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /bumblebeeCenter cloud from map!");
//    if(!stereoCloudStruct.Clouds.insert(std::pair<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >(2,PointCloudPCL.makeShared())).second){
//      ROS_ERROR("Could not insert /bumblebeeCenter cloud into map!");
//    } 
    StereoPointCloudPCLCombined += PointCloudPCL;
  }
 
 if(stereoCloudStruct.stereo_received[0] == true && stereoCloudStruct.stereo_received[1] == true && stereoCloudStruct.stereo_received[2] == true)
 {
  //std::cout << "All Stereo frames received " << std::endl;
   stereoCloudStruct.stereo_received[0] = false;
   stereoCloudStruct.stereo_received[1] = false;
   stereoCloudStruct.stereo_received[2] = false;
   
/*   for(int i = 0; i== 3; i++)
   {
     filtered = stereoCloudStruct.Clouds[i].makeShared();
     PassThroughFilter(filtered,filtered,0.0,2.0,"z");
     stereoCloudStruct.Clouds[i] = *filtered;
   }
  */ 
   StereoPointCloudPCLCombined.header.frame_id = "/world";
   StereoPointCloudPCLCombined.header.stamp = ros::Time::now().toNSec();
   _pubAlignStereo.publish(StereoPointCloudPCLCombined);
   
 }
 
}

void kinectPointCloudSub(ros::NodeHandle nh, string name)
{

  std::stringstream PointCloudPath;
  PointCloudPath << name << "/depth_registered/points";

  point_cloud_vector.push_back(new ros::Subscriber());
  *point_cloud_vector.back() = nh.subscribe(PointCloudPath.str(), 1, kinectPointCloudCallback);

}

void stereoPointCloudSub(ros::NodeHandle nh, string name)
{

  std::stringstream PointCloudPath;
  //TODO: Change to this when running at MARVIN PointCloudPath << name << "/points";
  PointCloudPath << name << "/depth_registered/points";

  point_cloud_vector.push_back(new ros::Subscriber());
  *point_cloud_vector.back() = nh.subscribe(PointCloudPath.str(), 1, stereoPointCloudCallback);

}

int main(int argc, char **argv)
{
	 ros::init(argc, argv, "cloud_merge");
	 ros::NodeHandle nodeHandle = ros::NodeHandle("~");

//	 kinectPointCloudSub(nodeHandle, "/kinect_left");
//	 kinectPointCloudSub(nodeHandle, "/kinect_right");
//	 kinectPointCloudSub(nodeHandle, "/kinect_center");
//	 stereoPointCloudSub(nodeHandle, "/bumblebeeRight");
//	 stereoPointCloudSub(nodeHandle, "/bumblebeeLeft");
//	 stereoPointCloudSub(nodeHandle, "/bumblebeeCenter");

	 //Publisher
        _pubAlignKinect = pcl_ros::Publisher<PointT> (nodeHandle, "kinect_aligned", 1);
	_pubAlignStereo = pcl_ros::Publisher<PointT> (nodeHandle, "stereo_aligned", 1);
	_pubAlignFull = pcl_ros::Publisher<PointT> (nodeHandle, "all_aligned", 1);
	_pubKinectLeft = pcl_ros::Publisher<PointT> (nodeHandle, "kinect_left_global", 1);
	_pubKinectRight = pcl_ros::Publisher<PointT> (nodeHandle, "kinect_right_global", 1);
	_pubKinectCenter = pcl_ros::Publisher<PointT> (nodeHandle, "kinect_center_global", 1);
	_pubBBLeft = pcl_ros::Publisher<PointT> (nodeHandle, "bumblebeeLeft_left_global", 1);
	_pubBBRight = pcl_ros::Publisher<PointT> (nodeHandle, "bumblebeeRight_right_global", 1);
	_pubBBCenter = pcl_ros::Publisher<PointT> (nodeHandle, "bumblebeeCenter_center_global", 1);
	_model_srv = nodeHandle.advertiseService("create_model", create_model_callback);
        _crop_box_marker_publisher = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	 tf::TransformListener tf_listener;
	 
	 Eigen::Affine3d mat; Eigen::Matrix4d eig_mat;  Eigen::Matrix4f transformation;
	 kinectCloudStruct.transform.clear();
	 stereoCloudStruct.transform.clear();
	   
	 try
	 {
	    tf::StampedTransform transform_kinect_left;
	    if(tf_listener.waitForTransform("/world", "/kinect_left",ros::Time(0),ros::Duration(3.0))){
	      tf_listener.lookupTransform("/world", "/kinect_left",ros::Time(0),transform_kinect_left);
	      tf::transformTFToEigen(transform_kinect_left,mat);
	      eig_mat =  mat.matrix(); transformation = eig_mat.cast<float>();
	      kinectCloudStruct.transform.push_back(transformation);
	      std::cout << "kinect_left to world: " << std::endl;
	      std::cout << transformation << std::endl;
	  }else
	    ROS_WARN("Failed to get /kinect_left transform!");
	  
	    tf::StampedTransform transform_kinect_right;
	    if(tf_listener.waitForTransform("/world", "/kinect_right",ros::Time(0),ros::Duration(3.0))){
	      tf_listener.lookupTransform("/world", "/kinect_right",ros::Time(0),transform_kinect_right);
	      tf::transformTFToEigen(transform_kinect_right,mat);
	      eig_mat =  mat.matrix(); transformation = eig_mat.cast<float>();
	      kinectCloudStruct.transform.push_back(transformation);
	      std::cout << "kinect_right to world: " << std::endl;
	      std::cout << transformation << std::endl;
	    }else
	      ROS_WARN("Failed to get /kinect_right transform!");
	 
	    tf::StampedTransform transform_kinect_center;
	    if(tf_listener.waitForTransform("/world", "/kinect_center",ros::Time(0),ros::Duration(3.0))){
	      tf_listener.lookupTransform("/world", "/kinect_center",ros::Time(0),transform_kinect_center);
	      tf::transformTFToEigen(transform_kinect_center,mat);
	      eig_mat =  mat.matrix(); transformation = eig_mat.cast<float>();
	      kinectCloudStruct.transform.push_back(transformation);
	      std::cout << "kinect_center to world: " << std::endl;
	      std::cout << transformation << std::endl;
	    }else
	      ROS_WARN("Failed to get /kinect_center transform!");
	    
	    tf::StampedTransform transform_stereo_left;
	    if(tf_listener.waitForTransform("/world", "/bumblebeeLeft",ros::Time(0),ros::Duration(3.0))){
	      tf_listener.lookupTransform("/world", "/bumblebeeLeft",ros::Time(0),transform_stereo_left);
	      tf::transformTFToEigen(transform_stereo_left,mat);
	      eig_mat =  mat.matrix(); transformation = eig_mat.cast<float>();
	      stereoCloudStruct.transform.push_back(transformation);
	      std::cout << "bumblebeeLeft to world: " << std::endl;
	      std::cout << transformation << std::endl;
	    }else
	      ROS_WARN("Failed to get /bumblebeeLeft transform!");
	    
	 
	    tf::StampedTransform transform_stereo_right;
	    if(tf_listener.waitForTransform("/world", "/bumblebeeRight",ros::Time(0),ros::Duration(3.0))){
	      tf_listener.lookupTransform("/world", "/bumblebeeRight",ros::Time(0),transform_stereo_right);
	      tf::transformTFToEigen(transform_stereo_right,mat);
	      eig_mat =  mat.matrix(); transformation = eig_mat.cast<float>();
	      stereoCloudStruct.transform.push_back(transformation);
	      std::cout << "bumblebeeRight to world: " << std::endl;
	      std::cout << transformation << std::endl;
	    }else
	      ROS_WARN("Failed to get /bumblebeeRight transform!");
	    
	 
	    tf::StampedTransform transform_stereo_center;
	    if(tf_listener.waitForTransform("/world", "/bumblebeeCenter",ros::Time(0),ros::Duration(3.0))){
	      tf_listener.lookupTransform("/world", "/bumblebeeCenter",ros::Time(0),transform_stereo_center);
	      tf::transformTFToEigen(transform_stereo_center,mat);
	      eig_mat =  mat.matrix(); transformation = eig_mat.cast<float>();
	      stereoCloudStruct.transform.push_back(transformation);
	      std::cout << "bumblebeeCenter to world: " << std::endl;
	      std::cout << transformation << std::endl;
	    }else
	      ROS_WARN("Failed to get /bumblebeeCenter transform!");
	    
	    
	  }
	  catch (tf::TransformException &ex)
	  {
	    ROS_ERROR("%s", ex.what());
	  }
	
	 ros::spin();
	
	return 0;
}


