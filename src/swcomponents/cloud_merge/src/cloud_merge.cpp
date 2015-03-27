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
#include <pcl/filters/statistical_outlier_removal.h>
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

#include <omp.h>


using namespace std;
using namespace tr1;
using namespace sensor_msgs;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

vector<ros::Subscriber*> point_cloud_vector;
//std::vector<double> plane_coeff;

CloudT PointCloudPCLCombined;
CloudT KinectPointCloudPCLCombined;
PointCloudNT StereoPointCloudPCLCombined;

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

struct SegmentationParams{ 
  SegmentationParams():
      voxel_resolution(0.0035f), //-v
      seed_resolution(0.022f), //-s
      color_importance(2.0f), //-c
      spatial_importance(5.0f), //-z
      normal_importance (8.0f), // -n
      use_single_cam_transform (false), // -tvoxel  Kinect = true
      use_supervoxel_refinement (false), // -refine
  
    // LCCPSegmentation Stuff
      concavity_tolerance_threshold (13), //-ct
      smoothness_threshold (0.1), //-st
      min_segment_size (10), //-smooth
      use_extended_convexity (true), //-ec
      use_sanity_criterion (true) {} //-sc
  
  float voxel_resolution; //-v
  float seed_resolution; //-s
  float color_importance; //-c
  float spatial_importance; //-z
  float normal_importance;  // -n
  bool use_single_cam_transform; // -tvoxel  Kinect = true
  bool use_supervoxel_refinement; // -refine
  
    // LCCPSegmentation Stuff
  float concavity_tolerance_threshold; //-ct
  float smoothness_threshold; //-st
  uint32_t min_segment_size; //-smooth
  bool use_extended_convexity; //-ec
  bool use_sanity_criterion; //-sc
  
};

pcl_ros::Publisher<PointT> _pubAlignKinect;
pcl_ros::Publisher<PointNT> _pubAlignStereo;
pcl_ros::Publisher<PointT> _pubAlignFull;

pcl_ros::Publisher<PointT> _pubKinectLeft;
pcl_ros::Publisher<PointT> _pubKinectRight;
pcl_ros::Publisher<PointT> _pubKinectCenter;

pcl_ros::Publisher<PointT> _pubBBLeft;
pcl_ros::Publisher<PointT> _pubBBRight;
pcl_ros::Publisher<PointT> _pubBBCenter;

ros::ServiceServer _model_srv;

void computePlaneCoeff(CloudT::Ptr &src_cloud,pcl::ModelCoefficients::Ptr coefficients, double dist_threads)
{
   //*********************************************************************//
   //	Plane fitting
   /**********************************************************************/
    
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
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

   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                       << coefficients->values[1] << " "
                                       << coefficients->values[2] << " "
                                       << coefficients->values[3] << std::endl;



}

bool extractLargestClusters(CloudT::Ptr &src_cloud, CloudT::Ptr largest_clusters, double cluster_tol)
{
    
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
   
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (cluster_tol); // 2cm
    ec.setMinClusterSize (50); //100
    ec.setMaxClusterSize (800000); //50000
    ec.setSearchMethod (tree);
    ec.setInputCloud(src_cloud);
    ec.extract (cluster_indices); 
    
     std::cout<<"Found "  << cluster_indices.size() << " clusters" << std::endl;
     unsigned int largest_cloud = 0;
     
    if ( cluster_indices.size() < 1) return false;
     
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){   
	CloudT::Ptr cloud_cluster (new CloudT);
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back (src_cloud->points[*pit]); //*

	    if(cloud_cluster->points.size () > largest_cloud){ //clusterThreadshold = 500{
	     	 cloud_cluster->width = cloud_cluster->points.size ();
	     	 cloud_cluster->height = 1;
	     	 cloud_cluster->is_dense = true;
		 largest_cloud = cloud_cluster->points.size ();
		 
		 pcl::copyPointCloud(*cloud_cluster, *largest_clusters);
	     	 
		 std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

	     }     
	  
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

bool moveObjectFrame(CloudT::Ptr &src_cloud, CloudT::Ptr &tar_cloud){
  
 	Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*src_cloud,centroid);
	
	//  pcl::transformPointCloud(*src_cloud,*temp,mat);
		 
	pcl::PCA<PointT> _pca; 
	PointT projected; 
	PointT reconstructed;
	CloudT cloudi = *src_cloud;
	CloudT finalCloud;
		 
	try{
	     //Do PCA for each point to preserve color information
	     //Add point cloud to force PCL to init_compute else a exception is thrown!!!HACK
	     _pca.setInputCloud(src_cloud);
	     int i;
	 //    #pragma omp parallel for
	     for(i = 0; i < (int)src_cloud->size(); i++)     {
	       _pca.project(cloudi[i],projected);
	       Eigen::Matrix3f eigen = _pca.getEigenVectors();
	       
	       // flip axis to satisfy right-handed system
              if (eigen.col(0).cross(eigen.col(1)).dot(eigen.col(2)) < 0) {
		        projected.z = -projected.z; //Avoid flipping the model
              }
              
	       _pca.reconstruct (projected, reconstructed);

	       pcl::PCLPointCloud2 c;
	       pcl::toPCLPointCloud2(cloudi,c);
	       if(pcl::getFieldIndex(c,"rgba") >= 0){
		 //assign colors
		 projected.r = cloudi[i].r;
		 projected.g = cloudi[i].g;
		 projected.b = cloudi[i].b;
	       }
	       //add point to cloud
	       finalCloud.push_back(projected);
	       
	    }
	    
	}catch(pcl::InitFailedException &e){
	  PCL_ERROR(e.what());
	  
	}


//	   pcl::io::savePCDFile("/home/thso/pca_cloud.pcd",finalCloud);	
	pcl::copyPointCloud(finalCloud,*tar_cloud);
  
}

bool LCCPSegmentation(CloudT::Ptr &src_cloud,pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr,
		      std::vector<CloudT, Eigen::aligned_allocator_indirection<CloudT> > &clusters,
		      SegmentationParams params, bool has_normals, int view_nr ){
  
   ///Compute plane coefficients
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  computePlaneCoeff(src_cloud,coefficients,0.010);
  
   if(!has_normals)
     PCL_WARN ("Could not find normals point cloud. Normals will be calculated. This only works for single-camera-view pointclouds.\n");
  /// Supervoxel parameters
/*  float voxel_resolution = params.voxel_resolution;
  float seed_resolution = params.seed_resolution;
  float color_importance = params.color_importance;
  float spatial_importance = params.spatial_importance;
  float normal_importance = params.normal_importance;
  bool use_single_cam_transform = params.use_single_cam_transform; //Kinect = true
  bool use_supervoxel_refinement = params.use_supervoxel_refinement;
  */   
     
  float voxel_resolution = 0.0035f;
  float seed_resolution = 0.020f;
  float color_importance = 2.0f;
  float spatial_importance = 5.0f;
  float normal_importance = 8.0f;
  bool use_single_cam_transform = false; //Kinect = true
  bool use_supervoxel_refinement = true;
  
  
   /// LCCPSegmentation Variables
/*  float concavity_tolerance_threshold = params.concavity_tolerance_threshold; //30
  float smoothness_threshold = params.smoothness_threshold;
  uint32_t min_segment_size = params.min_segment_size;
  bool use_extended_convexity = params.use_extended_convexity;
  bool use_sanity_criterion = params.use_sanity_criterion;
 */ 
  float concavity_tolerance_threshold = 15.0f; //30
  float smoothness_threshold = 0.1f;
  uint32_t min_segment_size = 10;
  bool use_extended_convexity = false;
  bool use_sanity_criterion = true;
  
 ///------------------------------------------------- Supervoxel computation ------------------------------------------------------
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
  
 
  
  uint k_factor = 0;
 // if (use_extended_convexity)
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
  
    std::stringstream ss;
    ss << "/home/thso/original_labled_cloud_" << view_nr; ss << ".pcd";
    pcl::io::savePCDFile(ss.str(),*lccp_labeled_cloud);	 
 // pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList sv_adjacency_list;
 // lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization
  
   /// Creating Colored Clouds and Output
  if (lccp_labeled_cloud->size () == src_cloud->size ()){
    
   //  if (pcl::getFieldIndex (*src_cloud, "label") >= 0)
   //       PCL_WARN ("Input cloud already has a label field. It will be overwritten by the lccp segmentation output.\n");
  
      ///Get all LCCP segments
      std::map<uint32_t, std::vector<uint32_t> > segment_supervoxel_map;   
      lccp.getSegmentSupervoxelMap(segment_supervoxel_map);
      if(segment_supervoxel_map.empty()){ 
	 pcl::console::print_error("ERROR: Failed to get segmented supervoxel map for LCCP algorithm!");
	 return false;
      }
        
      std::map<uint32_t, pcl::PointIndices::Ptr> segment_indice;
  /*    for(uint32_t i = 0; i< _num_of_segment; i++){
	std::cout << "i: " << i << std::endl;
	 pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	 segment_indice.insert(std::pair<uint32_t, pcl::PointIndices::Ptr>(i,inliers));
      }
    */  
      ///Iterate through LCCP labled cloud to copy label to inlier map
      pcl::PointCloud<pcl::PointXYZL>::iterator cloud_iter;
      std::map<uint32_t, uint32_t> pointcloud_label_to_new_label;
      int count = 0;
      for (cloud_iter = lccp_labeled_cloud->begin(); cloud_iter != lccp_labeled_cloud->end(); cloud_iter++) {
	//TODO: Kig på om label er ny. hvis: lig gamle lable i map på 1 til antal segmenters plads
	  uint32_t label =cloud_iter->label; 
	  //if(label != 1) std::cout << label << " " << std::endl;
	  if(pointcloud_label_to_new_label.count(label) == 0){ //The label dosen't exist
	     pcl::console::print_highlight("adding original label %d as label %d in inlier map\n",label, count);
	     ///Initialize map to store inliers(segments)
	     ///(Create new inlier object to hold the segment points)
	     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	     segment_indice.insert(std::pair<uint32_t, pcl::PointIndices::Ptr>(count,inliers));
             
	     pointcloud_label_to_new_label.insert(std::pair<uint32_t,uint32_t>(label, count));
	     count++;
	  }
	  //label already exist. get the label position from pointcloud_label_to_new_label
	   uint32_t index = pointcloud_label_to_new_label.find(label)->second;
	//   std::cout << "Save label :" << pointcloud_label_to_new_label.find(label)->first << " at index: " << index << std::endl;
	
	    //Map label to point
	    segment_indice.at(index)->indices.push_back(cloud_iter - lccp_labeled_cloud->begin());
	  
      }
      pcl::console::print_highlight("Cloud has %d LCCP segments\n", segment_indice.size());
   //   pcl::console::print_highlight("Number of labels : %d\n", segment_indice.size());
      
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
	  std::map<uint32_t, pcl::PointIndices::Ptr >::iterator seg_iter;
	//  std::cout << "segment_supervoxel_map size: " << segment_supervoxel_map.size() << std::endl;
	  for(seg_iter = segment_indice.begin(); seg_iter != segment_indice.end(); seg_iter++) {
		///First get the label
		uint32_t segment_label = seg_iter->first;
		std::cout << "-------------------Segment " << segment_label << " --------------------------" << std::endl;
	/*	std::vector<uint32_t> supervoxels_in_segment = seg_iter->second;
	           pcl::console::print_highlight("Number of supervoxels in segment: %d\n ",supervoxels_in_segment.size());
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
		  */
		 // std::cout << " samples: " << samples << std::endl;
		 // std::cout << " theta_acc: " << theta_avg << std::endl;
		//  pcl::console::print_highlight(" theta_avg: %f\n", theta_avg/samples);
		//  theta_avg = theta_avg/samples;
		  
		
		  pcl::PointIndices::Ptr inliers = segment_indice.at(segment_label);
		  if (inliers->indices.size () != 0)
		  {
		      extract.setIndices (inliers);
		      extract.setNegative (false);
		      extract.filter (*segment_cloud);
		      
		      pcl::console::print_highlight("%d points in segment\n",segment_cloud->points.size());
		      ///Only consider clouds larger than 10 points
		      if(segment_cloud->points.size() > 10)
		      { 
		        //Estimate the plane coefficients
			 pcl::ModelCoefficients::Ptr seg_coefficients (new pcl::ModelCoefficients);
			 seg.setInputCloud (segment_cloud);
			 seg.segment (*inliers_sac, *seg_coefficients);
			 
			 if (inliers_sac->indices.size () == 0)
				PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	      
			 //coefficients.reset();
			 pcl::PointNormal segment_normal;
			 segment_normal.normal_x = seg_coefficients->values[0];
			 segment_normal.normal_y = seg_coefficients->values[1];
			 segment_normal.normal_z = seg_coefficients->values[2];
			 
			 pcl::PointNormal plane_normal;
			 plane_normal.normal_x = coefficients->values[0];
			 plane_normal.normal_y = coefficients->values[1];
			 plane_normal.normal_z = coefficients->values[2];
			 
			 float seg_radius, seg_theta,  seg_phi = 0;
		         toSpherical(segment_normal, seg_radius, seg_theta, seg_phi); 
			 std::cout <<"Segment - radius: " << seg_radius << " theta: " << seg_theta << " phi: " << seg_phi << std::endl;
		   
			 float plane_radius, plane_theta,  plane_phi = 0;
			 toSpherical(plane_normal, plane_radius, plane_theta, plane_phi); 
			 std::cout <<"table plane - radius: " << plane_radius << " theta: " << plane_theta << " phi: " << plane_phi << std::endl;
		   
			  //Compute centroid 
			  Eigen::Vector4d centroid;
			  pcl::compute3DCentroid(*segment_cloud,centroid);
			  
			  //Dertermine if the segment lies on the table plane solve ax * by *cz = d 
			  float value = coefficients->values[0] * centroid[0] + coefficients->values[1] * centroid[1] + coefficients->values[2] * centroid[2];
			  std::cout << "value: " << value << std::endl;
			  std::cout << "d: " << coefficients->values[3] << std::endl;
			  std::cout << "diff: " << coefficients->values[3] - std::abs(value)  << std::endl;
			  
			  pcl::PointCloud<pcl::PointXYZL>::Ptr temp_label_cloud (new pcl::PointCloud<pcl::PointXYZL> );
			
			  std::cout << "normal diff: " << std::abs(plane_theta -seg_theta) << std::endl;
			   
			  if((std::abs(coefficients->values[3] - std::abs(value)) > 0.001) &&
			     (std::abs(plane_theta -seg_theta) > 0.005))
			  {
			  //  computePrincipalCurvature(segment_cloud)
			   CloudT::Ptr temp (new CloudT);
			            
			  //Deep copy
			  *temp = *segment_cloud; 
			  label_cloud(segment_cloud,temp_label_cloud, 1);
			  *labled_cloud += *temp_label_cloud;
			  PCL_WARN("Valid cluster found!\n");
			  // std::stringstream ss;
			  // std::cout << "Saving segment " << label <<  "..."<< std::endl;
			  // ss << "/home/thso/segment_" << label; ss << ".pcd";
			  // pcl::io::savePCDFile(ss.str(),*segment_cloud);
		    
			  clusters.push_back(*temp);
			 }else{
			    label_cloud(segment_cloud,temp_label_cloud, 2);
			   *labled_cloud += *temp_label_cloud;
			 }
			 
			  
		      }else{
			PCL_WARN("To few points in segment = %d\n", segment_cloud->points.size());
		      }
		      
		  }else{
		      PCL_ERROR("No inliers!!!!\n");
		  }
		
	
	 segment_cloud->clear();
	
	  }
	 if(labled_cloud->points.size() > 0){
	   std::stringstream ss;
	   ss << "/home/thso/labled_cloud_" << view_nr; ss << ".pcd";
	   pcl::io::savePCDFile(ss.str(),*labled_cloud);	  
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

void 	radiusOutlierRemoval(CloudT::Ptr &src_cloud, CloudT::Ptr &target_cloud, double radius, int min_neighbpr_pts)
{
	CloudT::Ptr cloud_filtered (new CloudT);

	if(src_cloud->size() > 0)
	{
	    try{
	      // Create the filtering object
	      pcl::RadiusOutlierRemoval<PointT> ror;
	      ror.setInputCloud(src_cloud);
	      ror.setRadiusSearch(radius);
	      ror.setMinNeighborsInRadius(min_neighbpr_pts);
	      ror.filter (*cloud_filtered);
	      ror.setKeepOrganized(true);

	      pcl::copyPointCloud(*cloud_filtered, *target_cloud);
	    }catch(...)
	    {
	      PCL_ERROR("Somthing went wrong in object_modeller::radiusOutlierRemoval()");
	    }
	}
}


void statisticalOutlierRemoval(CloudT::Ptr &src_cloud, CloudT::Ptr &target_cloud, int mean)
{
	 CloudT::Ptr cloud_filtered (new CloudT);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (src_cloud);
	sor.setMeanK (mean);
	sor.setStddevMulThresh (1.0);

	sor.filter (*cloud_filtered);

  	pcl::copyPointCloud(*cloud_filtered, *target_cloud);

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

void MLSApproximation(CloudT::Ptr &cloud, PointCloudNT::Ptr &target, double search_radius)
{
	 using namespace pcl::console;
	 
	 // Create a KD-Tree
	  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

	  // Output has the PointNormal type in order to store the normals calculated by MLS
	  PointCloudNT::Ptr mls_points (new PointCloudNT);

	  // Init object (second point type is for the normals, even if unused)
	  pcl::MovingLeastSquares<PointT, PointNT> mls;

	  mls.setComputeNormals (true);

	  // Set parameters
	  mls.setInputCloud (cloud);
	  mls.setPolynomialFit (true);
	  mls.setSearchMethod (tree);
	  mls.setSearchRadius (search_radius); //0.025
	  mls.setDilationVoxelSize(0.001);
	//  mls.setPointDensity(0.0005);
  
         // mls.setPolynomialOrder(4); 
	
	  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointNT>::VOXEL_GRID_DILATION);
	 // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::UpsamplingMethod::VOXEL_GRID_DILATION);

	  // Reconstruct
	  TicToc tt;
	  tt.tic ();
	  print_highlight("Computing smoothed point cloud using MLS algorithm....");
	  mls.process (*mls_points);
	  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

	  pcl::copyPointCloud(*mls_points, *target);

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

  std::vector<int> dummy;
       
  /// Allocate point clouds
  PointCloudNT::Ptr model_with_normals(new PointCloudNT);
  PointCloudNT::Ptr final_cloud (new PointCloudNT);
  
  SegmentationParams params;
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
     Eigen::Matrix4f mat_inv = mat.inverse();
     pcl::transformPointCloud(*filtered,*filtered,mat_inv);
  //   publish_marker( rx,ry,minz,maxz, mat);
	
     ROS_INFO("Saving view %i to hard drive!!", i);
     std::stringstream sst;
     sst << "/home/thso/view_" << i; sst << ".pcd";
     if(filtered->points.size() > 0) pcl::io::savePCDFile(sst.str(), *filtered  );
    
	 
     pcl::PointCloud<pcl::Normal>::Ptr dummy_ptr;
     LCCPSegmentation(filtered, dummy_ptr, clusters,params,false, i);
     
      //Find the correct models
     PCL_INFO("Selecting cluster for view %i ......\n", i);
     for(int j = 0; j < clusters.size(); j++){
	 ROS_INFO("Selecting cluster with %d points!!\n", (int)clusters[j].points.size() );
         *out += clusters.at(j);
     }
     clusters.clear();
     
      statisticalOutlierRemoval(out,out, 3);
      radiusOutlierRemoval(out, out,0.01, 40);
      extractLargestClusters(out,out,0.02);
      
      radiusOutlierRemoval(out, out,1.0, 800);
      MLSApproximation(out, model_with_normals,0.0075);

     pcl::transformPointCloudWithNormals<PointNT>(*model_with_normals,*final_cloud,mat);
     pcl::removeNaNFromPointCloud(*final_cloud,*final_cloud,dummy);
     
       ROS_INFO("Saving model segment %i to hard drive!!", i);
       std::stringstream ss;
	ss << "/home/thso/model_segment_" << i; ss << ".pcd";
       if(final_cloud->points.size() > 0) pcl::io::savePCDFile(ss.str(), *final_cloud  );
     
     if(req.sensor == 0){// KinectPointCloudPCLCombined += *final_cloud;
     }else if(req.sensor == 1){ StereoPointCloudPCLCombined += *final_cloud;}
     
     ss.str("");
     ss << "/home/thso/selected_cluster_" << i; ss << ".pcd";
  
     if(final_cloud->points.size() > 0)pcl::io::savePCDFile(ss.str(), *final_cloud  );      
     else{
       PCL_ERROR("No points in segmented model!");
     }
     out->clear();
     filtered->clear();
     final_cloud->clear();
   }
   
   CloudT::Ptr model(new CloudT);    
   sensor_msgs::PointCloud2 cloud_model;
   
   //Sample model to remove redundant points 
   pcl::VoxelGrid<PointT> vg;
   vg.setLeafSize(0.001, 0.001, 0.001);
   
   try{
	  if(req.sensor == 0){
	    pcl::copyPointCloud(KinectPointCloudPCLCombined, *model);
	    moveObjectFrame(model,model);
	    pcl::io::savePCDFile("/home/thso/final_kinect_model.pcd",*model);
	    pcl::toROSMsg(*model,cloud_model);
	  }else if(req.sensor == 1){
	    pcl::copyPointCloud(StereoPointCloudPCLCombined, *model);
	    moveObjectFrame(model,model);
	    vg.setInputCloud(model);
	    vg.filter(*model);
	    pcl::io::savePCDFile("/home/thso/final_stereo_model.pcd", *model);
	    pcl::toROSMsg(*model,cloud_model);
	  }
   }catch(pcl::IOException &e){
	std::cerr << e.what() << std::endl; 
   }
  
  KinectPointCloudPCLCombined.clear();
  StereoPointCloudPCLCombined.clear();
  
  _pubAlignKinect.publish(*model);
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

  ROS_INFO	("Create new model request with sensor %d\n", req.sensor);
  
  switch(req.sensor)
  {      
    case 0: //Kinect sensor
   {   
	cloud_left = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_left/depth_registered/points", ros::Duration(5.0));
	 if(cloud_left) {
	    pcl::fromROSMsg < PointT > (*cloud_left, PointCloudPCL);
	    mat = kinectCloudStruct.transform.at(0);
	    pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	
	    if(kinectCloudStruct.Clouds.erase(0) < 1) ROS_ERROR("Could not erase /kinect_left cloud from map!");
	    if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(0,PointCloudPCL.makeShared())).second){
	      ROS_ERROR("Could not insert /kinect_left cloud into map!");
	    }
	 }else{
	    ROS_WARN("Retrieval of /kinect_left/depth_registered/points point cloud failed!");
	  return false;
	 }
	 
	cloud_right = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_right/depth_registered/points", ros::Duration(5.0));
	if(cloud_right){
	    pcl::fromROSMsg < PointT > (*cloud_right, PointCloudPCL);
	    mat = kinectCloudStruct.transform.at(1);
	    pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	
	    if(kinectCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /kinect_right cloud from map!");
	    if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(1,PointCloudPCL.makeShared())).second){
		ROS_ERROR("Could not insert /kinect_right cloud into map!");  
	    }
	 }else{
	  ROS_WARN("Retrieval of /kinect_right/depth_registered/points point cloud failed!");
	  return false;
	}
	
	cloud_center = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect_center/depth_registered/points", ros::Duration(5.0));
	if(cloud_center){
	    pcl::fromROSMsg <PointT > (*cloud_center, PointCloudPCL);
	    mat = kinectCloudStruct.transform.at(2);
	    pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	    if(kinectCloudStruct.Clouds.erase(2) < 1) ROS_ERROR("Could not erase /kinect_center cloud from map!");
	    if(!kinectCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(2,PointCloudPCL.makeShared())).second){
	      ROS_ERROR("Could not insert /kinect_center cloud into map!"); 
	  }
	}else{
	  ROS_WARN("Retrieval of /kinect_center/depth_registered/points point cloud failed!");
	  return false;
	}
    
     /// All Kinect clouds are received. Lets process the clouds
      process_cloud(req,res);
      
      std::cout << "Extracted model has " << res.model.data.size() << " points"<< std::endl;
      
      break;
   }  
      
    case 1: //Stereo sensor
   {
	  cloud_left = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/bumblebeeLeft/depth_registered/points", ros::Duration(15.0));
	  if(cloud_left) {
	    std::cout << "points received: " << cloud_left->width * cloud_left->height << std::endl;
	    pcl::fromROSMsg < PointT > (*cloud_left, PointCloudPCL);
 
	    mat = stereoCloudStruct.transform.at(0);
	    pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);
	    if(stereoCloudStruct.Clouds.erase(0) < 1) ROS_ERROR("Could not erase /bumblebeeLeft cloud from map!");
	    if(!stereoCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(0,PointCloudPCL.makeShared())).second){
	      ROS_ERROR("Could not insert /bumblebeeLeft cloud into map!");
	    }
	  }else{
	    ROS_WARN("Retrieval of /bumblebeeLeft/points point cloud failed!");
	    return false;
	  }
	  
	  
	cloud_right = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/bumblebeeRight/depth_registered/points", ros::Duration(15.0));
	if(cloud_right) {
	  std::cout << "points received: " << cloud_right->width * cloud_right->height << std::endl;
	  pcl::fromROSMsg < PointT > (*cloud_right, PointCloudPCL);
	  mat = stereoCloudStruct.transform.at(1);
	  pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);

	  if(stereoCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /bumblebeeRight cloud from map!");
	  if(!stereoCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(1,PointCloudPCL.makeShared())).second){
	    ROS_ERROR("Could not insert /bumblebeeRight cloud into map!");  
	  }
	}else{
	   ROS_WARN("Retrieval of /bumblebeeRight/points point cloud failed!");
	   return false;
	}
	
	cloud_center = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/bumblebeeCenter/depth_registered/points", ros::Duration(15.0));
	if(cloud_center) {
	  std::cout << "points received: " << cloud_center->width * cloud_center->height << std::endl;
	  pcl::fromROSMsg <PointT > (*cloud_center, PointCloudPCL);  
	  mat = stereoCloudStruct.transform.at(2);
	  pcl::transformPointCloud(PointCloudPCL,PointCloudPCL,mat);

	  if(stereoCloudStruct.Clouds.erase(2) < 1) ROS_ERROR("Could not erase /bumblebeeCenter cloud from map!");
	  if(!stereoCloudStruct.Clouds.insert(std::pair<int, CloudT::Ptr >(2,PointCloudPCL.makeShared())).second){
	    ROS_ERROR("Could not insert /bumblebeeCenter cloud into map!"); 
	  }
	}else{
	  ROS_WARN("Retrieval of /bumblebeeCenter/points point cloud failed!");
	  return false;
	}
	
        /// All stereo clouds are received. Lets process the clouds
	process_cloud(req,res);
	std::cout << "Extracted model has " << res.model.data.size() << " points"<< std::endl;
      
      break;
   }
   
    case 2: //Both sensors
   {
	  ROS_WARN("cloud_merge not implemented yet for stereo+kinect!");
      break;
   }
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
  PointCloudNT PointCloudPCL;
  pcl::fromROSMsg < PointNT > (PointCloud, PointCloudPCL);
  
   if(received_frame.compare("/bumblebeeLeft") == 0){
    stereoCloudStruct.stereo_received[0] = true;
    Eigen::Matrix4f mat = stereoCloudStruct.transform.at(0);
    pcl::transformPointCloudWithNormals<PointNT>(PointCloudPCL,PointCloudPCL,mat);
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
    pcl::transformPointCloudWithNormals<PointNT>(PointCloudPCL,PointCloudPCL,mat);
//    if(stereoCloudStruct.Clouds.erase(1) < 1) ROS_ERROR("Could not erase /bumblebeeRight cloud from map!");
//    if(!stereoCloudStruct.Clouds.insert(std::pair<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >(1,PointCloudPCL.makeShared())).second){
//      ROS_ERROR("Could not insert /bumblebeeRight cloud into map!");
//    }  
    StereoPointCloudPCLCombined += PointCloudPCL;
  }
  else if(received_frame.compare("/bumblebeeCenter") == 0){
    stereoCloudStruct.stereo_received[2] = true;
    Eigen::Matrix4f mat = stereoCloudStruct.transform.at(2);
    pcl::transformPointCloudWithNormals<PointNT>(PointCloudPCL,PointCloudPCL,mat);
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
	_pubAlignStereo = pcl_ros::Publisher<PointNT> (nodeHandle, "stereo_aligned", 1);
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


