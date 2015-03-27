#include <one_shot_learning/ObjectModeller.hpp>

//--------------PCL Includes-------------------
//#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>
#include <pcl/exceptions.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <qcoreapplication.h>

namespace dti{
namespace one_shot_learning {
  
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
  
ObjectModeller::ObjectModeller(SharedData *data, QObject *parent) 
      : _sharedData(data){
 // start();
  leaf_size_ = 0.005f;
  doModel = false;
  doSolidModel = false;
  doModelAlignment = false; 
  _post_processed_cloud.reset(new PointCloudT);
  _raw_model.reset(new PointCloudT);
}

ObjectModeller::~ObjectModeller() {

}

void ObjectModeller::shutdown(){
  //isRunning = false;
}

void ObjectModeller::createmodel(PointCloudT in)
{
   PointCloudT::Ptr src (new PointCloudT);
   PointCloudT::Ptr tar (new PointCloudT);
   
   PointCloudNT::Ptr srcNT (new PointCloudNT);
   PointCloudNT::Ptr tarNT (new PointCloudNT);
   
   
   PointCloudT::Ptr cloud = in.makeShared();
   
   std::vector<int> dummy;
   pcl::removeNaNFromPointCloud(*cloud, *cloud, dummy);
   
   //EstimateNormals(cloud,src);
   pcl::copyPointCloud(*cloud,*_raw_model);

  ///Concatenate the XYZRGBA and normal fields*
//  pcl::copyPointCloud(*_cloud, *_point_normals);
  views.push_back(cloud);
  //src->clear();
  
  if(views.size() > 1){
    Eigen::Matrix4f initial_transformation, final_transformation;
    src = views[0];
    tar = views[1];
    pcl::copyPointCloud(*src,*srcNT);
    pcl::copyPointCloud(*tar,*tarNT);
    
    pcl::io::savePCDFile("/home/thso/full_model0.pcd",*src);
    pcl::io::savePCDFile("/home/thso/full_model1.pcd",*tar);
    
 /*   initial_alignment(srcNT, tarNT,initial_transformation,true,false, false);
    PointCloudT::Ptr new_source(new PointCloudT);
    PointCloudT::Ptr output(new   PointCloudT);
    pcl::transformPointCloud(*src, *new_source, initial_transformation);
    pcl::io::savePCDFile("/home/thso/ia0.pcd",*new_source);
    pcl::io::savePCDFile("/home/thso/ia1.pcd",*tar);
	    
    std::cout << "running ICP!!" << std::endl;
    pairAlign(new_source,tar,output,final_transformation,true);
    //Copy result to global pointCloud instance
    pcl::copyPointCloud(*output, *_raw_model);
    pcl::io::savePCDFile("/home/thso/registred_model.pcd",*output);
   */ 
    std::cout << "Done ...." << std::endl;
  }//else{
    Q_EMIT modelCreated();
  //}
    
}

void ObjectModeller::createSolidmodel(PointCloudT in)
{
/* PointCloudT::Ptr cloud = in.makeShared();
  
  //Create solid model from _cloud
  PointCloudNT::Ptr _normals(new PointCloudNT);
      
 pcl::io::savePCDFile("/home/thso/before_filtering.pcd",*cloud);
  radiusOutlierRemoval(cloud, _post_processed_cloud, 0.003, 8);
//  pcl::io::savePCDFile("/home/thso/radius_filter.pcd",*_post_processed_cloud);
//  statisticalOutlierRemoval(cloud, _post_processed_cloud,0.001);
  pcl::io::savePCDFile("/home/thso/radius_std_filter.pcd",*_post_processed_cloud);
 */
  //MLS algorithm cannot take a PointNT type -> gives rumtime error
 /* PointCloudT::Ptr d (new PointCloudT);
  d->points.resize(_post_processed_cloud->size());
    for (size_t i = 0; i < _post_processed_cloud->points.size(); i++) {
    d->points[i].x = _post_processed_cloud->points[i].x;
    d->points[i].y = _post_processed_cloud->points[i].y;
    d->points[i].z = _post_processed_cloud->points[i].z;
  }
 */
/*  reconstruct->MLSApproximation(_post_processed_cloud, _normals, 0.01);
  pcl::io::savePCDFile("/home/thso/mls_model.pcd",*_normals);
  */
  //reconstruct->BilateralUpsampling(_post_processed_cloud,_post_processed_cloud,3,5,0.2);
  // pcl::io::savePCDFile("/home/thso/upsampled_model.pcd",*_post_processed_cloud);
	  
//  EstimateNormals(_post_processed_cloud, _normals);  
  // Concatenate the XYZRGBA and normal fields*
//  pcl::copyPointCloud (*_post_processed_cloud, *_normals);
  
 // pcl::io::savePCDFile("/home/thso/before_shadow_filter.pcd",*_normals);
//  pcl::PointIndices::Ptr point_ind (new pcl::PointIndices);
 // reconstruct->ShadowFilter(_normals, _normals,0.005,point_ind);
 // pcl::io::savePCDFile("/home/thso/after_shadow_filter.pcd",*_normals);
 /* 
  reconstruct->poisson(_normals,_mesh,8,8,8,4.0f); //8,12,8,4.0f)
  reconstruct->saveToVTK("/home/thso/poisson_mesh.vtk", _mesh);
  */
//  pcl::PolygonMesh gpt_mesh;
//  reconstruct->GreedyProjectionTriangulation(_normals,gpt_mesh, 0.005);
//  reconstruct->saveToVTK("/home/thso/greedy_mesh.vtk", gpt_mesh);
  
//  pcl::PolygonMesh mc_mesh;
//  reconstruct->MarchingCubes(_normals,mc_mesh,35,0.8,50);
//  reconstruct->saveToVTK("/home/thso/mc_mesh.vtk", mc_mesh);
  
//  pcl::PolygonMesh gp_mesh;
//  reconstruct->GridProjection(_normals,gp_mesh,0.001);
//  reconstruct->saveToVTK("/home/thso/gp_mesh.vtk", gp_mesh);
  
  Q_EMIT solidModelCreated();

 // doSolidModel = true;
}

void ObjectModeller::alignGTModelAndSceneModel(PointCloudT src, PointCloudT tar)
{
 // _raw_model = src.makeShared();
 // _tar = tar.makeShared();
 // doModelAlignment = true;
}

void ObjectModeller::run(){
   std::cout << "Object modeller thread: " << QThread::currentThreadId() << std::endl;     

  //  reconstruct = new ReconstructPointCloud();
    if(doModel)
    {
      std::cout << "doModel" << std::endl;
       
//	  EstimateNormals(clusters[0], _normals);	
	  // Concatenate the XYZRGBA and normal fields*
//	  pcl::copyPointCloud (*clusters[0], *_normals);
      
     
	//  pcl::PolygonMesh mesh;
//	  reconstruct->poisson(_normals,mesh,8,6,32,4.0f); //8,12,8,4.0f)
	  // mesh = reconstruct->GreedyProjectionTriangulation(_new, 0.05);
	  //pcl::PolygonMesh mesh = reconstruct->MarchingCubes(_normals);
     
	  //reconstruct->saveToObj("mesh.obj", mesh);
	  //reconstruct->saveToVTK("mesh.vtk", mesh);
     
//	  views.push_back(clusters[0]);
/*	  if(views.size() > 1)
	  {
	    Eigen::Matrix4f initial_transformation, final_transformation;
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src = views[0];
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tar = views[1];
	    
	    initial_alignment(src, tar,initial_transformation,true);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_source(new   pcl::PointCloud<pcl::PointXYZRGBA>);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output(new   pcl::PointCloud<pcl::PointXYZRGBA>);
	    pcl::transformPointCloud (*src, *new_source, initial_transformation);
	    pcl::io::savePCDFile("ia0.pcd",*new_source);
	    pcl::io::savePCDFile("ia1.pcd",*tar);
	    
	    std::cout << "running ICP!!" << std::cout;
	    
	    pairAlign(new_source,tar,output,final_transformation,true);
	    pcl::io::savePCDFile("registred_model.pcd",*output);
	    
	    
	  }else
	  {
*/
//	  _sharedData->setFlagTurnModel();
	 //   Q_EMIT modelCreated();
	//    emit modelCreated(*clusters[0]);
//	  }
 //    }
      doModel = false;
    }
      
    if(doSolidModel)
    {
     
	  doSolidModel = false;
      
    }
    
    if(doModelAlignment)
    {	//Aligning GT_model and scene model to compute variance in coordinate system
   /*    Eigen::Matrix4f initial_transformation, final_transformation;
       initial_alignment(_raw_model, _tar,initial_transformation,true);
       PointCloudNT::Ptr new_source(new PointCloudNT);
       PointCloudNT::Ptr output(new   PointCloudNT);
       pcl::transformPointCloudWithNormals<PointNT> (*_raw_model, *new_source, initial_transformation);
       pcl::io::savePCDFile("gt_align0.pcd",*new_source);
       pcl::io::savePCDFile("gt_align1.pcd",*_tar);
	    
       std::cout << "running ICP!!" << std::cout;
	  
       pairAlign(new_source,_tar,output,final_transformation,true);
       pcl::io::savePCDFile("gt_registred_model.pcd",*output);
       
       doModelAlignment = false;
     */ 
    }
    
    
    std::cout << "Runnable ended" << std::endl;
}

void ObjectModeller::initial_alignment(PointCloudNT::Ptr src_cloud, PointCloudNT::Ptr target_cloud, 
				        Eigen::Matrix4f &final_transform, bool downsample, 
				        bool source_has_normals, bool target_has_normals ){

  using namespace pcl;
  
  float max_correspondence_distance_ = 1.0;
	  int nr_iterations_ =  100;
	  float min_sample_distance_ = leaf_size_;
          
	  pcl::console::print_highlight("Source cloud contains %d data points\n", src_cloud->size ());
          pcl::console::print_highlight("Target cloud contains %d data points\n", target_cloud->size ()); 
	  
	  pcl::console::print_highlight("leaf size = %f\n", leaf_size_); 
	  
	  
  pcl::VoxelGrid<PointNT> grid;
   if (downsample){
     grid.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
     grid.setInputCloud (target_cloud);
     grid.filter (*target_cloud);
	    
     grid.setInputCloud (src_cloud);
     grid.filter (*src_cloud);
     pcl::console::print_highlight("Filtered model cloud contains %d data points\n", src_cloud->size ());
     pcl::console::print_highlight("Filtered scene cloud contains %d data points\n", target_cloud->size ());
  }
   
  NormalEstimationOMP<PointNT, PointNT> ne;

  search::KdTree<PointNT>::Ptr search_tree (new search::KdTree<PointNT>);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (1.5f * leaf_size_);

  if(!target_has_normals){
    pcl::console::print_highlight("Estimating scene normals....");
    ne.setInputCloud (target_cloud);
    ne.compute (*target_cloud);
    pcl::console::print_highlight ("Finish...\n");
  }
   if(!source_has_normals){
    pcl::console::print_highlight("Estimating model normals....");
    ne.setInputCloud (src_cloud);
    ne.compute (*src_cloud);
    pcl::console::print_highlight ("Finish...\n");
  }
  
    pcl::console::print_highlight("FPFH - started\n");
    FeatureEstimationT pfh_est_src;
    pcl::search::KdTree<PointNT>::Ptr tree_pfh (new pcl::search::KdTree<PointNT>());
    pfh_est_src.setSearchMethod (tree_pfh);
    pfh_est_src.setRadiusSearch(3 * leaf_size_);
       // pfh_est_src.setSearchSurface (keypoints_src);
    pfh_est_src.setInputNormals (target_cloud);
    pfh_est_src.setInputCloud (target_cloud);

    pcl::PointCloud<FeatureT>::Ptr pfh_scene (new pcl::PointCloud<FeatureT>);
    pcl::console::print_highlight("FPFH - Compute scene features\n");
    pfh_est_src.compute (*pfh_scene);
    pcl::console::print_highlight("FPFH - finished\n");
    
    pfh_est_src.setInputNormals (src_cloud);
    pfh_est_src.setInputCloud (src_cloud);

    pcl::PointCloud<FeatureT>::Ptr pfh_model (new pcl::PointCloud<FeatureT>);
    pcl::console::print_highlight("FPFH - Compute model features\n");
    pfh_est_src.compute (*pfh_model);
    pcl::console::print_highlight("FPFH - finished\n");
    
    
    pcl::SampleConsensusInitialAlignment<PointNT, PointNT, FeatureT> sac_ia_;
    // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm

    sac_ia_.setMinSampleDistance (min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
    sac_ia_.setMaximumIterations (nr_iterations_);
    sac_ia_.setCorrespondenceRandomness(5);
    
    PointCloudNT::Ptr rotated_model (new PointCloudNT);
    
   
    copyPointCloud(*src_cloud, *rotated_model);
    std::vector<std::pair<float,Eigen::Matrix4f> > votes;
    Eigen::Matrix4f rotation;
    for(int i = 0; i<36;i++){
      
       if(i < 4){
	rotation = rotateZ(i* 1.57);
       }else if(i >= 4 && i < 8) {
	 rotation = rotateY((i-4)* 1.57);
       }else if(i >= 8 && i < 12) {
	 rotation = rotateX((i-8)* 1.57);
       }else if(i >= 12 && i < 16) {
	Eigen::Matrix4f temp =  rotateY(1.57);
	Eigen::Matrix4f temp2 = rotateZ((i-12)* 1.57);
	rotation = temp * temp2;
       }else if(i >= 16 && i < 20) {
	Eigen::Matrix4f temp =  rotateY(1.57);
	Eigen::Matrix4f temp2 = rotateX((i-16)* 1.57);
	rotation = temp * temp2;
       }else if(i >= 20 && i < 24) {
	Eigen::Matrix4f temp =  rotateX(1.57);
	Eigen::Matrix4f temp2 = rotateZ((i-20)* 1.57);
	rotation = temp * temp2;
       }else if(i >= 24 && i < 28) {
	Eigen::Matrix4f temp =  rotateX(1.57);
	Eigen::Matrix4f temp2 = rotateY((i-24)* 1.57);
	rotation = temp * temp2;
       }else if(i >= 28 && i < 32) {
	Eigen::Matrix4f temp =  rotateZ(1.57);
	Eigen::Matrix4f temp2 = rotateY((i-28)* 1.57);
	rotation = temp * temp2;
       }else if(i >= 32 && i < 36) {
	Eigen::Matrix4f temp =  rotateZ(1.57);
	Eigen::Matrix4f temp2 = rotateX((i-32)* 1.57);
	rotation = temp * temp2;
       }
       
       pcl::transformPointCloudWithNormals<PointNT>(*rotated_model, *rotated_model,rotation);
      
      sac_ia_.setInputSource(rotated_model);//setInputCloud (src);
      sac_ia_.setSourceFeatures (pfh_model);
      sac_ia_.setInputTarget (target_cloud);
      sac_ia_.setTargetFeatures (pfh_scene);
      
      pcl::console::print_highlight("Alignment %d started!\n", i);
      PointCloudNT registration_output;
      sac_ia_.align (registration_output);
    
      if (sac_ia_.hasConverged ()){
	// Print results
	printf ("\n");
	Eigen::Matrix4f transformation = sac_ia_.getFinalTransformation ();
	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
	pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
	pcl::console::print_info ("\n");
	pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
	pcl::console::print_info ("\n");
	pcl::console::print_info ("SAC-ia Fitness Score: %f\n", sac_ia_.getFitnessScore()); 
	std::pair<float,Eigen::Matrix4f> res(sac_ia_.getFitnessScore(), transformation);
	votes.push_back(res);
      }else{
	pcl::console::print_highlight("Alignment failed!\n");
      // return (1);
      }
    }
    
    //std::sort(votes.begin(),votes.end(),my_sort);
    std::sort(votes.begin(),votes.end(),boost::bind(&ObjectModeller::my_sort, this, _1, _2));
	    
    Eigen::Matrix4f best_transformation = votes.at(0).second;
    pcl::console::print_info ("Best Transformation:\n");
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", best_transformation (0,0), best_transformation (0,1), best_transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", best_transformation (1,0), best_transformation (1,1), best_transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", best_transformation (2,0), best_transformation (2,1), best_transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", best_transformation (0,3), best_transformation (1,3), best_transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Best Fitness Score: %f\n", votes.at(0).first); 
    
    final_transform = best_transformation;
   // pcl::transformPointCloudWithNormals<PointNT>(*src_cloud, *src_cloud,best_transformation);
	  
    
    
	  // Downsample for consistency and speed
	  // \note enable this for large datasets
/*	  PointCloudT::Ptr src (new PointCloudT);
	  PointCloudT::Ptr tgt (new PointCloudT);
	  
	  pcl::PointCloud<pcl::Normal>::Ptr src_normals (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::Normal>::Ptr tgt_normals (new pcl::PointCloud<pcl::Normal>);
	 
	  pcl::VoxelGrid<PointT> grid;
	  if (downsample){
	    grid.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
	    grid.setInputCloud (src_cloud);
	    grid.filter (*src);
	    
	    grid.setInputCloud (target_cloud);
	    grid.filter (*tgt);

	    pcl::console::print_highlight("Filtered source cloud contains %d data points\n", src->size ());
            pcl::console::print_highlight("Filtered target cloud contains %d data points\n", tgt->size ()); 
	  }else{
	    src = src_cloud;
	    tgt = target_cloud;
	  }
	  
	   pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	   pcl::search::KdTree<PointT>::Ptr search_tree (new pcl::search::KdTree<PointT>);
	   ne.setSearchMethod (search_tree);
	   ne.setRadiusSearch (1.5f * leaf_size_);

	  if(!target_has_normals){
	    pcl::console::print_highlight("Estimating target normals....");
	    ne.setInputCloud (tgt);
	    ne.compute (*tgt_normals);
	    pcl::console::print_highlight ("Finish...\n");
          }
   
	  if(!source_has_normals){
	    pcl::console::print_highlight("Estimating source normals....");
	    ne.setInputCloud (src);
	    ne.compute (*src_normals);
	    pcl::console::print_highlight ("Finish...\n");
	  }

	    pcl::console::print_highlight("FPFH - started\n");
	    FeatureEstimationT pfh_est_src;
	    pcl::search::KdTree<PointT>::Ptr tree_pfh (new pcl::search::KdTree<PointT>());
	    pfh_est_src.setSearchMethod (tree_pfh);
	    pfh_est_src.setRadiusSearch(3 * leaf_size_);
	    // pfh_est_src.setSearchSurface (keypoints_src);
	    pfh_est_src.setInputNormals (tgt_normals);
	    pfh_est_src.setInputCloud (tgt);

	    pcl::PointCloud<FeatureT>::Ptr pfh_tgt (new pcl::PointCloud<FeatureT>);
	    pcl::console::print_highlight("FPFH - Compute target features\n");
	    pfh_est_src.compute (*pfh_tgt);
	    pcl::console::print_highlight("FPFH - finished\n");
    
	    pfh_est_src.setInputNormals (src_normals);
	    pfh_est_src.setInputCloud (src);

	    pcl::PointCloud<FeatureT>::Ptr pfh_src (new pcl::PointCloud<FeatureT>);
	    pcl::console::print_highlight("FPFH - Compute source features\n");
	    pfh_est_src.compute (*pfh_src);
	    pcl::console::print_highlight("FPFH - finished\n");
	    
	    PointCloudNT::Ptr src_with_normals (new PointCloudNT);
	    PointCloudNT::Ptr tgt_with_normals (new PointCloudNT);
	    
	    pcl::PointCloud<pcl::PointXYZ>::Ptr src_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::copyPointCloud(*src, *src_xyz);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::copyPointCloud(*tgt, *tgt_xyz);
	    
	    pcl::concatenateFields(*src_xyz, *src_normals, *src_with_normals);
	    pcl::concatenateFields(*tgt_xyz, *tgt_normals, *tgt_with_normals);
	    
	    pcl::SampleConsensusInitialAlignment<PointNT, PointNT, FeatureT> sac_ia_;
	    // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm

	    sac_ia_.setMinSampleDistance (min_sample_distance_);
	    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
	    sac_ia_.setMaximumIterations (nr_iterations_);
	    sac_ia_.setCorrespondenceRandomness(5);
    
	    PointCloudNT::Ptr rotated_model (new PointCloudNT);
	   
	    pcl::copyPointCloud(*src_with_normals, *rotated_model);
	    std::vector<std::pair<float,Eigen::Matrix4f> > votes;
	    Eigen::Matrix4f rotation;
	    for(int i = 0; i<12;i++){
	      if(i < 4){
		rotation = rotateZ(i* 1.57);
	      }else if(i >= 4 && i < 8) {
		rotation = rotateY(i* 1.57);
	      }else if(i >= 8 && i < 12) {
		rotation = rotateX(i* 1.57);
	      }
	      pcl::transformPointCloudWithNormals<PointNT>(*rotated_model, *rotated_model,rotation);
	      
	      sac_ia_.setInputSource(rotated_model);//setInputCloud (src);
	      sac_ia_.setSourceFeatures (pfh_src);
	      sac_ia_.setInputTarget (tgt_with_normals);
	      sac_ia_.setTargetFeatures (pfh_tgt);
	      
	      pcl::console::print_highlight("Alignment %d started!\n", i);
	      PointCloudNT registration_output;
	      sac_ia_.align (registration_output);
	    
	      if (sac_ia_.hasConverged ()){
		// Print results
		printf ("\n");
		Eigen::Matrix4f transformation = sac_ia_.getFinalTransformation ();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("SAC-ia Fitness Score: %f\n", sac_ia_.getFitnessScore()); 
		std::pair<float,Eigen::Matrix4f> res(sac_ia_.getFitnessScore(), transformation);
		votes.push_back(res);
	      }else{
		pcl::console::print_highlight("Alignment failed!\n");
	      // return (1);
	      }
	    }
	    
	    std::sort(votes.begin(),votes.end(),boost::bind(&ObjectModeller::my_sort, this, _1, _2));
	    
	    Eigen::Matrix4f best_transformation = votes.at(0).second;
	    pcl::console::print_info ("Best Transformation:\n");
	    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", best_transformation (0,0), best_transformation (0,1), best_transformation (0,2));
	    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", best_transformation (1,0), best_transformation (1,1), best_transformation (1,2));
	    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", best_transformation (2,0), best_transformation (2,1), best_transformation (2,2));
	    pcl::console::print_info ("\n");
	    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", best_transformation (0,3), best_transformation (1,3), best_transformation (2,3));
	    pcl::console::print_info ("\n");
	    pcl::console::print_info ("Best Fitness Score: %f\n", votes.at(0).first); 
	    
	    final_transform = best_transformation;
  //  pcl::transformPointCloudWithNormals<PointNT>(*src, *src,best_transformation);
	*/ 
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
bool ObjectModeller::pairAlign (PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud, PointCloudT::Ptr &output, Eigen::Matrix4f &final_transform, bool downsample = true){

  bool result = true;
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloudT::Ptr src (new PointCloudT);
  PointCloudT::Ptr tgt (new PointCloudT);
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
  PointCloudNT::Ptr points_with_normals_src (new PointCloudNT);
  PointCloudNT::Ptr points_with_normals_tgt (new PointCloudNT);

  
  pcl::NormalEstimationOMP<PointT, PointNT> norm_est;
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
  pcl::IterativeClosestPointNonLinear<PointNT, PointNT> reg;
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
  pcl::PointCloud<PointNT>::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (10);

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
    float sum = (reg.getLastIncrementalTransformation () - prev).sum (); 
    std::cout << "sum: " << sum << std::endl;
    if(sum == 0){
      break; 
    }
    
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();
    conv = reg.getFitnessScore();
    std::cout << "Registration has converged: " << reg.hasConverged() << " with fitness score: " << reg.getFitnessScore() << std::endl;

  }

  if(conv > 0.0001)
  {
	  PCL_ERROR("Registration Error!!");
	//TODO: comment this in!!  result = false;
  }
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud(*target_cloud, *output, targetToSource);
  
  //add the source to the transformed target
  *output += *src_cloud;
  
  final_transform = targetToSource;
  
  return result;
 }

void ObjectModeller::EstimateNormals(PointCloudT::Ptr &src_cloud,PointCloudNT::Ptr &target_cloud){
	// Normal estimation*
	pcl::NormalEstimationOMP<PointT, PointNT> n;
	PointCloudNT::Ptr cloud_with_normals (new PointCloudNT);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		 
	tree->setInputCloud (src_cloud);
	n.setInputCloud (src_cloud);
	n.setSearchMethod (tree);
	n.setRadiusSearch(1.5 * leaf_size_);
	//n.setKSearch (20);
	n.compute (*cloud_with_normals);
	
	PCL_INFO("Finish normal estimation for model. Size: %d\n", (int)cloud_with_normals->size());
	
	pcl::copyPointCloud(*cloud_with_normals, *target_cloud);
}

void ObjectModeller::statisticalOutlierRemoval(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud, int mean_num_pts){
	PointCloudT::Ptr cloud_filtered (new PointCloudT);
	if(src_cloud->size() > 0)
	{
	    try{
	      // Create the filtering object
	      pcl::StatisticalOutlierRemoval<PointT> sor;
	      sor.setInputCloud (src_cloud);
	      sor.setMeanK (mean_num_pts);
	      sor.setStddevMulThresh (1.0);
	      sor.filter (*cloud_filtered);
	      
	      if(src_cloud->isOrganized()){
		sor.setUserFilterValue(0.0);
		sor.setKeepOrganized(true);
	      }
	      
	      pcl::copyPointCloud(*cloud_filtered, *target_cloud);
	    }catch(...)
	    {
	      PCL_ERROR("Somthing went wrong in object_modeller::statisticalOutlierRemoval()");
	    }
	}
}

void ObjectModeller::radiusOutlierRemoval(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud, double radius, int min_neighbpr_pts){
	PointCloudT::Ptr cloud_filtered (new PointCloudT);

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


/*
void ObjectModeller::removePlane(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud, double dist_threads){
   
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

   // Create the segmentation object
   pcl::SACSegmentation<PointNT> seg;
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


   PointCloudT::Ptr cloud_f (new PointCloudT);
   PointCloudT::Ptr cloud_p (new PointCloudT);
  // Create the filtering object
  pcl::ExtractIndices<PointNT> extract;
  // Extract the inliers
  extract.setInputCloud (src_cloud);
  extract.setIndices(inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);

  pcl::copyPointCloud(*cloud_f, *target_cloud);

}
*/
/*
void ObjectModeller::CropBox(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud,float rx, float ry, float minz, float maxz,
			      Eigen::Vector3f boxTranslatation, Eigen::Vector3f boxRotation){

  PointCloudT::Ptr filtered (new PointCloudT);
  
  pcl::CropBox<PointNT> cropFilter; 
  cropFilter.setInputCloud (src_cloud); 
  cropFilter.setMin(Eigen::Vector4f(-rx, -ry, minz, 1.0f)); 
  cropFilter.setMax(Eigen::Vector4f(rx, ry, maxz, 1.0f)); 
  cropFilter.setTranslation(boxTranslatation); 
  cropFilter.setRotation(boxRotation); 
  cropFilter.setKeepOrganized(true);

  cropFilter.filter (*filtered); 
  
  pcl::copyPointCloud(*filtered, *target_cloud);
  
}
*/
/*
bool ObjectModeller::extractClusters(PointCloudT::Ptr &src_cloud, std::vector<CloudT::Ptr, Eigen::aligned_allocator_indirection<CloudT::Ptr> > &clusters){
    
    pcl::search::KdTree<PointNT>::Ptr tree (new pcl::search::KdTree<PointNT>);
   
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (50000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(src_cloud);
    ec.extract (cluster_indices); 
    
     std::cout<<"Found "  << cluster_indices.size() << " clusters" << std::endl;
     
    if ( cluster_indices.size() < 1) return false;
    
     PointCloudT::Ptr model (new PointCloudT);
     
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
       
	PointCloudT::Ptr cloud_cluster (new PointCloudT);
   
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back (src_cloud->points[*pit]); //

	    if(cloud_cluster->points.size () > 500)
	     {
	     	 cloud_cluster->width = cloud_cluster->points.size ();
	     	 cloud_cluster->height = 1;
	     	 cloud_cluster->is_dense = true;
		 
		 Eigen::Vector4f centroid;
		 pcl::compute3DCentroid(*cloud_cluster,centroid);
		 
		 pcl::PCA<PointT> _pca; 
		 PointT projected; 
		 PointT reconstructed;
		 CloudT cloudi = *cloud_cluster;
		 CloudT finalCloud;
		 
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


	     	 std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		 clusters.push_back(finalCloud.makeShared());
	     }
	     
	     
	      pcl::copyPointCloud(*cloud_cluster, *model);
      }
    
    return true;
  
}
*/


Eigen::Matrix4f ObjectModeller::rotateX(float radians) {
    float Sin = sinf(radians);
    float Cos = cosf(radians);

    Eigen::Matrix4f rotationMatrix( Eigen::Matrix4f::Identity() );

    rotationMatrix(1, 1) =  Cos;
    rotationMatrix(1, 2) =  Sin;
    rotationMatrix(2, 1) = -Sin;
    rotationMatrix(2, 2) =  Cos;

    return rotationMatrix;
}

Eigen::Matrix4f ObjectModeller::rotateY(float radians) {
    float Sin = sinf(radians);
    float Cos = cosf(radians);

    Eigen::Matrix4f rotationMatrix( Eigen::Matrix4f::Identity() );

    rotationMatrix(0, 0) =  Cos;
    rotationMatrix(0, 2) =  Sin;
    rotationMatrix(2, 0) = -Sin;
    rotationMatrix(2, 2) =  Cos;

    return rotationMatrix;
}

Eigen::Matrix4f ObjectModeller::rotateZ(float radians) {
    float Sin = sinf(radians);
    float Cos = cosf(radians);

    Eigen::Matrix4f rotationMatrix( Eigen::Matrix4f::Identity() );

    rotationMatrix(0, 0) =  Cos;
    rotationMatrix(0, 1) =  Sin;
    rotationMatrix(1, 0) = -Sin;
    rotationMatrix(1, 1) =  Cos;

    return rotationMatrix;
}

void ObjectModeller::getRawModel(PointCloudT::Ptr &model){
  QMutexLocker lock(&_mutexCloud);
  model = _raw_model;
}

void ObjectModeller::getPostProcessedModel(PointCloudT::Ptr &model){
  QMutexLocker lock(&_mutexCloud);
  model = _post_processed_cloud;
}

void ObjectModeller::getMeshModel(pcl::PolygonMesh &mesh){ 
  QMutexLocker lock(&_mutexMesh);
  mesh = _mesh;
}

}
}  // namespace object_modeller_gui