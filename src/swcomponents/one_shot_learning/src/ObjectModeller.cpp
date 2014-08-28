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
#include <pcl/common/pca.h>
#include <pcl/exceptions.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

namespace dti{
namespace one_shot_learning {
  
  
ObjectModeller::ObjectModeller(SharedData *data) : _sharedData(data)
{
  start();
  isRunning = true;
  doModel = false;
  doSolidModel = false;
  doModelAlignment = false;
  
  reconstruct = new ReconstructPointCloud();
  
}

ObjectModeller::~ObjectModeller() {
  delete reconstruct;
}
void ObjectModeller::shutdown()
{
  isRunning = false;
  this->quit();
  this->exit();
}

void ObjectModeller::createmodel(pcl::PointCloud<pcl::PointXYZRGBA> in)
{
  _cloud = in.makeShared();
  //if(_cloud->isOrganized()) std::cout << "_cloud is organized" << std::endl;
  //else std::cout << "_cloud is not organized" << std::endl;
   doModel = true;
}

void ObjectModeller::createSolidmodel(pcl::PointCloud<pcl::PointXYZRGBA> in)
{
  _cloud = in.makeShared();
   
  doSolidModel = true;
}

void ObjectModeller::alignGTModelAndSceneModel(pcl::PointCloud<pcl::PointXYZRGBA> src, pcl::PointCloud<pcl::PointXYZRGBA> tar)
{
  _cloud = src.makeShared();
  _tar = tar.makeShared();
  doModelAlignment = true;
}

void ObjectModeller::run()
{
 /* Crop box for world frame = kinect right 
  Eigen::Vector3f boxTranslatation; boxTranslatation[0]=0.25;  boxTranslatation[1]=-0.3;   boxTranslatation[2]=0.6;  
  Eigen::Vector3f boxRotation;   boxRotation[0]=0;  // rotation around x-axis  
				  boxRotation[1]=-0.785;  // rotation around y-axis 
				  boxRotation[2]=0.785;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 
  */
  Eigen::Vector3f boxTranslatation; boxTranslatation[0]=0;  boxTranslatation[1]=-0.5;   boxTranslatation[2]=0;  
  Eigen::Vector3f boxRotation;   boxRotation[0]=0;  // rotation around x-axis  
				  boxRotation[1]=0;  // rotation around y-axis 
				  boxRotation[2]=-0.785;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.
 
 
  while(isRunning)
  {
   
    if(doModel)
    {
      //CropBox(_cloud,_cloud,0.7,0.7, 0.0, 0.6,boxTranslatation,boxRotation);//1.0,1.0,0.9,1.4
      CropBox(_cloud,_cloud,0.4,0.4, 0.0, 0.6,boxTranslatation,boxRotation);//1.0,1.0,0.9,1.4
      //pcl::io::savePCDFile("crop_box.pcd",*_cloud);
   
      removePlane(_cloud,_cloud,0.012);
       
    //  radiusOutlierRemoval(_cloud, _cloud, 0.06);

   //   statisticalOutlierRemoval(_cloud,_cloud,0.5);
      
      std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > clusters;
      if(extractClusters(_cloud, clusters))
      {
	  pcl::PointCloud<pcl::PointNormal>::Ptr _normals(new pcl::PointCloud<pcl::PointNormal>);
      //  pcl::PointCloud<pcl::PointNormal>::Ptr _new(new pcl::PointCloud<pcl::PointNormal>);
      
	 // pcl::io::savePCDFile("before_mls.pcd",*clusters[0]);
      
	 // reconstruct->MLSApproximation(clusters[0], clusters[0]);
      
	//  pcl::io::savePCDFile("model_created.pcd",*clusters[0]);
	  
	  //Signal to GUI to turn the model
	 // emit turnModel();
       
//	  EstimateNormals(clusters[0], _normals);	
	  // Concatenate the XYZRGBA and normal fields*
//	  pcl::copyPointCloud (*clusters[0], *_normals);
      
     
	//  pcl::PolygonMesh mesh;
//	  reconstruct->poisson(_normals,mesh,8,6,32,4.0f); //8,12,8,4.0f)
	  // mesh = reconstruct->GreedyProjectionTriangulation(_new, 0.05);
	  //pcl::PolygonMesh mesh = reconstruct->MarchingCubes(_normals);
     
	  //reconstruct->saveToObj("mesh.obj", mesh);
	  //reconstruct->saveToVTK("mesh.vtk", mesh);
     
	  views.push_back(clusters[0]);
	  if(views.size() > 1)
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
	    emit modelCreated(*clusters[0]);
	  }
      }
      doModel = false;
    }
    
    
    if(doSolidModel)
    {
      
	//Create solid model from _cloud
	  pcl::PointCloud<pcl::PointNormal>::Ptr _normals(new pcl::PointCloud<pcl::PointNormal>);
      
	  //pcl::io::savePCDFile("before_mls.pcd",*_cloud);
	  //reconstruct->MLSApproximation(_cloud, _cloud);
	  //pcl::io::savePCDFile("model_created.pcd",*_cloud);
	  
	  EstimateNormals(_cloud, _normals);	
	  // Concatenate the XYZRGBA and normal fields*
	  pcl::copyPointCloud (*_cloud, *_normals);
      
	  pcl::PolygonMesh mesh;
	  reconstruct->poisson(_normals,mesh,8,6,32,4.0f); //8,12,8,4.0f)
	  // mesh = reconstruct->GreedyProjectionTriangulation(_new, 0.05);
	  //pcl::PolygonMesh mesh = reconstruct->MarchingCubes(_normals);
     
	  //reconstruct->saveToObj("mesh.obj", mesh);
	  //reconstruct->saveToVTK("mesh.vtk", mesh);
	  
	  emit solidModelCreated(*_cloud,mesh);
     
	  doSolidModel = false;
      
    }
    
    if(doModelAlignment)
    {	//Aligning GT_model and scene model to compute variance in coordinate system
       Eigen::Matrix4f initial_transformation, final_transformation;
       initial_alignment(_cloud, _tar,initial_transformation,true);
       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_source(new   pcl::PointCloud<pcl::PointXYZRGBA>);
       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output(new   pcl::PointCloud<pcl::PointXYZRGBA>);
       pcl::transformPointCloud (*_cloud, *new_source, initial_transformation);
       pcl::io::savePCDFile("gt_align0.pcd",*new_source);
       pcl::io::savePCDFile("gt_align1.pcd",*_tar);
	    
       std::cout << "running ICP!!" << std::cout;
	  
       pairAlign(new_source,_tar,output,final_transformation,true);
       pcl::io::savePCDFile("gt_registred_model.pcd",*output);
       
       doModelAlignment = false;
      
    }
    
    //Sleep 500msek
    QThread::msleep(500);
  }
  std::cout << "Object modeller thread is shutting down!" << std::endl;
}


void ObjectModeller::initial_alignment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud, Eigen::Matrix4f &final_transform, bool downsample = true)
{
	  float max_correspondence_distance_ = 5.0;
	  int nr_iterations_ =  500;
	  float min_sample_distance_ = 0.005;


	  // Downsample for consistency and speed
	    // \note enable this for large datasets
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGBA>);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGBA>);
	    pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
	    if (downsample)
	    {
	      grid.setLeafSize (0.005, 0.005, 0.005);
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

	    
	    int normest_Ksearch = 20;
	    pcl::console::print_info("Computing Surface Normales!\n");

	    pcl::PointCloud<pcl::Normal>::Ptr norm_src (new pcl::PointCloud<pcl::Normal>);
	    pcl::PointCloud<pcl::Normal>::Ptr norm_tgt (new pcl::PointCloud<pcl::Normal>);

	    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> norm_est;
	    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	    norm_est.setSearchMethod (tree);
	    norm_est.setKSearch (normest_Ksearch);

	    norm_est.setInputCloud (src);
	    norm_est.compute (*norm_src);
	    std::cout << "Computed normals: " << norm_src->size () << std::endl;
	    		//pcl::copyPointCloud (*src_sampled, *points_with_normals_src);
   	    norm_est.setInputCloud (tgt);
   	    norm_est.compute (*norm_tgt);
	    std::cout << "Computed normals: " << norm_tgt->size () << std::endl;

   	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZRGBA>);
   	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints_tgt (new pcl::PointCloud<pcl::PointXYZRGBA>);

	    PCL_INFO ("FPFH - started\n");
	    pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> pfh_est_src;
	    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_pfh_src (new pcl::search::KdTree<pcl::PointXYZRGBA>());
	    pfh_est_src.setSearchMethod (tree_pfh_src);
	    pfh_est_src.setKSearch(20);
	    //pfh_est_src.setSearchSurface (keypoints_src);
	    pfh_est_src.setInputNormals (norm_src);
	    // pfh_est_src.setInputCloud (src_sampled);
	    pfh_est_src.setInputCloud (src);

	    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_src (new pcl::PointCloud<pcl::FPFHSignature33>);
	    PCL_INFO ("	FPFH - Compute Source\n");
	    pfh_est_src.compute (*pfh_src);
	    PCL_INFO ("	FPFH - finished\n");

	    pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> pfh_est_tgt;
	    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_pfh_tgt (new pcl::search::KdTree<pcl::PointXYZRGBA>());
	    pfh_est_tgt.setSearchMethod (tree_pfh_tgt);
	    pfh_est_tgt.setKSearch(20);
	  //  pfh_est_tgt.setSearchSurface (tgt);
	    pfh_est_tgt.setInputNormals (norm_tgt);
	    // pfh_est_tgt.setInputCloud (tar_sampled);
	    pfh_est_tgt.setInputCloud (tgt);

	    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
	    PCL_INFO ("	FPFH - Compute Target\n");
	    pfh_est_tgt.compute (*pfh_tgt);
	    PCL_INFO ("	FPFH - finished\n");


	  pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::FPFHSignature33> sac_ia_;

	  // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm

	  sac_ia_.setMinSampleDistance (min_sample_distance_);
	  sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
	  sac_ia_.setMaximumIterations (nr_iterations_);
	  sac_ia_.setInputSource(src);//setInputCloud (src);
	  sac_ia_.setSourceFeatures (pfh_src);
	  sac_ia_.setInputTarget (tgt);
	  sac_ia_.setTargetFeatures (pfh_tgt);

	  pcl::PointCloud<pcl::PointXYZRGBA> registration_output;
	  sac_ia_.align (registration_output);


	  float fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
	  // Print the alignment fitness score (values less than 0.00002 are good)
	  printf ("Best fitness score: %f\n", fitness_score);
	  Eigen::Matrix4f f_transformation = sac_ia_.getFinalTransformation ();

	  std::cout << f_transformation << std::endl;

	  final_transform = f_transformation;
//	  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
	  //cloudNext is my target cloud
//	  pcl::transformPointCloud(*tar,cloud_transformed,final_transformation);
	  
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
bool ObjectModeller::pairAlign (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &output, Eigen::Matrix4f &final_transform, bool downsample = true)
{

  bool result = true;
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
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

  
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
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


void ObjectModeller::EstimateNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud,pcl::PointCloud<pcl::PointNormal>::Ptr &target_cloud)
{
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> n;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		 
	tree->setInputCloud (src_cloud);
	n.setInputCloud (src_cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*cloud_with_normals);
	
	PCL_INFO("Finish normal estimation for model. Size: %d\n", (int)cloud_with_normals->size());
	
	pcl::copyPointCloud(*cloud_with_normals, *target_cloud);
}

void ObjectModeller::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud, double mean)
{
	 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	if(src_cloud->size() > 0)
	{
	    try{
	      // Create the filtering object
	      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	      sor.setInputCloud (src_cloud);
	      sor.setMeanK (mean);
	      sor.setStddevMulThresh (1.0);
	      sor.filter (*cloud_filtered);
	      sor.setKeepOrganized(true);
	      
	      pcl::copyPointCloud(*cloud_filtered, *target_cloud);
	    }catch(...)
	    {
	      PCL_ERROR("Somthing went wrong in object_modeller::statisticalOutlierRemoval()");
	    }
	}
}

void ObjectModeller::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud, double radius)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

	if(src_cloud->size() > 0)
	{
	    try{
	      // Create the filtering object
	      pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> ror;
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

void ObjectModeller::removePlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud, double dist_threads)
{
   //*********************************************************************//
   //	Plane fitting
   /**********************************************************************/
    
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
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


   //*********************************************************************//
   //	Extract Indices
   /**********************************************************************/

   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
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

void ObjectModeller::CropBox(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud,
             float rx, float ry, float minz, float maxz, Eigen::Vector3f boxTranslatation, Eigen::Vector3f boxRotation)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  pcl::CropBox<pcl::PointXYZRGBA> cropFilter; 
  cropFilter.setInputCloud (src_cloud); 
  cropFilter.setMin(Eigen::Vector4f(-rx, -ry, minz, 1.0f)); 
  cropFilter.setMax(Eigen::Vector4f(rx, ry, maxz, 1.0f)); 
  cropFilter.setTranslation(boxTranslatation); 
  cropFilter.setRotation(boxRotation); 
  cropFilter.setKeepOrganized(true);

  cropFilter.filter (*filtered); 
  
  pcl::copyPointCloud(*filtered, *target_cloud);
  
}

bool ObjectModeller::extractClusters(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > &clusters)
{
    
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
   
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (50000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(src_cloud);
    ec.extract (cluster_indices); 
    
     std::cout<<"Found "  << cluster_indices.size() << " clusters" << std::endl;
     
    if ( cluster_indices.size() < 1) return false;
    
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
     
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
       
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
   
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back (src_cloud->points[*pit]); //*

	    if(cloud_cluster->points.size () > 500)
	     {
	     	 cloud_cluster->width = cloud_cluster->points.size ();
	     	 cloud_cluster->height = 1;
	     	 cloud_cluster->is_dense = true;
		 
		 Eigen::Vector4f centroid;
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


	     	 std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		 clusters.push_back(finalCloud.makeShared());
	     }
	     
	     
	      pcl::copyPointCloud(*cloud_cluster, *model);
      }
    
    return true;
  
}
}
}  // namespace object_modeller_gui