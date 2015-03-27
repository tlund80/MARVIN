/*
 * ReconstructPointCloud.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#include <one_shot_learning/ReconstructPointCloud.hpp>

#include <pcl/io/pcd_io.h>
//#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/bilateral_upsampling.h>


#include <pcl/filters/median_filter.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/random_sample.h>


#include <pcl/surface/poisson.h>
#include <pcl/console/time.h>

namespace dti{
namespace one_shot_learning {

ReconstructPointCloud::ReconstructPointCloud() {
	// TODO Auto-generated constructor stub

}

ReconstructPointCloud::~ReconstructPointCloud() {
	// TODO Auto-generated destructor stub
}

//////////////////////////////////////////////////////////////////////////
///  This algorithm filters only the depth (z-component) of organized ////
///  and untransformed (i.e., in camera coordinates) point clouds.	////
///  An error will be outputted if an unorganized cloud is given 	////
///  to the class instance.						////
//////////////////////////////////////////////////////////////////////////
bool ReconstructPointCloud::OrganizedMedianFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, int window_size, float movement)
{
  using namespace pcl::console;
   
  if(!src_cloud->isOrganized()){
    PCL_ERROR("You cannot apply a Median filter to non-organized PointClouds!");
    pcl::copyPointCloud(*src_cloud, *tar_cloud);
    return false;
  }
  
  pcl::MedianFilter<PointT> mf;
  //Set the window size of the filter.
  mf.setWindowSize(window_size);
  //Set the largest value one dexel is allowed to move.
  mf.setMaxAllowedMovement(movement);
  mf.setInputCloud(src_cloud);
  
  TicToc tt;
  tt.tic ();
  
  print_highlight("Applying Median filter to cloud....");
  mf.applyFilter(*tar_cloud);
  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
  
  return true;
}

/////////////////////////////////////////////////////////////////////////
///  Filters the pointCloud based on the camera frustum aka. view cone////
////////////////////////////////////////////////////////////////////////
void ReconstructPointCloud::FrustumCullingFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, 
						  float vertical_FOV, float horizontal_FOV, 
						  float near_plane_dist, float far_plane_dist, Eigen::Matrix4f camera_pose){
 
  using namespace pcl::console;
  
  pcl::PointCloud <PointT>::Ptr source; 

  pcl::FrustumCulling<PointT> fc;
  fc.setInputCloud (src_cloud);
  //Set the vertical field of view for the camera in degrees.
  fc.setVerticalFOV (vertical_FOV); //45
  //Set the horizontal field of view for the camera in degrees.
  fc.setHorizontalFOV (horizontal_FOV); //60
  //Set the near plane distance.
  fc.setNearPlaneDistance (near_plane_dist); //5.0
  //Set the far plane distance.
  fc.setFarPlaneDistance (far_plane_dist); //15
  //Set the pose of the camera w.r.t the origin.
  fc.setCameraPose (camera_pose);
  
  TicToc tt;
  tt.tic ();
  
  print_highlight("Applying Frustum culling filter to cloud....");
  fc.filter (*tar_cloud);
  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
  
}
/////////////////////////////////////////////////////////////////////////
///  Removes the ghost points appearing on edge discontinuties////
////////////////////////////////////////////////////////////////////////
void ReconstructPointCloud::ShadowFilter(PointCloudNT::Ptr &src_cloud, PointCloudNT::Ptr &tar_cloud, float threshold, pcl::PointIndices::Ptr removedIndices){
  
  using namespace pcl::console;
    
  pcl::ShadowPoints<PointNT, PointNT> spf(true);
  //Set the normals computed on the input point cloud.
  spf.setNormals(src_cloud);
  if(src_cloud->isOrganized()){
    spf.setKeepOrganized(true);
    spf.setUserFilterValue(0.0);
  }
  //Set the threshold for shadow points rejection.
  spf.setThreshold(threshold);
  spf.setNegative(false);
  spf.setInputCloud(src_cloud);
    
  TicToc tt;
  tt.tic ();
  print_highlight("Applying Shadow filter to cloud....");
  spf.filter(*tar_cloud);
  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
  
  if(removedIndices)
    spf.getRemovedIndices(*removedIndices);
}


void ReconstructPointCloud::PlaneClippingFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, const Eigen::Vector4f &plane_params){
  
  pcl::PlaneClipper3D<PointT> pc(plane_params);
  pc.setPlaneParameters(plane_params);
  std::vector<int> clipped_points;
  
  pc.clipPointCloud3D(*tar_cloud,clipped_points);
  
}
/////////////////////////////////////////////////////
///  Data smoothing and improved normal estimation////
/////////////////////////////////////////////////////
void ReconstructPointCloud::MLSApproximation(PointCloudT::Ptr &cloud, PointCloudNT::Ptr &target, double search_radius)
{
	 using namespace pcl::console;
	 
	 // Create a KD-Tree
	  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

	  // Output has the PointNormal type in order to store the normals calculated by MLS
	  pcl::PointCloud<PointNT>::Ptr mls_points (new pcl::PointCloud<PointNT>);

	  // Init object (second point type is for the normals, even if unused)
	  pcl::MovingLeastSquares<PointT, PointNT> mls;

	  mls.setComputeNormals (true);

	  // Set parameters
	  mls.setInputCloud (cloud);
	  mls.setPolynomialFit (true);
	  mls.setSearchMethod (tree);
	  mls.setSearchRadius (search_radius); //0.025
	  mls.setComputeNormals(false);
	   mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointNT>::VOXEL_GRID_DILATION);
	  //mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointNT>::UpsamplingMethod::RANDOM_UNIFORM_DENSITY);
	  // Reconstruct
	  TicToc tt;
	  tt.tic ();
	  print_highlight("Computing smoothed point cloud using MLS algorithm....");
	  mls.process (*mls_points);
	  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

	  pcl::copyPointCloud(*mls_points, *target);

}
////////////////////////////////////////////////////////////////
///  A bilateral filter implementation for point cloud data.////
///////////////////////////////////////////////////////////////
void ReconstructPointCloud::BilateralFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, double sigma_s, double sigma_r){
  
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  
 /* pcl::BilateralFilter<PointT> bf;
  bf.setHalfSize(sigma_s);
  bf.setSearchMethod(tree);
  bf.setStdDev(sigma_r);
  bf.setInputCloud(src_cloud);
 */
//  bf.filter(*tar_cloud);

}

bool ReconstructPointCloud::BilateralUpsampling(PointCloudT::Ptr cloud, PointCloudT::Ptr output,
         int window_size, float sigma_color, float sigma_depth)
{
	using namespace pcl::console;
	print_info("Bilateral Upsampling using parameters: window_size %d, sigma_color %2f, sigma_depth %2f\n", window_size, sigma_color, sigma_depth);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_upsampled (new pcl::PointCloud<pcl::PointXYZRGBA> ());

	// BilateralUpsampling requires organized point clouds
	if(cloud->isOrganized())
	{
		pcl::BilateralUpsampling<pcl::PointXYZRGBA, pcl::PointXYZRGBA> bu;
		bu.setInputCloud (cloud);
		bu.setWindowSize (window_size);
		bu.setSigmaColor (sigma_color);
		bu.setSigmaDepth (sigma_depth);
		
		// TODO need to fix this somehow
		bu.setProjectionMatrix (bu.KinectVGAProjectionMatrix);

		TicToc tt;
		tt.tic ();
		bu.process (*cloud_upsampled);
		print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_upsampled->width * cloud_upsampled->height); print_info (" points]\n");

		pcl::copyPointCloud(*cloud_upsampled, *output);
		return true;
	}else
	{
		pcl::copyPointCloud(*cloud, *output);
		print_error("Input cloud is not organized in ReconstructPointCloud::BilateralUpsampling()\n");
		return false;
	}

}

void ReconstructPointCloud::RandomSampling(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, const unsigned int samples){
  
  pcl::RandomSample<PointT> rs;
  
  if(src_cloud->isOrganized()){
    rs.setKeepOrganized(true);
    rs.setUserFilterValue(0.0);
  }
  
  rs.setSample(samples);
  rs.setSeed(0);
  
  rs.filter(*tar_cloud);
  
  
}

//////////////////////////////////////////////////////////////////////////
///  NormalSpaceSampling samples the input point cloud in the space of ////
///  normal directions computed at every point.			////
/////////////////////////////////////////////////////////////////////////
/*void ReconstructPointCloud::NormalSpaceSampling(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, unsigned int bin_size){
  
  pcl::NormalSpaceSampling<PointT,  PointT> nss;
  
  nss.setInputCloud (src_cloud);
  nss.setNormals (src_cloud);
  
   if(src_cloud->isOrganized()){
    nss.setKeepOrganized(true);
    nss.setUserFilterValue(0.0);
  }
  //Set the number of bins in x, y and z direction.
  nss.setBins (bin_size, bin_size, bin_size); //4
  //Set seed of random function.
  nss.setSeed (0);
  //Set number of indices to be sampled.
  nss.setSample (static_cast<unsigned int> (src_cloud->size ()) / 4);

 // pcl::IndicesPtr walls_indices (new std::vector<int> ());
  nss.filter (*tar_cloud);
  
}
*/

void ReconstructPointCloud::LocalMaximumDownSampling(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, const float radius){
  
  pcl::LocalMaximum<PointT> lm;
  lm.setRadius(radius);
  
  if(src_cloud->isOrganized()){
    lm.setKeepOrganized(true);
    lm.setUserFilterValue(0.0);
  }
  
  lm.setInputCloud(src_cloud);
  lm.filter(*tar_cloud);
}

/////////////////////////////////////////////////////////////////////////
///  Water tight reconstruction method which modifies vertex positions////
////////////////////////////////////////////////////////////////////////
void ReconstructPointCloud::poisson (const PointCloudNT::Ptr &cloud, pcl::PolygonMesh &output,
         int depth, int solver_divide, int iso_divide, float point_weight)
{

  using namespace pcl::console;
  print_info ("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth (depth);
  poisson.setSolverDivide (solver_divide);
  poisson.setIsoDivide (iso_divide);
  poisson.setInputCloud(cloud);

  //Enabling this flag tells the reconstructor to output a polygon mesh
  //(rather than triangulating the results of Marching Cubes).
  poisson.setOutputPolygons(true);

  TicToc tt;
  tt.tic ();
  print_highlight ("Reconstructing point cloud with the poisson algorithm ...");
  poisson.reconstruct (output);

  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

////////////////////////////////////////////////////////////////////
///Convert point cloud to mesh without modifying vertex positions//
///////////////////////////////////////////////////////////////////
void ReconstructPointCloud::GreedyProjectionTriangulation(PointCloudNT::Ptr cloud_with_normals, pcl::PolygonMesh &output, double SearchRadius)
{
  using namespace pcl::console;
  
  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  
  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (SearchRadius); //0.025
  
  // Set typical values for the parameters
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100); //100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees M_PI/4
  gp3.setMinimumAngle(M_PI/18); // 10 degrees M_PI/18
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);
  
  // Define inputs to the triangulation structure
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  
  // Reconstruct
  TicToc tt;
  tt.tic ();
  print_highlight ("Reconstructing point cloud with the Greedy Projection Triangulation algorithm ...");
  gp3.reconstruct (output);
  
  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

  // Additional vertex information
  //std::vector<int> parts = gp3.getPartIDs();
  //std::vector<int> states = gp3.getPointStates();
  //print_info ("Result, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
  //std::cout << "GreedyProjectionTriangulation parts: " << (int)parts.size()<< std::endl;

	
}

////////////////////////////////////////////////////////////////////
///Convert point cloud to mesh by modifying vertex positions//
///////////////////////////////////////////////////////////////////
bool ReconstructPointCloud::MarchingCubes(PointCloudNT::Ptr cloud_with_normals, pcl::PolygonMesh &output, 
					   int leaf_size, double iso_level, float extend_grid_percentage){
  
  using namespace pcl::console;
  
  //double leafSize = 0.5;
 // double isoLevel = 0.5;
  
  if(iso_level > 1.0 || iso_level < 0.0){
    print_error("Wrong isolevel (%f) in MarchingCube",iso_level);
    return false;
  }
  // Create search tree*
  //pcl::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::KdTreeFLANN<pcl::PointNormal>);
  pcl::search::KdTree<PointNT>::Ptr tree2 (new pcl::search::KdTree<PointNT>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::MarchingCubesHoppe<PointNT> mc;
  
  // Set parameters
  mc.setIsoLevel(iso_level);   //ISO: must be between 0 and 1.0
  mc.setSearchMethod(tree2);
  mc.setGridResolution(leaf_size,leaf_size,leaf_size);
  mc.setPercentageExtendGrid(extend_grid_percentage);
  mc.setInputCloud(cloud_with_normals);

  // Reconstruct
  TicToc tt;
  tt.tic ();
  print_highlight("Reconstructing point cloud with the MarchingCube (Hoppe) algorithm....");
  mc.reconstruct (output);
  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
  
  return true;

}

////////////////////////////////////////////////////////////////////
///Convert point cloud to mesh by modifying vertex positions//
///////////////////////////////////////////////////////////////////
void ReconstructPointCloud::GridProjection(PointCloudNT::Ptr cloud_with_normals, pcl::PolygonMesh &output,
					    double resolution, int padding_size, int nearestNeighborNum,int setMaxBinarySearchLevel)
{
    using namespace pcl::console;
    
   // double resolution = 0.005;	//Set the size of the grid cell
   // int padding_size = 3;		// When averaging the vectors, we find the union of all the input data points within the padding area,and do a weighted average.
  //  int nearestNeighborNum = 100;
  //  int setMaxBinarySearchLevel = 10; //Binary search is used in projection.l
    
    // Create search tree*
    pcl::search::KdTree<PointNT>::Ptr tree2 (new pcl::search::KdTree<PointNT>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GridProjection<PointNT> gbpolygon;

    // Set parameters
    gbpolygon.setResolution(resolution);
    gbpolygon.setPaddingSize(padding_size);
    gbpolygon.setNearestNeighborNum(nearestNeighborNum);
    gbpolygon.setMaxBinarySearchLevel(setMaxBinarySearchLevel);
    
    // Get result
    gbpolygon.setInputCloud(cloud_with_normals);
    gbpolygon.setSearchMethod(tree2);
    
    TicToc tt;
    tt.tic ();
    print_highlight("Reconstructing point cloud with the GridProjection algorithm....");
    gbpolygon.reconstruct(output);
    print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}


PointCloudT::Ptr ReconstructPointCloud::ConvexHull(PointCloudT::Ptr cloud)
{
	pcl::ConvexHull<PointT> chull;

	PointCloudT::Ptr convex_hull(new PointCloudT);


	// Create search tree*
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	chull.setSearchMethod(tree);
	chull.setComputeAreaVolume(true);
	chull.setInputCloud(cloud);

	chull.reconstruct(*convex_hull);

	std::cout << "Total Area: " << chull.getTotalArea() << " Total Volume: " << chull.getTotalVolume() << std::endl;

	return convex_hull;

}
/////////////////////////////////////////////////////////////////////////////
/// Simple triangulation/surface reconstruction for organized point clouds.// 
/// No vertex modification						     //
/////////////////////////////////////////////////////////////////////////////
void ReconstructPointCloud::OrganizedFastMaesh(PointCloudT::Ptr cloud, pcl::PolygonMesh &output)
{
	// Initialize objects
	pcl::OrganizedFastMesh<PointT> orgMesh;
	//pcl::PolygonMesh triangles;

	// Create search tree*
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	orgMesh.setMaxEdgeLength (10);
	orgMesh.setTrianglePixelSize (1);
	orgMesh.setTriangulationType (pcl::OrganizedFastMesh<PointT>::TRIANGLE_ADAPTIVE_CUT );
	orgMesh.setInputCloud(cloud);
	orgMesh.setSearchMethod(tree);
	orgMesh.reconstruct(output);

}


pcl::PointCloud<pcl::PointXYZ>::Ptr ReconstructPointCloud::loadPCDfile(std::string file_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read the pcd file\n");
	}else
	{

	 std::cout << "Loaded "
	           << cloud->width * cloud->height
	           << " data points from"
	           << file_path
	           << " with the following fields: "
	           << std::endl;

	}

	return cloud;
}

void ReconstructPointCloud::saveToObj(const std::string file, pcl::PolygonMesh mesh)
{
	try{
		pcl::io::saveOBJFile(file,mesh);
	}catch(pcl::IOException &e)
	{
		std::cout << "Exception: In ReconstructPointCloud::saveToObj -> " << e.what() << std::endl;
	}
}

void ReconstructPointCloud::saveToVTK(const std::string file, pcl::PolygonMesh mesh)
{
	try{
		pcl::io::savePolygonFileVTK(file,mesh);
	}catch(pcl::IOException &e)
	{
		std::cout << "Exception: In ReconstructPointCloud::saveToVTK-> " << e.what() << std::endl;
	}
}

void ReconstructPointCloud::saveToSTL(const std::string file, pcl::PolygonMesh mesh)
{
	try{
		pcl::io::savePolygonFileSTL(file,mesh);
	}catch(pcl::IOException &e)
	{
		std::cout << "Exception: In ReconstructPointCloud::saveToSTL-> " << e.what() << std::endl;
	}
}

}
} /* namespace perception */
