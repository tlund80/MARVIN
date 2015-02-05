/*
 * ReconstructPointCloud.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#include <one_shot_learning/ReconstructPointCloud.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/bilateral_upsampling.h>

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
/////////////////////////////////////////////////////
//  Data smoothing and improved normal estimation////
/////////////////////////////////////////////////////
void ReconstructPointCloud::MLSApproximation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target)
{
	 using namespace pcl::console;
	 
	 // Create a KD-Tree
	  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);

	  // Output has the PointNormal type in order to store the normals calculated by MLS
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  // Init object (second point type is for the normals, even if unused)
	  pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA> mls;

	  mls.setComputeNormals (true);

	  // Set parameters
	  mls.setInputCloud (cloud);
	  mls.setPolynomialFit (true);
	  mls.setSearchMethod (tree);
	  mls.setSearchRadius (0.025); //0.03
	  mls.setComputeNormals(false);
	  //mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::UpsamplingMethod::VOXEL_GRID_DILATION);

	  // Reconstruct
	  TicToc tt;
	  tt.tic ();
	  print_highlight("Computing smoothed point cloud using MLS algorithm....");
	  mls.process (*mls_points);
	  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

	  pcl::copyPointCloud(*mls_points, *target);

}

bool ReconstructPointCloud::BilateralUpsampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output,
         int window_size, double sigma_color, double sigma_depth)
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
		print_error("Input cloud is not organized in ReconstructPointCloud::BilateralUpsampling()\n");
		return false;
	}

}

void ReconstructPointCloud::poisson (const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, pcl::PolygonMesh &output,
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
  print_highlight ("Computing ...");
  poisson.reconstruct (output);

  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

////////////////////////////////////////////////////////////////////
////Convert point cloud to mesh without modifying vertex positions//
///////////////////////////////////////////////////////////////////
pcl::PolygonMesh ReconstructPointCloud::GreedyProjectionTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double SearchRadius)
{
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
	// Compute the mesh
	pcl::PolygonMesh triangles;
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	 std::cout << "GreedyProjectionTriangulation parts: " << (int)parts.size()<< std::endl;

	return triangles;
}

////////////////////////////////////////////////////////////////////
////Convert point cloud to mesh by modifying vertex positions//
///////////////////////////////////////////////////////////////////
pcl::PolygonMesh ReconstructPointCloud::MarchingCubes(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
	//double leafSize = 0.5;
    double isoLevel = 0.5;

	// Create search tree*
	//pcl::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::KdTreeFLANN<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

	pcl::PolygonMesh triangles;

	// Initialize objects
	pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
	// Set parameters
	mc.setIsoLevel(isoLevel);   //ISO: must be between 0 and 1.0
	mc.setSearchMethod(tree2);
	mc.setInputCloud(cloud_with_normals);

	// Reconstruct
	mc.reconstruct (triangles);

	return triangles;

}

////////////////////////////////////////////////////////////////////
////Convert point cloud to mesh by modifying vertex positions//
///////////////////////////////////////////////////////////////////
pcl::PolygonMesh ReconstructPointCloud::GridProjection(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
	double resolution = 0.005;	//Set the size of the grid cell
	int padding_size = 3;		// When averaging the vectors, we find the union of all the input data points within the padding area,and do a weighted average.
	int nearestNeighborNum = 100;
	int setMaxBinarySearchLevel = 10; //Binary search is used in projection.l

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GridProjection<pcl::PointNormal> gbpolygon;
	pcl::PolygonMesh triangles;

	// Set parameters
	gbpolygon.setResolution(resolution);
	gbpolygon.setPaddingSize(padding_size);
	gbpolygon.setNearestNeighborNum(nearestNeighborNum);
	gbpolygon.setMaxBinarySearchLevel(setMaxBinarySearchLevel);

	// Get result
	gbpolygon.setInputCloud(cloud_with_normals);
	gbpolygon.setSearchMethod(tree2);
	gbpolygon.reconstruct(triangles);

	return triangles;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr ReconstructPointCloud::ConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::ConvexHull<pcl::PointXYZ> chull;

	pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull(new pcl::PointCloud<pcl::PointXYZ>);


	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	chull.setSearchMethod(tree);
	chull.setComputeAreaVolume(true);
	chull.setInputCloud(cloud);

	chull.reconstruct(*convex_hull);

	std::cout << "Total Area: " << chull.getTotalArea() << " Total Volume: " << chull.getTotalVolume() << std::endl;

	return convex_hull;

}
/////////////////////////////////////////////////////////////////////////////
/// Simple triangulation/surface reconstruction for organized point clouds //
/////////////////////////////////////////////////////////////////////////////
pcl::PolygonMesh ReconstructPointCloud::OrganizedFastMaesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Initialize objects
	pcl::OrganizedFastMesh<pcl::PointXYZ> orgMesh;
	pcl::PolygonMesh triangles;

	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	orgMesh.setMaxEdgeLength (10);
	orgMesh.setTrianglePixelSize (1);
	orgMesh.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT );
	orgMesh.setInputCloud(cloud);
	orgMesh.setSearchMethod(tree);
	orgMesh.reconstruct(triangles);

	return triangles;
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
		pcl::io::saveVTKFile(file,mesh);
	}catch(pcl::IOException &e)
	{
		std::cout << "Exception: In ReconstructPointCloud::saveToVTK-> " << e.what() << std::endl;
	}
}

}
} /* namespace perception */
