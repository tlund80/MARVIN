/*
 * ReconstructPointCloud.h
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#ifndef RECONSTRUCTPOINTCLOUD_H_
#define RECONSTRUCTPOINTCLOUD_H_

#include <pcl/point_types.h>
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

class ReconstructPointCloud {
public:
	ReconstructPointCloud();
	virtual ~ReconstructPointCloud();

	void MLSApproximation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target);
	bool BilateralUpsampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output,int window_size, double sigma_color, double sigma_depth);

	void poisson(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, pcl::PolygonMesh &output,int depth, int solver_divide, int iso_divide, float point_weight);
	pcl::PolygonMesh GreedyProjectionTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double SearchRadius);
	pcl::PolygonMesh MarchingCubes(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
	pcl::PolygonMesh GridProjection(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
	pcl::PolygonMesh OrganizedFastMaesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCDfile(std::string file_path);
	void saveToObj(const std::string file, pcl::PolygonMesh mesh);
	void saveToVTK(const std::string file, pcl::PolygonMesh mesh);
};

}
} /* namespace object_modeller_gui */
#endif /* RECONSTRUCTPOINTCLOUD_H_ */
