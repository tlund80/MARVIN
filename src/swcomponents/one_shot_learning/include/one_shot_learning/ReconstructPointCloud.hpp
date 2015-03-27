  /*
 * ReconstructPointCloud.h
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#ifndef RECONSTRUCTPOINTCLOUD_H_
#define RECONSTRUCTPOINTCLOUD_H_

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace dti{
namespace one_shot_learning {

class ReconstructPointCloud {
public:
	ReconstructPointCloud();
	virtual ~ReconstructPointCloud();

	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	bool OrganizedMedianFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, int window_size, float movement);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void FrustumCullingFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, 
				   float vertical_FOV, float horizontal_FOV, 
				   float near_plane_dist, float far_plane_dist, Eigen::Matrix4f camera_pose);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void ShadowFilter(PointCloudNT::Ptr &src_cloud, PointCloudNT::Ptr &tar_cloud, float threshold, pcl::PointIndices::Ptr removedIndices);
  
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void BilateralFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, double sigma_s, double sigma_r);
 
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	//void NormalSpaceSampling(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, unsigned int bin_size);
 
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void LocalMaximumDownSampling(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, const float radius);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void PlaneClippingFilter(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, const Eigen::Vector4f &plane_params);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void MLSApproximation(PointCloudT::Ptr &cloud, PointCloudNT::Ptr &target,  double search_radius);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	bool BilateralUpsampling(PointCloudT::Ptr cloud, PointCloudT::Ptr output,int window_size = 5, float sigma_color = 15.0f, float sigma_depth = 0.5f);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void RandomSampling(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &tar_cloud, const unsigned int samples);
 
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void poisson(const PointCloudNT::Ptr &cloud, pcl::PolygonMesh &output,int depth, int solver_divide, int iso_divide, float point_weight);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void GreedyProjectionTriangulation(PointCloudNT::Ptr cloud_with_normals, pcl::PolygonMesh &output, double SearchRadius);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	bool MarchingCubes(PointCloudNT::Ptr cloud_with_normals, pcl::PolygonMesh &output,
			   int leaf_size, double iso_level = 0.5, float extend_grid_percentage = 10.0);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void GridProjection(PointCloudNT::Ptr cloud_with_normals, pcl::PolygonMesh &output, double resolution =  0.005,
			    int padding_size = 3, int nearestNeighborNum = 100 ,int setMaxBinarySearchLevel = 10);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void OrganizedFastMaesh(PointCloudT::Ptr cloud, pcl::PolygonMesh &output);

	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	PointCloudT::Ptr ConvexHull(PointCloudT::Ptr cloud);
        
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCDfile(std::string file_path);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void saveToObj(const std::string file, pcl::PolygonMesh mesh);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void saveToVTK(const std::string file, pcl::PolygonMesh mesh);
	
	/*! \brief Brief description.
	 *      Brief description continued.
	 *  	Detailed description starts here.
	 */
	void saveToSTL(const std::string file, pcl::PolygonMesh mesh);
};

}
} /* namespace object_modeller_gui */
#endif /* RECONSTRUCTPOINTCLOUD_H_ */
