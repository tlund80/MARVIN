/**
 * @file /include/object_modeller_gui/ObjectModeller.h
 *
 * @brief Communications central!
 *
 * @date November 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

//#ifndef OBJECTMODELLERGUI_ROSINTERFACE_HPP_
//#define OBJECTMODELLERGUI_ROSINTERFACE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

//--------------PCL Includes-------------------
//#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
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

//--------------ROS Includes -------------------
//#include <ros/package.h>
//#include <ros/ros.h>
//#include <tf/tf.h>
//#include <tf_conversions/tf_eigen.h>
//#include <pcl_ros/publisher.h>

//-------------Qt-------------------------------
#include <QThread>
#include <QObject>

#include <Eigen/Geometry>

#include <one_shot_learning/SharedData.hpp>
#include <one_shot_learning/ReconstructPointCloud.hpp>

namespace dti{
namespace one_shot_learning {

/*****************************************************************************
** Class
*****************************************************************************/

class ObjectModeller : public QThread {
    Q_OBJECT
public:
	ObjectModeller(SharedData *data);
	virtual ~ObjectModeller();
	
	void run();
	void shutdown();
	void createmodel(pcl::PointCloud<pcl::PointXYZRGBA> in);
	void createSolidmodel(pcl::PointCloud<pcl::PointXYZRGBA> in);
	void alignGTModelAndSceneModel(pcl::PointCloud<pcl::PointXYZRGBA> src, pcl::PointCloud<pcl::PointXYZRGBA> tar);
	

Q_SIGNALS:
	void consoleSignal(QString msg);
	void modelCreated(pcl::PointCloud<pcl::PointXYZRGBA> model);
	void solidModelCreated(pcl::PointCloud<pcl::PointXYZRGBA> model, pcl::PolygonMesh solid_model);
	void turnModel();

private:
  
  void initial_alignment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud,
			  Eigen::Matrix4f &final_transform, bool downsample);
  
  bool pairAlign (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud,
		  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &output, Eigen::Matrix4f &final_transform, bool downsample);


  void EstimateNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud,pcl::PointCloud<pcl::PointNormal>::Ptr &target_cloud);
  
  void removePlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud,
		   double dist_threads);
  
  void CropBox(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud,
             float rx, float ry, float minz, float maxz, Eigen::Vector3f boxTranslatation, Eigen::Vector3f boxRotation);

  bool extractClusters(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > &clusters);
  
  void radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud, double radius);
  
  void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target_cloud, double mean);
  
  bool isRunning;
  bool doModel;
  bool doSolidModel;
  bool doModelAlignment;
  SharedData *_sharedData;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _tar;
  ReconstructPointCloud *reconstruct;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > views;
  
};
}
}  // namespace object_modeller_gui

//#endif /* OBJECTMODELLERGUI_ROSINTERFACE_HPP_ */
