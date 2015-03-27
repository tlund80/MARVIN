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

#ifndef OBJECTMODELLER_HPP
#define OBJECTMODELLER_HPP

/*****************************************************************************
** Includes
*****************************************************************************/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

//-------------Qt-------------------------------
#include <QThread>
#include <QRunnable>
#include <QObject>

#include <Eigen/Geometry>

#include <one_shot_learning/SharedData.hpp>
#include <one_shot_learning/ReconstructPointCloud.hpp>

namespace dti{
namespace one_shot_learning {

/*****************************************************************************
** Class
*****************************************************************************/

class ObjectModeller : public QObject {
    Q_OBJECT
public:
	ObjectModeller(SharedData *data, QObject *parent = 0);
	virtual ~ObjectModeller();
	
	void shutdown();
	void createmodel(PointCloudT in);
	void createSolidmodel(PointCloudT in);
	void alignGTModelAndSceneModel(PointCloudT src, PointCloudT tar);
	void getRawModel(PointCloudT::Ptr &model);
	void getPostProcessedModel(PointCloudT::Ptr &model);
	void getMeshModel(pcl::PolygonMesh &mesh);
	void clear(){views.clear();};
	
public Q_SLOTS:	
	void run();	

Q_SIGNALS:
	void consoleSignal(QString msg);
	void modelCreated();
	void solidModelCreated();
	void turnModel();
	void finished();

protected:
	// virtual void run();	
private:
  
  void initial_alignment(PointCloudNT::Ptr src_cloud,PointCloudNT::Ptr target_cloud,
			  Eigen::Matrix4f &final_transform, bool downsample = true,
			  bool source_has_normals = true, bool target_has_normals = true);
  
  bool pairAlign (PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud,
		  PointCloudT::Ptr &output, Eigen::Matrix4f &final_transform, bool downsample);


  void EstimateNormals(PointCloudT::Ptr &src_cloud,PointCloudNT::Ptr &target_cloud);
  
/*  void removePlane(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud,
		   double dist_threads);
  
  void CropBox(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud,
             float rx, float ry, float minz, float maxz, Eigen::Vector3f boxTranslatation, Eigen::Vector3f boxRotation);

  bool extractClusters(PointCloudT::Ptr &src_cloud, std::vector<PointCloudT::Ptr, Eigen::aligned_allocator_indirection<PointCloudT::Ptr> > &clusters);
 */ 
  void radiusOutlierRemoval(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud, double radius, int min_neighbpr_pts = 1);
  
  void statisticalOutlierRemoval(PointCloudT::Ptr &src_cloud, PointCloudT::Ptr &target_cloud, int mean_num_pts = 1);
  
  Eigen::Matrix4f rotateX(float radians);
  Eigen::Matrix4f rotateY(float radians);
  Eigen::Matrix4f rotateZ(float radians);
  bool my_sort(std::pair<float,Eigen::Matrix4f> i,std::pair<float,Eigen::Matrix4f> j) { return (i.first < j.first); }

  
  QMutex					_mutexCloud;
  QMutex					_mutexMesh;
  
//  bool 						isRunning;
  bool 						doModel;
  bool 						doSolidModel;
  bool 						doModelAlignment;
  float						leaf_size_;
  SharedData 					*_sharedData;
  pcl::PolygonMesh	 			_mesh;
  PointCloudT::Ptr				 	_post_processed_cloud;
  PointCloudNT::Ptr				 	_post_processed_cloud_with_normals;
  PointCloudT::Ptr				 	_raw_model;
  PointCloudNT::Ptr				 	_tar;
  ReconstructPointCloud 			*reconstruct;
  std::vector<PointCloudT::Ptr, Eigen::aligned_allocator_indirection<PointCloudT::Ptr> > views;
  
};
}
}  // namespace object_modeller_gui

#endif /* OBJECTMODELLER_HPP */
