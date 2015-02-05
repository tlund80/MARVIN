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
	void createmodel(pcl::PointCloud<pcl::PointXYZRGBA> in);
	void createSolidmodel(pcl::PointCloud<pcl::PointXYZRGBA> in);
	void alignGTModelAndSceneModel(pcl::PointCloud<pcl::PointXYZRGBA> src, pcl::PointCloud<pcl::PointXYZRGBA> tar);
	void getCloudModel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &model);
	void getMeshModel(pcl::PolygonMesh &mesh);
	
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
  
  QMutex					_mutexCloud;
  QMutex					_mutexMesh;
  
//  bool 						isRunning;
  bool 						doModel;
  bool 						doSolidModel;
  bool 						doModelAlignment;
  SharedData 					*_sharedData;
  pcl::PolygonMesh	 			_mesh;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 	_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 	_tar;
  ReconstructPointCloud 			*reconstruct;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > views;
  
};
}
}  // namespace object_modeller_gui

#endif /* OBJECTMODELLER_HPP */
