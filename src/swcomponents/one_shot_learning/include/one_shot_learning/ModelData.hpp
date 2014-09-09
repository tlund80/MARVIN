#ifndef MODELDATA_HPP
#define MODELDATA_HPP

#include <QList>
#include <QMap>
#include <QString>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <rw/graphics/Model3D.hpp>
#include <rw/geometry/Geometry.hpp>

namespace dti{
namespace one_shot_learning {
class ModelData
{
  
public:
     ModelData();
     virtual ~ModelData();

private:
   // Name of the object
   QString _name;
   
   // Estimated and ground truth models for pose estimation
   pcl::PolygonMesh _meshData;
   pcl::PointCloud<pcl::PointXYZRGBA> _pcdData;
   pcl::PointCloud<pcl::PointXYZRGBA>  _gtCloudModel;
   pcl::PolygonMesh  _gtMeshModel;
   bool _has_GTMeshModel;
   bool _has_GTCloudModel;
   
   //Pose estimates of the object
   std::vector<rw::math::Transform3D<double> > _poses;

   // Grasp tables of the estimated and ground truth model
   std::vector<rwlibs::task::GraspTask::Ptr> _gtask_list;
   std::vector<rwlibs::task::GraspTask::Ptr> _gtask_GT_list;
  // rwlibs::task::GraspTask _gtask;
 //  rwlibs::task::GraspTask _gtask_GT;
   bool  _has_grasp_table;
   bool  _has_grasp_table_GT;
   
   rw::graphics::Model3D::Ptr _rwmodel;
   rw::geometry::Geometry::Ptr _geometry;
   rw::graphics::Model3D::Ptr _gt_rwmodel;
   rw::geometry::Geometry::Ptr _gt_geometry;

public:
  
   void setRwModel3D(rw::graphics::Model3D::Ptr &rwmodel){_rwmodel = rwmodel; };
   rw::graphics::Model3D::Ptr getRwModel3D(void){return _rwmodel; };
   
   void setRwGeometry(rw::geometry::Geometry::Ptr &geometry){_geometry = geometry; };
   rw::geometry::Geometry::Ptr getRwGeometry(void){return _geometry; };
  
   void setGTRwModel3D(rw::graphics::Model3D::Ptr &rwmodel){_gt_rwmodel = rwmodel; };
   rw::graphics::Model3D::Ptr getGTRwModel3D(void){return _gt_rwmodel; };
   
   void setGTRwGeometry(rw::geometry::Geometry::Ptr &geometry){_gt_geometry = geometry; };
   rw::geometry::Geometry::Ptr getGTRwGeometry(void){return _gt_geometry; };
  
   void setPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloud){_pcdData = cloud; };
   pcl::PointCloud<pcl::PointXYZRGBA> getPointCloud(void){return _pcdData; };
   
   void setMesh(pcl::PolygonMesh mesh){_meshData = mesh; };
   pcl::PolygonMesh getMesh(void){return _meshData; };
   
   void setGTPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloud){
     _gtCloudModel = cloud; 
     _has_GTCloudModel = true;
  };
   pcl::PointCloud<pcl::PointXYZRGBA> getGTPointCloud(void){return _gtCloudModel; };
   
   void setGTMesh(pcl::PolygonMesh mesh){
     _gtMeshModel = mesh;
     _has_GTMeshModel = true;
  };
   pcl::PolygonMesh getGTMesh(void){return _gtMeshModel; };
   
   void addGraspTask(rwlibs::task::GraspTask::Ptr gtask){
    // _gtask = gtask;
    _gtask_list.push_back(gtask);
     _has_grasp_table = true;
  };
   rwlibs::task::GraspTask::Ptr getFirstGraspTask(void){return _gtask_list.front(); };
   
   void addGTGraspTask(rwlibs::task::GraspTask::Ptr gtask){
     _gtask_GT_list.push_back(gtask);
     _has_grasp_table_GT = true;
  };
   rwlibs::task::GraspTask::Ptr getGTGraspTask(void){return _gtask_GT_list.front(); };
   
   void setName(QString name){_name = name; };
   QString getName(void){return _name; };
   
   void addPoseHypothesis(rw::math::Transform3D<double> pose){_poses.push_back(pose); };
   rw::math::Transform3D<double> getBestPose(void){return _poses[0];};
   void getPoseHypothesis( std::vector<rw::math::Transform3D<double> > &hypothesis){hypothesis = _poses; };
  
   bool has_gtask(void) {return _has_grasp_table; };
   bool has_ground_truth_gtask(void) {return _has_grasp_table_GT; };
   bool has_ground_truth_mesh(void) {return _has_GTMeshModel; };
   bool has_ground_truth_cloud(void) {return _has_GTCloudModel; };
  /* void delete_PointCloud(void){ _pcdData = NULL;};
   void delete_Mesh(void){ _meshData = NULL;};
   void delete_PointCloud_GT(void){ _gtCloudModel = NULL;};
   void delete_Mesh_GT(void){ _gtMeshModel = NULL;};
   */
  
};

}
}
#endif /* MODELDATA_HPP */
















