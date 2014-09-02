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
#include <rw/geometry.hpp>

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
   rwlibs::task::GraspTask _gtask;
   rwlibs::task::GraspTask _gtask_GT;
   bool  _has_grasp_table;
   bool  _has_grasp_table_GT;
   
 //  rw::graphics::Model3D _rwmodel;
 //  rw::geometry::Geometry _geometry;

public:
  
//   void setRwModel3D(rw::graphics::Model3D &rwmodel){_rwmodel = rwmodel; };
//   rw::graphics::Model3D getRwModel3D(void){return _rwmodel; };
   
//   void setRwGeometry(rw::geometry::Geometry &geometry){_geometry = geometry; };
//   rw::geometry::Geometry getRwGeometry(void){return _geometry; };
  
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
   
   void setGraspTask(rwlibs::task::GraspTask gtask){
     _gtask = gtask;
     _has_grasp_table = true;
  };
   rwlibs::task::GraspTask getGraspTask(void){return _gtask; };
   
   void setGTGraspTask(rwlibs::task::GraspTask gtask){
     _gtask_GT = gtask;
     _has_grasp_table_GT = true;
  };
   rwlibs::task::GraspTask getGTGraspTask(void){return _gtask_GT; };
   
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
















