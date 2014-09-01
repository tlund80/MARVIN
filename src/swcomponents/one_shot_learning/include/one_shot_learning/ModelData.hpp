#ifndef MODELDATA_HPP
#define MODELDATA_HPP

#include <QList>
#include <QMap>
#include <QString>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <qt4/QtCore/QList>
#include <qt4/QtCore/qmap.h>

#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>

namespace dti{
namespace one_shot_learning {
class ModelData
{
  
public:
     ModelData(int index);
     virtual ~ModelData();

private:
  
   pcl::PolygonMesh _meshData;
   pcl::PointCloud<pcl::PointXYZRGBA> _pcdData;
   pcl::PointCloud<pcl::PointXYZRGBA>  _gtCloudModel;
   pcl::PolygonMesh  _gtMeshModel;
   bool  _has_grasp_table;
   bool  _has_grasp_table_GT;
   bool _has_GTMeshModel;
   bool _has_GTCloudModel;
   QString _name;
   rwlibs::task::GraspTask _gtask;
   rwlibs::task::GraspTask _gtask_GT;
   int _index;
   std::vector<rw::math::Transform3D<double> > _poses;
   
   // QList<pcl::PolygonMesh> meshData;
   // QList<pcl::PointCloud<pcl::PointXYZRGBA> > pcdData;
   // QMap<int, pcl::PointCloud<pcl::PointXYZRGBA> >  gtModel;
   // QMap<int, pcl::PolygonMesh >  gtMeshModel;
   // QMap<int, bool >  _has_grasp_table;
   // QList<QString> name;

public:
  
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
   int getIndex(void){return _index; };
  
};

}
}
#endif /* MODELDATA_HPP */
















