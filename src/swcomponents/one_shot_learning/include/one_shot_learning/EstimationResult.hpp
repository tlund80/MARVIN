#ifndef ESTIMATIONRESULT_HPP
#define ESTIMATIONRESULT_HPP

#include <QString>
#include <geometry_msgs/Transform.h>

namespace dti{
namespace one_shot_learning {
class EstimationResult
{
  
public:
     EstimationResult();
     virtual ~EstimationResult();

private:
  
  double _error;
  double _inlier;
  QString _name;
  QString _id;
  geometry_msgs::Transform _transformation;
  int _scene_instances;
  QString _scene_name;

public:
  
  double getError(void){return _error;};
  double getInlierFraction(void){return _inlier;};
  QString getName(void){return _name;};
  QString getId(void){return _id;};
  geometry_msgs::Transform getPose(void){return _transformation;};
  int getSceneInstances(void){return _scene_instances;};
  QString getSceneName(void){return _scene_name;};
  
  void setError(double error){_error = error;};
  void setInlierFraction(double inlier_fraction){_inlier = inlier_fraction;};
  void setName(QString name){_name = name;};
  void setId(QString id){_id = id;};
  void setPose(geometry_msgs::Transform pose){_transformation = pose;};
  void setSceneInstances(int instances){_scene_instances = instances;};
  void setSceneName(QString scene_name){_scene_name = scene_name;};
  
  void addToLog(QString fileName, bool append);
  
    
};

}
}
#endif /* ESTIMATIONRESULT_HPP */