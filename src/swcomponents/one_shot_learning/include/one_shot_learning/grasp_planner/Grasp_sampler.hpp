
#ifndef GRASP_SAMPLER_HPP_
#define GRASP_SAMPLER_HPP_

/**
 * @file Grasp_sampler.hpp
 */

#include <one_shot_learning/grasp_planner/Workcell.hpp>
#include <one_shot_learning/grasp_planner/SDHInvKinSolver.hpp>

#include <rwsim/util/SurfacePoseSampler.hpp>
#include <rw/rw.hpp>

#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <QThread>
#include <QObject>
#include <QMutex>


namespace dti{ 
namespace grasp_planning
{

class Grasp_sampler : public QThread {
      Q_OBJECT
public:
  Grasp_sampler();
  virtual ~Grasp_sampler();

   void SaveGraspTask(std::string fileName);
   
   rwlibs::task::GraspTask::Ptr getGraspTask(void){return _gtasks;};
   
   void setGripperInverseKinPath(std::string path);
   void setGripperType(GripperType type);
   void setObjectName(std::string object_name);
   void setNumberOfTargets(unsigned int targets);
   void setSamples(unsigned int samples);
   void setWorkcell(dti::grasp_planning::Workcell *workcell);
  
protected:
  void run();
  
private:
    
   void LoadGripperInverseKin(void);
   bool generateTask();
   bool Sample();
   
Q_SIGNALS:
  void finish_sampling(bool finish);
  void status(double percent);
  
private:
  
  Workcell 					*_workcell;
  SurfacePoseSampler::Ptr 			_sampler;
  SDHInvKinSolver 				*invKin;
  rw::proximity::CollisionDetector::Ptr 	cd;

  //std::string _grippertype;
  rwlibs::task::GraspTask::Ptr 		_gtasks;
  
  rw::models::Object::Ptr 			_object;
  rw::models::Device::Ptr 			_gripper;
  rw::kinematics::Frame::Ptr 			_gripperTcpFrame;
  rw::kinematics::MovableFrame::Ptr 		_gripperBase;
  
  //Hand open and close configuration
  rw::math::Q 					openQ;
  rw::math::Q 					closeQ;
  rw::math::Q					minQ;
  rw::math::Q 					maxQ;
  
  std::string 					_path_to_gripper_kin;
  dti::grasp_planning::GripperType 		_grippertype;
  std::string					_object_name;
  unsigned int					_nrTarget;
  unsigned int					_samples;
  bool 						_running;
  
  QMutex					_mutex_gripper_kin; 
  QMutex					_mutex_gripper_type; 
  QMutex					_mutex_object_name; 
  QMutex					_mutex_nrTarget; 
  QMutex					_mutex_samples; 
  QMutex					_mutex_workcell; 
	
  
 rw::math::Vector3D<> projPlane(rw::math::Vector3D<> vec, rw::math::Vector3D<> normal);
  double angleProj(rw::math::Vector3D<> v1, rw::math::Vector3D<> v2, rw::math::Vector3D<> normal);

  
};

}
}
#endif


