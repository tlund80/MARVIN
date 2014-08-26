
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


namespace dti{ 
namespace grasp_planning
{

class Grasp_sampler
{
 
public:
  Grasp_sampler(Workcell *workcell);
  virtual ~Grasp_sampler();
  
   void LoadGripperInverseKin(std::string filename);
   void generateTask(GripperType type, std::string objectName, int nrTarget);
   void Sample(int samples);
   void SaveGraspTask(std::string fileName);
   
   rwlibs::task::GraspTask::Ptr getGraspTask(void){return _gtasks;};
   
  
private:
  
  Workcell *_workcell;
  SurfacePoseSampler::Ptr _sampler;
  SDHInvKinSolver* invKin;
  rw::proximity::CollisionDetector::Ptr cd;
  dti::grasp_planning::GripperType _grippertype;
  //std::string _grippertype;
  rwlibs::task::GraspTask::Ptr _gtasks;
  
  rw::models::Object::Ptr _object;
  rw::models::Device::Ptr _gripper;
  rw::kinematics::Frame::Ptr _gripperTcpFrame;
  rw::kinematics::MovableFrame::Ptr _gripperBase;
  
  rw::math::Vector3D<> projPlane(rw::math::Vector3D<> vec, rw::math::Vector3D<> normal);
  double angleProj(rw::math::Vector3D<> v1, rw::math::Vector3D<> v2, rw::math::Vector3D<> normal);

  //Hand open and close configuration
  rw::math::Q openQ;
  rw::math::Q closeQ;
  rw::math::Q minQ;
  rw::math::Q maxQ;
  
  
};

}
}
#endif


