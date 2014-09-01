#ifndef GRASP_SIMULATOR_HPP_
#define GRASP_SIMULATOR_HPP_

/**
 * @file Grasp_simulator.hpp
 */

#include <one_shot_learning/grasp_planner/Workcell.hpp>
#include <one_shot_learning/grasp_planner/Grasp_sampler.hpp>
//#include <one_shot_learning/grasp_planner/SDHInvKinSolver.hpp>

#include <rwsim/util/SurfacePoseSampler.hpp>
#include <rw/rw.hpp>

#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <QThread>

namespace dti{
  namespace grasp_planning
  {
 enum FileFormat{RWTASK = 0, UIBK, Text};
class Grasp_simulator  : public QThread 
{

public:
  Grasp_simulator(Workcell *workcell, Grasp_sampler *sampler);
  virtual ~Grasp_simulator();
  
  bool SimulateGraspHypothesis(std::string objectName, std::string taskXMLFile = "");
  void FilterGrasps();
  bool SimulateGraspPertubations(double sigma_a, double sigma_p,  unsigned int pertubationsPerTarget= 100, std::string taskXMLFile = "");
  int calcPerturbedQuality(unsigned int pertubations = 100 );
  void SaveGraspTask(std::string& filename, FileFormat file);
  void stop();
  void RecordStatePath(bool record, std::string file_path)
  {
    _recordStatePath = record;
    _record_file_path = file_path;
  }; 
  //void setGraspTask(rwlibs::task::GraspTask::Ptr grasptask){_grasptask = grasptask; };
  
private:
    void run();
  
private:
  Workcell* 			_workcell;
  Grasp_sampler* 		_sampler;
  //Input Grasp task
 rwlibs::task::GraspTask::Ptr 	_grasptask;
 
 //Grasp task to hold all filtered subTasks
 rwlibs::task::GraspTask::Ptr 	_gtask;
 
 //Timed state path for grasp simulation playback
 rw::trajectory::TimedStatePath statep;
 bool 				_recordStatePath;
 std::string 			_record_file_path;
 int 				_numberOfThreads;
 bool	 			_isRunning; 
 std::map<int,bool> 		includeMap;
 
 void addPertubations(rwlibs::task::GraspTask::Ptr grasptask, double sigma_p, double sigma_a, int pertubationsPerTarget);
 int calcPerturbedQuality(rwlibs::task::GraspTask::Ptr gtask, std::string outfile, int pertubations );
 std::vector<rwlibs::task::GraspTask::Ptr> splitTask(rwlibs::task::GraspTask::Ptr grasptask, int split);
  
};
  }
}


#endif

