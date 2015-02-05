#ifndef GRASP_SIMULATOR_HPP
#define GRASP_SIMULATOR_HPP

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
#include <QObject>
#include <QMutex>

namespace dti{
  namespace grasp_planning
  {
 enum FileFormat{RWTASK = 0, UIBK, Text};
class Grasp_simulator  : public QThread {
    Q_OBJECT
public:
  Grasp_simulator(Grasp_sampler *sampler);
  virtual ~Grasp_simulator();
  

  bool SimulateGraspHypothesis();
  void PauseSimulator();
  void ResumeSimulator();
  
  void stop();
  void RecordStatePath(bool record, std::string file_path){
    _recordStatePath = record;
    _record_file_path = file_path;
  }; 
  void TimedState(int arg1, rw::kinematics::State initState);
  
  void setObjectName(std::string object_name);
  void setTaskXMLFilePath(std::string taskXMLFile = "");
  void setPertubationData(double sigma_a, double sigma_p,  unsigned int pertubationsPerTarget= 100);
  void setSavePath(std::string& filename, FileFormat format);
  void setWorkcell(dti::grasp_planning::Workcell* workcell);
  int getPercentDone();
  //void setGraspTask(rwlibs::task::GraspTask::Ptr grasptask){_grasptask = grasptask; };
  
protected:
  void run();
  
Q_SIGNALS:
  void finish();
  void status(double percent);
  
private:
  Workcell* 					_workcell;
  Grasp_sampler* 				_sampler;
  rwsim::simulator::GraspTaskSimulator::Ptr 	graspSim;
  rwsim::dynamics::DynamicWorkCell::Ptr 	_dwc;

  bool						_pause_sim;
  bool						_resume_sim;
  QMutex					_mutexPercentDone;
  int 						_percent_done;
  std::string 					_object_name;
  std::string					_taskXMLFile;
  //Input Grasp task
 rwlibs::task::GraspTask::Ptr 			_grasptask;
 
 //Grasp task to hold all filtered subTasks
 rwlibs::task::GraspTask::Ptr 			_gtask;
 
 //Timed state path for grasp simulation playback
 rw::trajectory::TimedStatePath 		statep;
 bool 						_recordStatePath;
 std::string 					_record_file_path;
 int 						_numberOfThreads;
 bool	 					_isRunning; 
 std::map<int,bool> 				includeMap;
 
 double 					_sigma_a; 
 double 					_sigma_p; 
 unsigned int 					_pertubations;
 
 std::string					_filename;
 FileFormat 					_format;
 bool						_save_gtask;
 
private:
  
  void FilterGrasps();
  bool SimulateGraspPertubations();
  bool calcPerturbedQuality( );
  void SaveGraspTask();
 
  void addPertubations(rwlibs::task::GraspTask::Ptr grasptask, double sigma_p, double sigma_a, int pertubationsPerTarget);
  int calcPerturbedQuality(rwlibs::task::GraspTask::Ptr gtask, std::string outfile, int pertubations );
  std::vector<rwlibs::task::GraspTask::Ptr> splitTask(rwlibs::task::GraspTask::Ptr grasptask, int split);
  
};
}
}
#endif /* GRASP_SIMULATOR_HPP */

