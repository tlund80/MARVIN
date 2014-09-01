
#include <one_shot_learning/motion_planner/RobotController.hpp>

#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rw/math.hpp>

namespace dti{
namespace one_shot_learning {
namespace motion_planner {

RobotController::RobotController(one_shot_learning::RosCommunication *rosComm) 
: _rosComm(rosComm)
{
  qRegisterMetaType<rw::trajectory::Path<rw::math::Q> >("rw::trajectory::Path<rw::math::Q>");
  
  _isRunning = false;
  _state_done = true;
  _sim = false;
  _speed = 0.5;
  
   connect(_rosComm, SIGNAL(finish()), this, SLOT(motionDone()), Qt::UniqueConnection);
   connect(_rosComm, SIGNAL(robotPose(rw::math::Q, QString)), this, SLOT(updateRobotQ(rw::math::Q, QString)), Qt::DirectConnection);

  
}

RobotController::~RobotController()
{
  stop();
}

void RobotController::initialize(rw::models::WorkCell::Ptr workcell, std::string robot)
{
    _rwc = workcell;
    _robot_name = robot;
    
    planner_config.reset(new PRMConfig());
    planner_config->setCollisionCheckingStrategy(PlannerConfiguration::LAZY);
    planner_config->setMaxTime(5.0);
    planner_config->setNeighborSearchStrategy(PlannerConfiguration::BRUTE_FORCE);
    planner_config->setResolution(0.01);
    planner_config->setRoadmapNodecount(1000);
    planner_config->setShortestPathSearchStrategy(PlannerConfiguration::DIJKSTRA);
    
    _path_length_config.reset(new PathLengthConfig());
    _path_length_config->setResolution(0.01);
    
    _planning_workcell.reset(new dti::grasp_planning::Workcell());
    _planning_workcell->init_workcell(_rwc, robot);
    
     planner.reset(new dti::one_shot_learning::motion_planner::Planner(_planning_workcell, planner_config ));  
    _path_optimizer.reset(new Path_optimization(_planning_workcell, _path_length_config));
    
   
   // _currentQ = rw::math::Q(6, -2.001,-1.571,-1.951,-1.572,0,0);
  
  
}
void RobotController::run()
{

 _isRunning = true;
 
 while(_isRunning)
 {
   if(_state_done)
   { 
    switch(_motionState)
    {
      case IDLE:
      {
	//NOP
	
	break;
      }
      case MOVE_TO_GRASP:
      {
	//Move to approach pose
	 std::cout << "RobotController state = MOVE_TO_GRASP" << std::endl; 
	 
	 std::vector<rw::math::Q> solution;
	 std::vector<float> _blend_radius;
	 _path.clear();
	 
	  rw::math::Q initQ(6, 0.0); 
	  if(_planning_workcell->solve_invkin(_object_pose,false,initQ,solution, true, false))
	  {
	    std::cout << "RobotController: Planning.....!" << std::endl;
	    if(planner->plan(_currentQ,solution[0],_path)){
	      
	      std::cout << "\n=================== Computed Path ===================" << std::endl;
	      for(int i = 0; i<=int(_path.size())-1; i++)
		  std::cout << "\t" << _path[i] << std::endl;
	      
	      _path_optimizer->optimize(_path);
	       std::cout << "\n=================== Optimized Path ===================" << std::endl;
	       for(int i = 0; i<= int(_path.size()-1); i++){
		 std::cout << "\t" << _path[i] << std::endl;
		 _blend_radius.push_back(1);
	       }
	        std::cout << "RobotController: Moving the robot!" << std::endl;
	       
	       if(!_sim){
	       if(!_rosComm->MoveRobotJoint(_path, _blend_radius, _speed)){
		 //_motionState = APPROACH;
		 _state_done = true;
		 std::cout << "RobotController: Could NOT move the robot!" << std::endl;
		 break;
		}
	       }else
	       {
	
		 Q_EMIT simulate(_path,QString::fromStdString(_robot_name));
		 
	       }
	    }else std::cout << "RobotController: Could not plan a valid motion!" << std::endl;
	 
	     }else {
	    std::cout << "RobotController: No inverse kinematic solution!!!" << std::endl;
	   _state_done = true;
	    _motionState = IDLE;
	    break;
	  }
	  
	  _state_done = false;
	 break;
	
      }
      case APPROACH:
      {
	// Approach the object
	 std::cout << "RobotController state = APPROACH" << std::endl;
	 _state_done = false;
	 

	break;
      }
      case GRASP:
      {
	//Grasp the object
	 std::cout << "RobotController state = GRASP" << std::endl; 
	 _state_done = false;
	 
	 
	 
	
	break;
      }
      case RETRACT:
      {
	//Retract from the object
	std::cout << "RobotController state = RETRACT" << std::endl; 
		_state_done = false;
	
	
	
	
	

	break;
      }
      case MOVE_TO_RELEASE:
      {
	// Move to the place position
	std::cout << "RobotController state = MOVE_TO_RELEASE" << std::endl; 
	_state_done = false;
	
	break;
      }
      case RELEASE:
      {
	// Release the object
	std::cout << "RobotController state = RELEASE" << std::endl;
	 _state_done = false;
	 
	 
	 
	

	break;
      }
      case HOME:
      {
	// Move to home position
	rw::math::Q _home_pose(6, -1.753, -0.314, -0.912, 4.135, 1.650, 3.042);
	std::cout << "RobotController state = HOME" << std::endl; 
	 _state_done = false;
	 
	 
	 
	
	break;
      }
      
      default:
      {
	break;
      }
    }
   }
  
     //Sleep 500msek
    QThread::msleep(100);
 }
  
}

void RobotController::stop()
{
    std::cout << "Quit RobotController thread!" << std::endl;
    _isRunning = false;
    wait();
  
}

void RobotController::motionDone()
{
  std::cout << "RobotController: Motion Done!" << std::endl;
  if(_motionState == MOVE_TO_GRASP) _motionState = APPROACH;
  else if(_motionState == APPROACH) _motionState = GRASP;
  else if(_motionState == GRASP)_motionState = RETRACT;
  else if(_motionState == RETRACT)_motionState = MOVE_TO_RELEASE;
  else if(_motionState == MOVE_TO_RELEASE) _motionState = RELEASE;
  else if(_motionState == RELEASE) _motionState = HOME;
  else if(_motionState == HOME) _motionState = IDLE;
  else _motionState = IDLE;
  
  _state_done = true;
}

void RobotController::graspObject(rw::math::Transform3D<double> pose)
{
    _object_pose = pose;
    _motionState = MOVE_TO_GRASP; 
}

bool RobotController::graspObject(ModelData& data)
{
  using namespace rwlibs::task;
  using namespace rw::math;

  _object_pose = data.getBestPose();
  
  if(!data.has_gtask()) return false;
  
  GraspTask gtask = data.getGraspTask();
  std::vector<std::pair<GraspSubTask*,GraspTarget*> > tasks = gtask.getAllTargets();
  for(size_t i = 0; i<tasks.size();i++){
    
    GraspResult::Ptr tres = tasks[i].second->getResult();
    
     if(tres->testStatus == GraspTask::Success)
     {
      //   std::cout << "\nQuality before lifting: " << tres->qualityBeforeLifting() << std::endl;
      //   std::cout << "Quality after lifting: " << tres->qualityAfterLifting << std::endl;
	// _Q_grasp = tres->gripperConfigurationGrasp();
	// Get the different transformations
	//Transform3D<double> objTtcp_app = tres->objectTtcpApproach();
	//Transform3D<double> objTtcp_grasp =tres->objectTtcpGrasp();
	//Transform3D<double> objTtcp_lift =tres->objectTtcpLift();
	//Transform3D<double> objTtar =tres->objectTtcpTarget;
     } 
  }
  
  _motionState = MOVE_TO_GRASP; 
  
  return true;
}


void RobotController::updateRobotQ(rw::math::Q q, QString robot_name)
{
    _currentQ = q;
}


}
}
}