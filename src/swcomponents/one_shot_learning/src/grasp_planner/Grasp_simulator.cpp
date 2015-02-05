#include <one_shot_learning/grasp_planner/Grasp_simulator.hpp>

#include <rw/RobWork.hpp>
#include <QFile>
#include <QStringList>

USE_ROBWORK_NAMESPACE
using namespace robwork;

namespace dti{
  namespace grasp_planning{

Grasp_simulator::Grasp_simulator(Grasp_sampler *sampler) :  _sampler(sampler)
{
  _grasptask = _sampler->getGraspTask();
  if(!_grasptask) RW_THROW("No grasp task for simulation!!"); 
  
  _recordStatePath = false;
   _numberOfThreads = 1;
   _isRunning  = false;
   _pause_sim = false;
   _resume_sim = false;
   _sigma_a = 0;
   _sigma_p = 0;
   _save_gtask = false;
}

Grasp_simulator::~Grasp_simulator()
{
   stop();
}

void Grasp_simulator::run()
{
  _isRunning = true;
   //std::cout << "grasp simulator current Thread Id: " << int(QThread::currentThreadId()) << std::endl;
   if(SimulateGraspHypothesis()){
      FilterGrasps();
       if(SimulateGraspPertubations()){
	  calcPerturbedQuality();
	  if(_save_gtask)
	    SaveGraspTask();
       }
   }
  
  Q_EMIT finish();
}

void Grasp_simulator::PauseSimulator(){
  _pause_sim = true;
}

void Grasp_simulator::ResumeSimulator(){
  _resume_sim = true; 
}

void Grasp_simulator::stop()
{
   std::cout << "Quit Grasp_simulator thread!" << std::endl;
   _isRunning = false;
   wait();
}

bool Grasp_simulator::SimulateGraspHypothesis()
{
   using namespace rwsim::simulator;
   using namespace rwsim::dynamics;
   using namespace rwlibs::task;
   using namespace rw::trajectory;
   using namespace rwsim::loaders;
   
   if(_object_name.empty()){
     RW_THROW("No object name in grasp simulator!!");
     return false;
   }
   
   if(!_dwc) return false;
     
   // Set GraspTask types which is include in the simulation
   std::vector<GraspTask::TestStatus> taskincludefilter;
   taskincludefilter.push_back(GraspTask::Success);
   taskincludefilter.push_back(GraspTask::ObjectSlipped);
   
   rw::kinematics::State initState = _dwc->getWorkcell()->getDefaultState();
  
   //Load graspTask if needed
   if(_taskXMLFile != "")
   {
     std::cout << "loading: " << _taskXMLFile << std::endl;
     _grasptask = GraspTask::load(_taskXMLFile);
   }else if(!_grasptask && _taskXMLFile != ""){
     RW_THROW("No invalid grasp task of path to grasptask.xml!");
     return false;
   }

   std::cout << "=================== Grasp task to simulate ===================" << std::endl;
   std::cout << "\tNumber of subtasks: " << int(_grasptask->getSubTasks().size()) << std::endl;
   std::cout << "\tNumber of targets: " << int(_grasptask->getAllTargets().size()) << std::endl;
   std::cout << "\tController ID: " << _grasptask->getGraspControllerID() << std::endl;
   std::cout << "\tGripper ID: " << _grasptask->getGripperID() << std::endl;
   std::cout << "\tTCP ID: " << _grasptask->getTCPID() << std::endl;


  // temporarilly change refframe to Object change
  BOOST_FOREACH(GraspSubTask &stask, _grasptask->getSubTasks()){
    //stask.setRefFrame("object");
    stask.setRefFrame(_object_name);
    // also remove results
    BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
      target.result = NULL;     
    }
  }
  
  std::cout << "Starting simulation:" << std::endl;
    
 // RobWork::getInstance()->initialize();
  const std::vector<std::string> engines = PhysicsEngine::Factory::getEngineIDs();
  std::cout << "Engines available: " << engines.size() << std::endl;

  BOOST_FOREACH(const std::string& str, engines) {
		std::cout << str << std::endl;
}

//  graspSim->init(dwc, initState);
  graspSim->load(_grasptask);
  graspSim->startSimulation(initState);
 
  if(_recordStatePath){
     statep.push_back(rw::trajectory::TimedState(0,initState));
  }
 
  TimerUtil::sleepMs(2000);
  std::cout << graspSim->getStatDescription() << std::endl;
  for(std::size_t j=0;j<graspSim->getStat().size(); j++){ std::cout << j << "\t"; }
  std::cout<< std::endl;

  do{
    
    
    if(_pause_sim){
      graspSim->pauseSimulation();
      _pause_sim = false;
    }
    if(_resume_sim){
      graspSim->resumeSimulation();
      _resume_sim = false;
    }
    
    if(_recordStatePath)statep.push_back(rw::trajectory::TimedState(statep.size()*0.01,graspSim->getSimulator()->getState()));
     
     TimerUtil::sleepMs(100);
     std::vector<int> stat = graspSim->getStat();
     _percent_done = (double(graspSim->getNrTargetsDone())/double(graspSim->getNrTargets()))*100;
     
     std::cout << "\r";
     BOOST_FOREACH(int i, stat){ std::cout << i << "\t"; }
     std::cout << "Simulated " << _percent_done << "%";
     std::cout << std::flush;
     } while(graspSim->isRunning());
  
     //Get the result from the simulator
     _grasptask = graspSim->getResult();
     //Filter the result
     _grasptask->filterTasks( taskincludefilter );
     // store timed state
     if(_recordStatePath){
       std::cout << "\nStatepath: " << statep.size() << std::endl;
       PathLoader::storeTimedStatePath(*_dwc->getWorkcell(),statep, _record_file_path);
    }
  std::cout << "\nDone simulating!!" << std::endl;
   return true;
  
}

void Grasp_simulator::FilterGrasps()
{
  using namespace rwlibs::task;
  
  includeMap[GraspTask::Success] = true;
  includeMap[GraspTask::ObjectSlipped] = true; 
  
  std::cout << "\n================= Filtering grasps ====================" << std::endl;
  std::cout << "Number of subtasks: " << int(_grasptask->getSubTasks().size()) << std::endl;
	
  int targets = 0, totaltargets = 0, localNrTargets=0;//,nrBatches=0;
  std::vector<int> testStat(GraspTask::SizeOfStatusArray,0);
  
  if(_gtask==NULL){
    _gtask = ownedPtr( new GraspTask() );
    _gtask->setGripperID( _grasptask->getGripperID() );
    _gtask->setTCPID( _grasptask->getTCPID() );
    _gtask->setGraspControllerID( _grasptask->getGraspControllerID() );
  }
            
  //Put all subtasks into gtask
  //Sort grasps to only include grasp include in includeMap
  BOOST_FOREACH(GraspSubTask &stask, _grasptask->getSubTasks())
  {
    std::vector<GraspTarget> filteredTargets;
    BOOST_FOREACH(GraspTarget &target, stask.targets)
    {
                 if(target.result==NULL)
		      continue;
                 int teststatus = target.result->testStatus;
                 if(teststatus<0) 
		     teststatus=0;
		   
		 testStat[teststatus]++;
                 totaltargets++;
               /*if( useGraspTarget ){
                    target.pose = target.result->objectTtcpGrasp;
                 }
	       */
                    if( includeMap[teststatus] ){
                        targets++;
                        localNrTargets++;
                        filteredTargets.push_back(target);
                    }
                   
            }
            stask.targets = filteredTargets;
            if(filteredTargets.size()>0)
              _gtask->getSubTasks().push_back( stask );
        }
    
  std::cout << "Number of subtasks after filtering: " << int(_gtask->getSubTasks().size()) << std::endl;
}

bool Grasp_simulator::SimulateGraspPertubations()
{
  using namespace rwlibs::task;
  using namespace rwsim::simulator;
  using namespace rwsim::dynamics;
  
  if(_sigma_a == 0 || _sigma_p == 0) return false;
  //if there only exiat one target we dont need to simulate
  if(_gtask->getAllTargets().size() <= 0 ) return false;
  
  if(!_dwc) return false;
  
  std::cout << "Simulating grasp pertubations!!" << std::endl;
   
  rw::kinematics::State initState = _dwc->getWorkcell()->getDefaultState();
   
   //Load graspTask if needed
   if(_taskXMLFile != ""){
     std::cout << "loading: " << _taskXMLFile << std::endl;
     _gtask = GraspTask::load(_taskXMLFile);
   }else if(!_gtask && _taskXMLFile != ""){
     RW_THROW("No invalid grasp task of path to grasptask.xml!");
     return false;
   }

    // do the simulation
    //Unused: int targets = 0;
    std::vector<int> testStat(GraspTask::SizeOfStatusArray,0); 
    bool useAlignedGrasp = true;
    
    // temporarilly change refframe to Object change
    BOOST_FOREACH(GraspSubTask &stask, _gtask->getSubTasks()){
      // also remove results
      BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
	if(target.result!=NULL){
          if(useAlignedGrasp){
            if( target.result->testStatus==GraspTask::Success || target.result->testStatus==GraspTask::ObjectSlipped){
                target.pose = target.result->objectTtcpLift;
            }
          }
        }
         target.result = NULL;
      }
    }
    
    std::vector<GraspTask::Ptr> tasks;
     
   //Add pertubations 
    addPertubations(_gtask, _sigma_p, _sigma_a, _pertubations );
    int nroftarg = 6000/(_pertubations+1);
    tasks = splitTask(_gtask,nroftarg*(_pertubations+1));
    
    std::cout << "=================== Grasp task to simulate ===================" << std::endl;
    std::cout << "\tNumber of tasks: " << int(tasks.size()) << std::endl;
    std::cout << "\tNumber of subtasks: " << int(_gtask->getSubTasks().size()) << std::endl;
    std::cout << "\tNumber of targets: " << int(_gtask->getAllTargets().size()) << std::endl;
    std::cout << "\tController ID: " << _gtask->getGraspControllerID() << std::endl;
    std::cout << "\tGripper ID: " << _gtask->getGripperID() << std::endl;
    std::cout << "\tTCP ID: " << _gtask->getTCPID() << std::endl;
    
    std::cout << "\n\nStarting simulation:" << std::endl;
	
     // create GraspTaskSimulator
  //  GraspTaskSimulator::Ptr graspSim = ownedPtr( new GraspTaskSimulator(dwc, _numberOfThreads) );

    std::cout << graspSim->getStatDescription() << std::endl;
    std::cout << std::endl;

    for(std::size_t i=0;i<tasks.size();i++){
	graspSim->load(tasks[i]);
	graspSim->startSimulation(initState);
            if(_recordStatePath){
                statep.push_back(rw::trajectory::TimedState(0,initState));
            }
            
	    
            for(std::size_t j=0;j<graspSim->getStat().size(); j++){ std::cout << j << "\t"; }
            std::cout<< std::endl;
            TimerUtil::sleepMs(2000);
            do{
                if(_recordStatePath){
                    statep.push_back(rw::trajectory::TimedState(statep.size()*0.01,graspSim->getSimulator()->getState()));
                }
                TimerUtil::sleepMs(100);
                std::vector<int> stat = graspSim->getStat();
		_percent_done = ((double(graspSim->getNrTargetsDone())/double(graspSim->getNrTargets()))*100)/int(tasks.size());
		
                std::cout << "\r";
                BOOST_FOREACH(int i, stat){ std::cout << i << "\t"; }
                 std::cout << "Simulated " << _percent_done << "%";
                std::cout << std::flush;
            } while(graspSim->isRunning());

            _gtask = graspSim->getResult();
          
            // store timed state
            if(_recordStatePath){
                std::cout << "Statepath: " << statep.size() << std::endl;
		// std::stringstream ss; ss << _record_file_path; ss << "/statepath_with_pertubations.rwplay";
                PathLoader::storeTimedStatePath(*_dwc->getWorkcell(),statep, _record_file_path);
            }
    }
 
    std::cout << "Done" << std::endl;
  return true;
}


void Grasp_simulator::addPertubations(GraspTask::Ptr grasptask, double sigma_p, double sigma_a, int pertubationsPerTarget)
{
    //double sigma_p = 0.005;
    //double sigma_a = 15*Deg2Rad;
    //int pertubationsPerTarget = 100;
    //Unused: int count = 0;
    
    // temporarilly change refframe to Object change
    BOOST_FOREACH(GraspSubTask &stask, grasptask->getSubTasks()){
        std::vector<GraspTarget> ntargets;
        // also remove results

        BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
            ntargets.push_back(target.pose);
            for(int i=0;i<pertubationsPerTarget;i++){
                Vector3D<> pos(Math::ranNormalDist(0,sigma_p), Math::ranNormalDist(0,sigma_p), Math::ranNormalDist(0,sigma_p));
                // we can do this only for small sigmas (approximation)
                EAA<> rot(Math::ranNormalDist(0,sigma_a), Math::ranNormalDist(0,sigma_a), Math::ranNormalDist(0,sigma_a));

                // TODO: we should truncate at 2*sigma sooo

                Transform3D<> ntarget = target.pose*Transform3D<>(pos, rot);
                ntargets.push_back(ntarget);
            }
        }
        stask.getTargets() = ntargets;
    }
}

bool Grasp_simulator::calcPerturbedQuality()
{
    using namespace rwlibs::task;
    
    std::cout << "Computing pertubated Quality measure!!" << std::endl;
    
   if(_gtask->getSubTasks().size() > 0){
     
     unsigned int count = 0, succCnt=0, failCnt=0;
     std::vector<std::pair<GraspSubTask*,GraspTarget*> > tasks = _gtask->getAllTargets();
     
     GraspTask::Ptr ngtask = _gtask->clone();
     
     for(size_t i = 0; i<tasks.size();i++){
        // save the target that the quality should be calculated for
        if(count==0){
            GraspSubTask nstask = tasks[i].first->clone();
            ngtask->addSubTask( nstask );
            ngtask->getSubTasks().back().addTarget( *tasks[i].second );
            count++;
            continue;
        }

        GraspResult::Ptr tres = tasks[i].second->getResult();
        if(tres->testStatus == GraspTask::Success || tres->testStatus == GraspTask::ObjectSlipped){
            succCnt++;
        } else {
            failCnt++;
        }

        if(count==_pertubations){
            // set the quality of the target
            GraspResult::Ptr result = ngtask->getSubTasks().back().getTargets().back().getResult();

            double successProbability = ((double)succCnt)/((double)(succCnt+failCnt));
	    std::cout << "successProbability: " << successProbability << " ";
            result->qualityAfterLifting = Q(1, successProbability );
            succCnt = 0;
            failCnt = 0;
            count = 0;
        } else {
            count++;
        }
    }
    _gtask = ngtask;
    return true;
   }else{
    return false;
   }
}

std::vector<GraspTask::Ptr> Grasp_simulator::splitTask(GraspTask::Ptr grasptask, int split)
{
    int count = 0;
    std::vector<GraspTask::Ptr> gtasks = std::vector<GraspTask::Ptr>(1, grasptask->clone() );
    BOOST_FOREACH(GraspSubTask &stask, grasptask->getSubTasks()){
        // also remove results
        GraspSubTask nstask = stask.clone();
        gtasks.back()->addSubTask( nstask );

        BOOST_FOREACH(GraspTarget &target, stask.getTargets() ){
            gtasks.back()->getSubTasks().back().addTarget( target );
            count++;
            if(count>=split){
                count=0;
                // create new grasp task
                gtasks.push_back( grasptask->clone() );
                GraspSubTask nstask = stask.clone();
                gtasks.back()->addSubTask( nstask );
            }
        }
    }
    return gtasks;
}

int Grasp_simulator::getPercentDone(){
  QMutexLocker locker(&_mutexPercentDone);
 return _percent_done; 
}

void Grasp_simulator::SaveGraspTask(){
  using namespace rwlibs::task;
 
  //Check if the file already exists
  QString file_name = QString::fromStdString(_filename);
  QFile file(file_name);
  if(file.exists()){
    QStringList l = file_name.split(".");
    QString path = l[0];
    
    //find next number to store
    for(int i = 0; i < 9; i++){
      QString nr = QString::number(i);
     if(!path.endsWith(nr)){
    //  path.replace()
     } 
    }
  }
  
  // save the result
  std::cout << "Saving Grasp task to: " << _filename << std::endl;
  
  if(_format==RWTASK){
    GraspTask::saveRWTask(_gtask, _filename);
  } else if(_format== UIBK){
    GraspTask::saveUIBK(_gtask, _filename);
  } else if(_format==Text){
    GraspTask::saveText(_gtask, _filename);
  }
    std::cout << "Done Saving" << std::endl;
}

void Grasp_simulator::setSavePath(std::string& filename, FileFormat format){
  _filename = filename;
  _format = format;
  _save_gtask = true;
  
}

void Grasp_simulator::setObjectName(std::string object_name){
  _object_name = object_name;
  
}

void Grasp_simulator::setTaskXMLFilePath(std::string taskXMLFile){
  _taskXMLFile = taskXMLFile; 
}

void Grasp_simulator::setPertubationData(double sigma_a, double sigma_p,  unsigned int pertubationsPerTarget){

  _sigma_a = sigma_a;
  _sigma_p = sigma_p;
  _pertubations = pertubationsPerTarget;
  
}

void Grasp_simulator::setWorkcell(dti::grasp_planning::Workcell* workcell){
  _workcell = workcell;
   // load dynamic workcell
   _dwc = _workcell->getDynamicWorkcell();
   if(!_dwc)  RW_THROW("No dynamic simulation!!"); 
   
    // create GraspTaskSimulator
   graspSim = ownedPtr( new rwsim::simulator::GraspTaskSimulator(_dwc, _numberOfThreads) );
}


  }
}