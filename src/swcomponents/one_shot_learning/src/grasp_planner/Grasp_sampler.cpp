#include <one_shot_learning/grasp_planner/Grasp_sampler.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

namespace dti{
  namespace grasp_planning
  {
Grasp_sampler::Grasp_sampler(Workcell *workcell) : _workcell(workcell)
{
 cd = _workcell->getCollisionDetector();
}

Grasp_sampler::~Grasp_sampler()
{
  
}

void Grasp_sampler::LoadGripperInverseKin(std::string filename)
{
     invKin = new SDHInvKinSolver(filename);
  
}
void Grasp_sampler::generateTask(GripperType type, std::string objectName, int nrTarget)
{
    using namespace rwlibs::task;
    using namespace rw::common;
    using namespace rw::kinematics;
    
    std::string gripperName;
    _grippertype = type;
	
   // Make a new grasp task
   _gtasks = ownedPtr(new GraspTask());

    rwsim::dynamics::Body::Ptr body = _workcell->getDynamicWorkcell()->findBody(objectName);
    if(body.isNull()){
        RW_THROW("BODY DOES NOT EXIST: " << objectName);
    }
    
    _object =  _workcell->getWorkcell()->findObject(objectName);
    //_object->getGeometry()[0]->getGeometryData();
      if(_object.isNull()){
        RW_THROW("OBJECT DOES NOT EXIST: " << objectName);
    }

    // Make a new Surface Pose Sampler
    std::vector<rw::geometry::Geometry::Ptr> obj = _object->getGeometry();
    //std::cout << "Object list size: " << int(obj.size()) << std::endl;
    if(_object->getGeometry().size() > 0){
      _sampler = ownedPtr( new SurfacePoseSampler(obj[0]));
      _sampler->setRandomPositionEnabled(false);
      _sampler->setRandomRotationEnabled(false);
    }else
      RW_THROW("Cannot create SurfacePoseSampler with no object geometry!");


    // these should be the object transformation
    Vector3D<> pos(0, 0, 0);
    Rotation3D<> rot(1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);

    // first set up the configuration
    Vector3D<> d(0,0,-0.02);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    
    openQ = Q(1,0.0);
    closeQ = Q(1,1.0);
    if( type== PG70 ){
        openQ  = Q(1, 0.034);
        closeQ = Q(1, 0.0);
	 gripperName = "PG70";
	 //Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
        _gtasks->setTCPID("PG70.TCPPG70");
    } else if( type== PG70_SMALL){
        openQ  = Q(1, 0.01);
        closeQ = Q(1, 0.0);
        gripperName = "PG70_SMALL";
	
	 //Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	 
        _gtasks->setTCPID("PG70.TCPPG70");
    } else if( type== GS20){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        gripperName = type;
	 //Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
        _gtasks->setTCPID("TCPGS20");
    } else if( type== GS20_WIDE){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        gripperName = "GS20";
	 //Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
        _gtasks->setTCPID("TCPGS20");
    } else if( type== SDH_PAR){
        //Set open/close and min/max configuration
        openQ =  Q(7,-1.571,-1.571,1.571, -1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
	minQ = Q(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
        maxQ = Q(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);
	gripperName = "SchunkHand";
	//Find the gripper in the scene
	_gripper = _workcell->getWorkcell()->findDevice(gripperName);
	if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	
	_gripperBase = _workcell->getWorkcell()->findFrame<MovableFrame>("SchunkHand.Base");
	if(!_gripperBase) RW_THROW("Could not find MovableFrame: SchunkHand.Base");
	
	_gripperTcpFrame = _workcell->getWorkcell()->findFrame("SchunkHand.SDHTCP");
	if(!_gripperTcpFrame) RW_THROW("Could not find gripper TCP frame: SchunkHand.SDHTCP");
	
        _gtasks->setTCPID("SchunkHand.SDHTCP");
    
    } else if( type== SDH_PAR1){
      //Set open/close and min/max configuration
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
	minQ = Q(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
        maxQ = Q(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);
	 gripperName = type;
	 //Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	 
	 _gripperBase = _workcell->getWorkcell()->findFrame<MovableFrame>("SchunkHand.Base");
	if(!_gripperBase) RW_THROW("Could not find MovableFrame: SchunkHand.Base");
	
	_gripperTcpFrame = _workcell->getWorkcell()->findFrame("SchunkHand.SDHTCP1");
	if(!_gripperTcpFrame) RW_THROW("Could not find gripper TCP frame: SchunkHand.SDHTCP1");
	
        _gtasks->setTCPID("SchunkHand.SDHTCP1");
       
    } else if( type== SDH_PAR2){
      //Set open/close and min/max configuration
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
	minQ = Q(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
        maxQ = Q(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);

        gripperName = "SchunkHand";
	//Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	 
	 _gripperBase = _workcell->getWorkcell()->findFrame<MovableFrame>("SchunkHand.Base");
	if(!_gripperBase) RW_THROW("Could not find MovableFrame: SchunkHand.Base");
	
	_gripperTcpFrame = _workcell->getWorkcell()->findFrame("SchunkHand.SDHTCP1");
	if(!_gripperTcpFrame) RW_THROW("Could not find gripper TCP frame: SchunkHand.SDHTCP1");
	
	_gtasks->setTCPID("SchunkHand.SDHTCP1");


    } else if( type== SDH_PAR1_TABLE){
      //Set open/close and min/max configuration
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
	minQ = Q(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
        maxQ = Q(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);

        gripperName = "SchunkHand";
	//Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	 
	 _gripperBase = _workcell->getWorkcell()->findFrame<MovableFrame>("SchunkHand.Base");
	if(!_gripperBase) RW_THROW("Could not find MovableFrame: SchunkHand.Base");
	
	_gripperTcpFrame = _workcell->getWorkcell()->findFrame("SchunkHand.SDHTCP1");
	if(!_gripperTcpFrame) RW_THROW("Could not find gripper TCP frame: SchunkHand.SDHTCP1");
	
	_gtasks->setTCPID("SchunkHand.SDHTCP1");
	
    } else if( type== SDH_PAR2_TABLE){
      //Set open/close and min/max configuration
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
	minQ = Q(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
        maxQ = Q(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);
  
        gripperName = "SchunkHand";
	//Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	 
	 _gripperBase = _workcell->getWorkcell()->findFrame<MovableFrame>("SchunkHand.Base");
	if(!_gripperBase) RW_THROW("Could not find MovableFrame: SchunkHand.Base");
	
	_gripperTcpFrame = _workcell->getWorkcell()->findFrame("SchunkHand.SDHTCP1");
	if(!_gripperTcpFrame) RW_THROW("Could not find gripper TCP frame: SchunkHand.SDHTCP1");
	
       _gtasks->setTCPID("SchunkHand.SDHTCP1");

    } else if( type== SDH_BALL){
      //Set open/close and min/max configuration
        openQ = Q(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 1.047,0.0, 0.349,0.0, 0.349);
	minQ = Q(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
        maxQ = Q(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);

        gripperName = "SchunkHand";
	//Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	 
	 _gripperBase = _workcell->getWorkcell()->findFrame<MovableFrame>("SchunkHand.Base");
	if(!_gripperBase) RW_THROW("Could not find MovableFrame: SchunkHand.Base");
	
	_gripperTcpFrame = _workcell->getWorkcell()->findFrame("SchunkHand.SDHTCP");
	if(!_gripperTcpFrame) RW_THROW("Could not find gripper TCP frame: SchunkHand.SDHTCP");
	
	_gtasks->setTCPID("SchunkHand.SDHTCP");
	
    } else if( type== SDH_CYL){
      //Set open/close and min/max configuration
        openQ = Q(7, -1.048, 0.174, 0.0, -1.048, 0.174,-1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 0.0, 0.0, 0.349, 0.0, 0.349);
	minQ = Q(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
        maxQ = Q(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);
  
        gripperName = "SchunkHand";
	//Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
	 
	  _gripperBase = _workcell->getWorkcell()->findFrame<MovableFrame>("SchunkHand.Base");
	if(!_gripperBase) RW_THROW("Could not find MovableFrame: SchunkHand.Base");
	
	_gripperTcpFrame = _workcell->getWorkcell()->findFrame("SchunkHand.SDHTCP");
	if(!_gripperTcpFrame) RW_THROW("Could not find gripper TCP frame: SchunkHand.SDHTCP");
	
	_gtasks->setTCPID("SchunkHand.SDHTCP");
	
    } else if( type== SCUP){
      //Set open/close and min/max configuration
        openQ  = Q(1, 0.0);
        closeQ = Q(1, 1.0);
	minQ = openQ;
        maxQ = closeQ;
        _gtasks->setTCPID("EndFrame");
        //_graspSim->setAlwaysResting(true);
        gripperName = type;
	//Find the gripper in the scene
	 _gripper = _workcell->getWorkcell()->findDevice(gripperName);
	 if(!_gripper) RW_THROW("Could not find gripper: " << gripperName);
    } else {
        RW_THROW(" The gripper type is wrong! please specify a valid grippertype: (PG70, SCUP, SDH_PAR, SDH_CYL, SDH_BALL)");
    }

    _gtasks->setGripperID(gripperName);
    _gtasks->setGraspControllerID("GraspController");
    //rtask->getPropertyMap().set<std::string >("Object", objectName);

    //CartesianTask::Ptr tasks = ownedPtr( new CartesianTask());
    _gtasks->getSubTasks().resize(1);
    GraspSubTask &subtask = _gtasks->getSubTasks()[0];


    if( gripperName=="SchunkHand"){
        Q tau = Q(7, 2.0, 2.0, 10.0, 2.0, 2.0, 2.0, 2.0);
        // depending on the value of joint 2 adjust the forces
        double alpha = openQ(2);
        if(alpha<45*Deg2Rad){
            tau(3) = tau(0)/(2*cos(alpha));
            tau(5) = tau(0)/(2*cos(alpha));
        } else {
            tau(0) = std::max( 2*cos(alpha)*tau(3), 0.2);
        }
        subtask.tauMax = tau;
    }

    subtask.offset = wTe_n;
    if( type== SCUP){
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.04));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.0));
    } else if( gripperName=="GS20"){
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.0));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.04));
    } else {
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.0));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.10));
    }

    subtask.openQ = openQ;
    subtask.closeQ = closeQ;

    if( type==GS20 || type==GS20_WIDE){
        _sampler->setBoundsD(-0.02,0.02);
    } else if( type==SCUP ){
        _sampler->setBoundsD(-0.02,0.005);
    } else {
        _sampler->setBoundsD(-0.04,0.04);
    }

    // now we choose a random number in the total area
    State state = _workcell->getDynamicWorkcell()->getWorkcell()->getDefaultState();
    Transform3D<> wTo = rw::kinematics::Kinematics::worldTframe(body->getBodyFrame(), state);

    for(int i=0; i<nrTarget; i++){
        Transform3D<> target;

        target = wTo*_sampler->sample();
        subtask.targets.push_back( GraspTarget( target ) );
    }

  
}

void Grasp_sampler::Sample(int samples)
{
  using namespace rw::kinematics;
  
    // Get the defalt state
    State state = _workcell->getWorkcell()->getDefaultState();
    
    // Calculate transforms tcpTbase and objectTworld	    
    Transform3D<> tcpTbase = Kinematics::frameTframe(_gripperTcpFrame.get(), _gripperBase.get(), state);
    Transform3D<> objectTworld = Kinematics::worldTframe(_object->getBase(), state);
    
    // Set SDH open and close state 
    Q aopen(7, -5*Deg2Rad, -5*Deg2Rad ,  0, -5*Deg2Rad, -5*Deg2Rad, -5*Deg2Rad, -5*Deg2Rad);
    Q aclose(7, 20*Deg2Rad,  35*Deg2Rad ,  0,  20*Deg2Rad,  35*Deg2Rad,  20*Deg2Rad,  35*Deg2Rad);
    
    // Creating parameters
    bool collOpen;
    Transform3D<> pose, target;
    Vector3D<> approach;
    Q tau, q;
    std::vector<boost::tuple<Transform3D<>, Q, bool> > res;
    std::vector<Transform3D<> > targets;
  
  
    // For each sample
    for(int i=0; i<samples; i++){
      if (i%(samples/100)==0) std::cout << i << std::endl; //Print sample count	

	if (_grippertype == PG70) {
	  RW_THROW("Sampling for PG70 is not implemented yet!!");
	  // Do nothing yet!
	} 
	else if( _grippertype == SDH_PAR || _grippertype == SDH_PAR1 || _grippertype == SDH_PAR1_TABLE || _grippertype == SDH_PAR2 
	  || _grippertype == SDH_PAR2_TABLE || _grippertype == SDH_BALL || _grippertype == SDH_CYL)
	{
	  targets.clear();
	  for(int j=0; j<3; j++) {
	    target = _sampler->sample();
	    target.P() -= (target.R()*Vector3D<>::z())*Math::ran(0.005,0.03);
	    targets.push_back(target);
	  }
	  
	  approach = normalize(cross(targets[1].P()-targets[0].P(), targets[2].P()-targets[0].P()));
	  Vector3D<> targetAvg = (targets[0].P()+targets[1].P()+targets[2].P())/3.0;

	  bool filtered = false;
	  for(size_t k=0; k<targets.size() && !filtered; k++){
	    Vector3D<> pos = targets[k].P();
	    Vector3D<> dir = targets[k].R()*Vector3D<>::z();// z points into the surface normal direction
	    Vector3D<> yaxis = targets[k].R()*Vector3D<>::y();

	    double a = angleProj(yaxis,approach,dir);
	    targets[k] = Transform3D<>(pos,targets[k].R()*(EAA<>(0.,0.,a)).toRotation3D());

	    double ang = std::abs(angleProj(normalize(pos-targetAvg),dir,approach));
	      if(ang > 45.*Deg2Rad && ang < 135.*Deg2Rad) filtered = true;
		ang = std::abs(angleProj(normalize(pos-targetAvg),dir,normalize(cross(approach,pos-targetAvg))));
	      if(ang < 135.*Deg2Rad) filtered = true;
	  }

	  //Transform3D<> wTo = Kinematics::worldTframe(object->getBase(),workcell->getDefaultState());
	  if (/*dot(wTo*approach,Vector3D<>::z())<=0.0 && */!filtered) {
	    if(invKin){
	      try{
		  res = invKin->solve(targets, approach);
		}catch(...){
		  RW_THROW("An exception occured in solving SDH inverse minematics!!");
		}
	    }
	    
	  } else {
	    res.clear();
	  }

	  if(res.size()>0 && res[0].get<2>())
	  {
	    std::cout << "Task Found at " << i << std::endl;;
	    pose = res[0].get<0>();
	    q = res[0].get<1>();
	    //Adjust Forces (tau) & Clamp Q
	    
	    // TODO: again very specific to device
	    Q openQ  = Math::clampQ(q + aopen, minQ, maxQ);
	    Q closeQ = Math::clampQ(q + aclose, minQ, maxQ);

	    // TODO: this is SDH specific
	    tau = Q(7, 2.0, 2.0, 10.0, 2.0, 2.0, 2.0, 2.0);
	    // depending on the value of joint 2 adjust the forces
	    double alpha = openQ(2);
	    if(alpha<45*Deg2Rad){
	      tau(3) = tau(0)/(2*cos(alpha));
	      tau(5) = tau(0)/(2*cos(alpha));
	    } else {
	      tau(0) = std::max( 2*cos(alpha)*tau(3), 0.2);
	    }
	
	state = _workcell->getWorkcell()->getDefaultState();
	_gripperBase->moveTo(objectTworld*pose*tcpTbase,state);
	_gripper->setQ(openQ,state);
	
	//Filter + add Collision Check!
//	state = _workcell->getWorkcell()->getDefaultState();
//	_workcell->getWorkcell()->getMovableFrame()->moveTo(objectTworld*pose*tcpTbase,state);
//	_workcell->getWorkcell()->getGripper()->setQ(openQ,state);
	collOpen = cd->inCollision(state);
			
	      //Not in Collision
	      if(!collOpen) {
		GraspSubTask stask;
		stask.setRetract( Transform3D<>( Vector3D<>(0,0,0.1)) );
		stask.setOpenQ( openQ );
		stask.setCloseQ( closeQ );
		stask.setTauMax( tau );
		stask.setRefFrame(_object->getName()); //_workcell->getObject()->getName());
		//std::ostringstream taskID;
		//taskID << i;
		//stask.setTaskID( taskID.str() );
		stask.addTarget( pose );
		_gtasks->addSubTask( stask );
	  
		//Uncertainty Measure + Priorities
		std::cout << "Task Added at " << i << std::endl;
		
	      } 
	    
	  }
	  
	}
    }
    
//   std::cout << "=================== Samples ===================" << std::endl;
//   std::cout << "\tNumber of subtasks: " << int(_gtasks->getSubTasks().size()) << std::endl;
//   std::cout << "\tNumber of targets: " << int(_gtasks->getAllTargets().size()) << std::endl;
//   std::cout << "\tController ID: " << _gtasks->getGraspControllerID() << std::endl;
//   std::cout << "\tGripper ID: " << _gtasks->getGripperID() << std::endl;
//   std::cout << "\tTCP ID: " << _gtasks->getTCPID() << std::endl;
}

void Grasp_sampler::SaveGraspTask(std::string fileName)
{
 rwlibs::task::GraspTask::saveRWTask(_gtasks,fileName); 
}

Vector3D<> Grasp_sampler::projPlane(Vector3D<> vec, Vector3D<> normal) {
	return vec-dot(vec,normal)*vec;
}

double Grasp_sampler::angleProj(Vector3D<> v1, Vector3D<> v2, Vector3D<> normal) {
	return angle(projPlane(v1,normal),projPlane(v2,normal),normal);
}

}
}
