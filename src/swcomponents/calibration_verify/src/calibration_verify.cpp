#include "calibration_verify.hpp"
#include "calibration_verify/stereo_chessboard_detector.h"
#include "calibration_verify/kinect_chessboard_detector.h"


moverobot::moverobot() :RobWorkStudioPlugin("Click_to_Move", QIcon(":/pa_icon.png"))
{
  cout << "Let's start calibration verify program.\n";
  setupUi(this);

  rw::models::WorkCell::Ptr WorkCell;
  
  try 
  {	
    stringstream xmlPath;
    xmlPath << ros::package::getPath("marvin") << "/scene/Marvin_calibration_verify_ur1.model.wc.xml";
    cout<<"Scene Address = "<<xmlPath.str()<<"\n";
    WorkCell = rw::loaders::WorkCellFactory::load(xmlPath.str());
  } 
  catch (...) 
  {
    std::cerr << "Problem in loading scene!" << std::endl;
    return;
  }
  _WorkCell = WorkCell;
  _useIntrinsicCalib=false;
  _move_Robot=false;
  _workcellCalibration = NULL;
  if (_WorkCell->getCalibrationFilename() != "") {
      _workcellCalibration = rwlibs::calibration::XmlCalibrationLoader::load(_WorkCell, _WorkCell->getFilePath() + _WorkCell->getCalibrationFilename());

  }
  else{
      std::cerr << "No workcell calibraton loaded!" << std::endl;
  }
}

moverobot::~moverobot()
{

}

void moverobot::open(rw::models::WorkCell* workcell)
{
  _WorkCell = getRobWorkStudio()->getWorkcell();
  cout << "Successful open the scene.\n";
}

void moverobot::close()
{

}

void moverobot::initialize()
{
  cout << "Initializing...\n";
  getRobWorkStudio()-> stateChangedEvent().add(boost::bind(&moverobot::stateChangedListener, this, _1), this);
  getRobWorkStudio()-> setWorkcell(_WorkCell);
  _state = _WorkCell-> getDefaultState();
  _deviceUR1 = _WorkCell-> findDevice("UR1");
  work_dir = ros::package::getPath("calibration_verify");
  
//   frame_object = _WorkCell-> findFrame<MovableFrame>("_Object");
//   frame_box = _WorkCell->findFrame<MovableFrame>("_Cube");
  State tempState = _state;
  
  //Ros initialization
  char** argv = NULL;
  int argc = 0;
  ros::init(argc, argv, "calibration_verify_node");
  
  _nodeHnd = ownedPtr(new ros::NodeHandle("~"));
  
  //Ros subscriber
  _poseUR1Subscriber = _nodeHnd->subscribe("/UR1/RobotState", 10, &moverobot::handleUR1State, this);
  
  //Ros services
  _clientUR1 = _nodeHnd->serviceClient<caros_control_msgs::SerialDeviceMovePTP>("/UR1/movePTP");
  _clientChessboard = _nodeHnd->serviceClient<calibration_verify::stereo_chessboard_detector>("/stereo_chessboard_detector/getcorner");
  //_clientChessboard = _nodeHnd->serviceClient<calibration_verify::kinect_chessboard_detector>("/kinect_chessboard_detector/getcorner");
  
  connect(Button_plus, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(Button_reduce, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(movehome, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(grab_scene, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(left_top_1, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(right_top_2, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(left_bottom_3, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(right_bottom_4, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(quit, SIGNAL(clicked()), this, SLOT(clickEvent()));
  connect(robot_intrinsic, SIGNAL(clicked()), this, SLOT(clickEvent()));
    connect(move_Robot, SIGNAL(clicked()), this, SLOT(clickEvent()));
  
  //record current location of calibration tool
  retreat_dist = 0.05;  //TDOD change the distance from 0.1 to 0.05
  location = 0;
  rw::kinematics::Frame* frame_tip = _WorkCell->findFrame("TIP");
  Transform3D<double> temp_tcpTtip = Kinematics::frameTframe(_deviceUR1->getEnd(), frame_tip, _state);
  tcpTtip = temp_tcpTtip;
  
  //start updating;
  _UR1StateUpdate = true;
  
  //end or not
  _end = false;
  
  //Calibration from robotbase to BB1, T2 is identity.
  /****** bumblebee right****************
  T1 = Transform3D<double>(Vector3D<double>(0.643406, 0.256284, 0.726622),Rotation3D<double>(-0.877336, 0.287081, -0.384534, 0.479732, 0.504976, -0.717535, -0.0118106, -0.813992, -0.580756));  //Good bumRight
  T2 = Transform3D<double>(Vector3D<double>(0, 0, 0), Rotation3D<>().identity());
  /****** kinect left****************/
  // T1 = Transform3D<double>(Vector3D<double>(0.533255, -1.36882, 0.83815), RPY<double>(27.98*Deg2Rad,-0.557*Deg2Rad,-124.44*Deg2Rad).toRotation3D());
  // T2 = Transform3D<double>(Vector3D<double>(0.0653529, 0.0670801, 0.0722794), Rotation3D<double>(0.999579,-0.0100488,-0.0272013,0.00931531,0.999593,-0.02696,0.0274612,0.0266953,0.999266));
  // bumblebee right****************
//  T1 = Transform3D<double>(Vector3D<double>(0.648155, 0.263888, 0.722187),Rotation3D<double>(-0.876935, 0.283033, -0.38843, 0.480476, 0.497164, -0.722475, -0.0113715, -0.820194, -0.571972));  //Good bumRight
//T1 =Transform3D<double>(Vector3D<double>(-0.634433, -1.3059, 0.73826), Rotation3D<double>(0.883452, -0.295727, 0.363399, -0.467604, -0.507978, 0.723398, -0.0293298, -0.809014, -0.587058)); // bumRigt
//  T2 = Transform3D<double>(Vector3D<double>(0, 0, 0), Rotation3D<>().identity());
  //kinect_right
  //T1 = Transform3D<double>(Vector3D<double>(0.546333, 0.264644, 0.649363), Rotation3D<double>(-0.881181, 0.303028, -0.362896, 0.471519, 0.507276, -0.721347, -0.0344996, -0.806749, -0.589886));
  //T2 = Transform3D<double>(Vector3D<double>(0, 0, 0), Rotation3D<double>().identity());
  rw::kinematics::Frame* camera = _WorkCell->findFrame("BB-1-Stereo");
  Transform3D<double> wTcamera = camera->getTransform(_state);
  std::cout<< std::endl << "wTcamera: "<< wTcamera << std::endl;
  std::cout<< std::endl << "bTcamera: "<< inverse( Kinematics::worldTframe(_deviceUR1->getBase(),_state)) * wTcamera << std::endl;
  T1 = inverse( Kinematics::worldTframe(_deviceUR1->getBase(),_state)) * wTcamera;
  //T1 = Transform3D<double>( Vector3D<double>( 0.529026,-1.36902, 0.73178 ), Rotation3D<double>( 0.885487, 0.267705, -0.379799,  0.464489, -0.487471, 0.739339, 0.0127838, -0.831087, -0.555995 ) );
  std::cout << "T1: "<< T1 << std::endl;
  T2 = Transform3D<double>(Vector3D<double>(0, 0, 0), Rotation3D<>().identity());
  /*********************************************************************new config only for peg******************/
  Q_home = Q(6, -92.465*Deg2Rad, -100.062*Deg2Rad, -48.701*Deg2Rad, -165.203*Deg2Rad, -15.560*Deg2Rad, -163.5*Deg2Rad); //TODO change a new home Q
  //Q_home = Q(6, -92.465*Deg2Rad, -100.062*Deg2Rad, -48.701*Deg2Rad, -119.681*Deg2Rad, 0.0*Deg2Rad, 57.185*Deg2Rad);
  
  _timer1 = new QTimer(this);
  connect(_timer1, SIGNAL(timeout()), this, SLOT(_updateScene()));
  _timer1-> start(10);
  
  //Threads
  _drawThread = new boost::thread(boost::bind(&moverobot::drawLoop, this));
}

void moverobot::drawLoop()
{
  while(!_end) {
    ros::spinOnce();
    _upUR();
  }
}

void moverobot::_upUR()
{
  if((_UR1State.size()==6)&&(_poseUR1Subscriber.getNumPublishers() != 0)) {
    if(_UR1StateUpdate == true) {
      _deviceUR1->setQ(_UR1State, _state);
      //cout << "_upUR loop!!!\n";
    }
  }
}

void moverobot::stateChangedListener(const rw::kinematics::State& state)
{
  _state = state;
}

void moverobot::handleUR1State(const caros_control_msgs::RobotState::ConstPtr& state)
{
  _UR1State = caros::toRw(state->q);
  _UR1movingTopic = state->isMoving;
  //cout << "current robot Q: " << _UR1State << "\n";
}

void moverobot::_updateScene()
{
  ros::spinOnce();
  getRobWorkStudio()->setState( _state );
}

void moverobot::clickEvent()
{
  QObject *obj = sender();
  if(obj == Button_plus) {
    rw::math::Q current_Q = _deviceUR1-> getQ(_state);
    rw::math::Q delta_Q = rw::math::Q(6, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01);
    rw::math::Q target_Q_1 = current_Q + delta_Q;
    caros_control_msgs::SerialDeviceMovePTP srv;
    srv.request.q_targets.push_back( caros::toRos(target_Q_1) );
    srv.request.blends.push_back(20.0f);
    srv.request.speeds.push_back(20.0f);
    
    bool result1 = _clientUR1.call(srv);
    cout << "++ Move the robot UR1 " << result1 << "\n";
  }
    if(obj == Button_reduce) {
    rw::math::Q current_Q = _deviceUR1-> getQ(_state);
    rw::math::Q delta_Q = rw::math::Q(6, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01);
    rw::math::Q target_Q_2 = current_Q - delta_Q;
    caros_control_msgs::SerialDeviceMovePTP srv;
    srv.request.q_targets.push_back( caros::toRos(target_Q_2) );
    srv.request.blends.push_back(20.0f);
    srv.request.speeds.push_back(20.0f);
    
    bool result2 = _clientUR1.call(srv);
    cout << "-- Move the robot UR1" << result2 << "\n";
  }
  if(obj == movehome) {
      cout<<"Move Home\n";
      rw::trajectory::QPath res00 = _moveRobotQ(Q_home, _state);
      _moveRobot(res00);
      location = 0;
  }
  if(obj == grab_scene) {
      cout<<"Grab stereo picture and compute position\n";
      Grab_scene();
  }
  if(obj == move_Robot) {

      _move_Robot=move_Robot->isChecked();
      std::cout<<"Toggle moveRobot : "<<_move_Robot<<std::endl;
  }
  if(obj == left_top_1) {
      cout<<"I will do something later\n";
      bottonCB(1);
  }
  if(obj == right_top_2) {
      cout<<"I will do something later\n";
      bottonCB(2);
  }
  if(obj == left_bottom_3) {
      cout<<"I will do something later\n";
      bottonCB(3);
  }
  if(obj == right_bottom_4) {
      cout<<"I will do something later\n";
      bottonCB(4);
  }
  if(obj == quit ){
    cout<< "You clicked quit button, but I am still running :-) .\n";
    cout<<"Current UR1 Q:\n"<<_UR1State<<"\n";
  }
  if(obj == robot_intrinsic ){
    if(robot_intrinsic->isChecked()) _useIntrinsicCalib = true;
    else _useIntrinsicCalib = false;
    cout<< "_useIntrinsicCalib is :"<<_useIntrinsicCalib<< endl;
    if ( _useIntrinsicCalib == true && _workcellCalibration != NULL) {
        _workcellCalibration->apply();
        rw::kinematics::Frame* camera = _WorkCell->findFrame("BB-1-Stereo");
        Transform3D<double> wTcamera = camera->getTransform(_state);
        std::cout<< std::endl << "wTcamera: "<< wTcamera << std::endl;
        std::cout<< std::endl << "bTcamera: "<< inverse( Kinematics::worldTframe(_deviceUR1->getBase(),_state)) * wTcamera << std::endl;
        T1 = inverse( Kinematics::worldTframe(_deviceUR1->getBase(),_state)) * wTcamera;
        //T1 = Transform3D<double>( Vector3D<double>( 0.529026,-1.36902, 0.73178 ), Rotation3D<double>( 0.885487, 0.267705, -0.379799,  0.464489, -0.487471, 0.739339, 0.0127838, -0.831087, -0.555995 ) );
        std::cout << "T1: "<< T1 << std::endl;
        T2 = Transform3D<double>(Vector3D<double>(0, 0, 0), Rotation3D<>().identity());
        camera = _WorkCell->findFrame("kinect-1"); //kinect_right
        Transform3D<double> wTcamera_right = camera->getTransform(_state);
        rw::kinematics::Frame* camera_left = _WorkCell->findFrame("kinect-2");//kinect_left
        Transform3D<double> wTcamera_left = camera_left->getTransform(_state);
        Transform3D<double> leftTright=Kinematics::frameTframe(camera_left,camera,_state);
         std::cout << "rightTleft: "<< leftTright << std::endl;
    }
    else if(_useIntrinsicCalib == false &&_workcellCalibration != NULL){
        _workcellCalibration->revert();
        rw::kinematics::Frame* camera = _WorkCell->findFrame("BB-1-Stereo");
        Transform3D<double> wTcamera = camera->getTransform(_state);
        std::cout<< std::endl << "wTcamera: "<< wTcamera << std::endl;
        std::cout<< std::endl << "bTcamera: "<< inverse( Kinematics::worldTframe(_deviceUR1->getBase(),_state)) * wTcamera << std::endl;
        T1 = inverse( Kinematics::worldTframe(_deviceUR1->getBase(),_state)) * wTcamera;
        //T1 = Transform3D<double>( Vector3D<double>( 0.529026,-1.36902, 0.73178 ), Rotation3D<double>( 0.885487, 0.267705, -0.379799,  0.464489, -0.487471, 0.739339, 0.0127838, -0.831087, -0.555995 ) );
        std::cout << "T1: "<< T1 << std::endl;
        T2 = Transform3D<double>(Vector3D<double>(0, 0, 0), Rotation3D<>().identity());
    }

  }
}

void moverobot::bottonCB(int flag)
{

    if(0 == location) {
	cout<<"Move Home\n";
	rw::trajectory::QPath res00 = _moveRobotQ(Q_home, _state);
    if(((signed int)res00.size())>0)_deviceUR1->setQ(res00[res00.size()-1],_state);
    _updateScene();
    if(_move_Robot)_moveRobot(res00);
    }
    if(1 == location) {
	cout<<"Retreat from Left top corner\n";
	rw::trajectory::QPath res01 = _moveRobotT(left_top_retreat*inverse(tcpTtip), _state);
    if(((signed int)res01.size())>0)_deviceUR1->setQ(res01[res01.size()-1],_state);
    _updateScene();
    if(_move_Robot)_moveRobot(res01);
    }
    if(2 == location) {
	cout<<"Retreat from Right top corner\n";
	rw::trajectory::QPath res02 = _moveRobotT(right_top_retreat*inverse(tcpTtip), _state);
    if(((signed int)res02.size())>0)_deviceUR1->setQ(res02[res02.size()-1],_state);
    _updateScene();
    if(_move_Robot) _moveRobot(res02);
    }
    if(3 == location) {
	cout<<"Retreat from Left bottom corner\n";
	rw::trajectory::QPath res03 = _moveRobotT(left_bottom_retreat*inverse(tcpTtip), _state);
    if(((signed int)res03.size())>0)_deviceUR1->setQ(res03[res03.size()-1],_state);
    _updateScene();
    if(_move_Robot) _moveRobot(res03);
    }
    if(4 == location) {
	cout<<"Retreat from Right bottom corner\n";
	rw::trajectory::QPath res04 = _moveRobotT(right_bottom_retreat*inverse(tcpTtip), _state);
    if(((signed int)res04.size())>0)_deviceUR1->setQ(res04[res04.size()-1],_state);
    _updateScene();
    if(_move_Robot)_moveRobot(res04);
    }
    if(1 == flag) {
	cout<<"Move Left top corner\n";
	rw::trajectory::QPath res05 = _moveRobotT(left_top_retreat*inverse(tcpTtip), _state);
    if(_move_Robot)_moveRobot(res05);
     if(((signed int)res05.size())>0)_deviceUR1->setQ(res05[res05.size()-1],_state);
    _updateScene();
	rw::trajectory::QPath res06 = _moveRobotT(left_top*inverse(tcpTtip), _state);
    if(((signed int)res06.size())>0)_deviceUR1->setQ(res06[res06.size()-1],_state);
    _updateScene();
    if(_move_Robot)_moveRobot(res06);
	location = 1;
    }
    if(2 == flag) {
	cout<<"Move Right top corner\n";
	rw::trajectory::QPath res07 = _moveRobotT(right_top_retreat*inverse(tcpTtip), _state);
    if(_move_Robot)_moveRobot(res07);
	rw::trajectory::QPath res08 = _moveRobotT(right_top*inverse(tcpTtip), _state);
    if(((signed int)res08.size())>0)_deviceUR1->setQ(res08[res08.size()-1],_state);
    _updateScene();
    if(_move_Robot)_moveRobot(res08);
	location = 2;
    }
    if(3 == flag) {
	cout<<"Move Left bottom corner\n";
	rw::trajectory::QPath res09 = _moveRobotT(left_bottom_retreat*inverse(tcpTtip), _state);
    if(_move_Robot)_moveRobot(res09);
	rw::trajectory::QPath res10 = _moveRobotT(left_bottom*inverse(tcpTtip), _state);
    if(((signed int)res10.size())>0)_deviceUR1->setQ(res10[res10.size()-1],_state);
    _updateScene();
    if(_move_Robot) _moveRobot(res10);
	location = 3;
    }
    if(4 == flag) {
	cout<<"Move Right bottom corner\n";
	rw::trajectory::QPath res11 = _moveRobotT(right_bottom_retreat*inverse(tcpTtip), _state);
    if(_move_Robot)_moveRobot(res11);
	rw::trajectory::QPath res12 = _moveRobotT(right_bottom*inverse(tcpTtip), _state);
    if(((signed int)res12.size())>0)_deviceUR1->setQ(res12[res12.size()-1],_state);
    _updateScene();
    if(_move_Robot)_moveRobot(res12);
	location = 4;
    }
}


void moverobot::Grab_scene()
{
    calibration_verify::stereo_chessboard_detector srv;
    //calibration_verify::kinect_chessboard_detector srv;
    srv.request.width = 9;
    srv.request.height = 6;
    if(!_clientChessboard.call(srv)) {
	std::cerr << "Fail to call chessboard detection service.\n";
    }
    
   
   Vector3D<double> temp_left_top_P(srv.response.left_top.at(0)/1000,
				    srv.response.left_top.at(1)/1000,
				    srv.response.left_top.at(2)/1000);
   Vector3D<double> temp_right_top_P(srv.response.right_top.at(0)/1000,
				     srv.response.right_top.at(1)/1000,
				     srv.response.right_top.at(2)/1000);
   Vector3D<double> temp_left_bottom_P(srv.response.left_bottom.at(0)/1000,
				       srv.response.left_bottom.at(1)/1000,
				       srv.response.left_bottom.at(2)/1000);
   Vector3D<double> temp_right_bottom_P(srv.response.right_bottom.at(0)/1000,
					srv.response.right_bottom.at(1)/1000,
					srv.response.right_bottom.at(2)/1000);
   
   Transform3D<double> A(temp_left_top_P, Rotation3D<double>().identity());
   Transform3D<double> B(temp_right_top_P, Rotation3D<double>().identity());
   Transform3D<double> C(temp_left_bottom_P, Rotation3D<double>().identity());
   Transform3D<double> D(temp_right_bottom_P, Rotation3D<double>().identity());
   
   A = ToWorld(A);
   B = ToWorld(B);
   C = ToWorld(C);
   D = ToWorld(D);
   
   //Calculate norm vector of the chessboard
//    Vector3D<double> AC = temp_left_bottom_P - temp_left_top_P;
//    Vector3D<double> AB = temp_right_top_P - temp_left_top_P;
   Vector3D<double> AC = C.P() - A.P();
   Vector3D<double> AB = B.P() - A.P();
   
   Vector3D<double> zaix = cross<double>(AB,AC); //TODO change z axis direction, point to table
   zaix = normalize<double>(zaix);
   Vector3D<double> yaix(0.0, 1.0, - (zaix[1])/(zaix[2]));
   yaix = normalize<double>(yaix);
   Vector3D<double> xaix = cross<double>(yaix,zaix);
   Rotation3D<double> normal_rota(xaix, yaix, zaix);
   
   Rotation3D<double> rotaZ(cos(-90*Deg2Rad),-sin(-90*Deg2Rad),0,sin(-90*Deg2Rad),cos(-90*Deg2Rad),0, 0, 0,1); 
   normal_rota = normal_rota*rotaZ; //TODO rotate Z get solve solution
   
//    Transform3D<double> temp_left_top(temp_left_top_P, normal_rota);
//    Transform3D<double> temp_right_top(temp_right_top_P, normal_rota);
//    Transform3D<double> temp_left_bottom(temp_left_bottom_P, normal_rota);
//    Transform3D<double> temp_right_bottom(temp_right_bottom_P, normal_rota);
   Transform3D<double> temp_left_top(A.P(), normal_rota);
   Transform3D<double> temp_right_top(B.P(), normal_rota);
   Transform3D<double> temp_left_bottom(C.P(), normal_rota);
   Transform3D<double> temp_right_bottom(D.P(), normal_rota);
   
   Transform3D<double> temp_left_top_retreat((temp_left_top.P() - temp_left_top.R() * Vector3D<>::z() * retreat_dist),temp_left_top.R());
   Transform3D<double> temp_right_top_retreat((temp_right_top.P() - temp_right_top.R() * Vector3D<>::z() * retreat_dist),temp_right_top.R());
   Transform3D<double> temp_left_bottom_retreat((temp_left_bottom.P() - temp_left_bottom.R() * Vector3D<>::z() * retreat_dist),temp_left_bottom.R());
   Transform3D<double> temp_right_bottom_retreat((temp_right_bottom.P() - temp_right_bottom.R() * Vector3D<>::z() * retreat_dist),temp_right_bottom.R());
   
//    left_top = ToWorld(temp_left_top);
//    right_top = ToWorld(temp_right_top);
//    left_bottom = ToWorld(temp_left_bottom);
//    right_bottom = ToWorld(temp_right_bottom);
//    
//    left_top_retreat = ToWorld(temp_left_top_retreat);
//    right_top_retreat = ToWorld(temp_right_top_retreat);
//    left_bottom_retreat = ToWorld(temp_left_bottom_retreat);
//    right_bottom_retreat = ToWorld(temp_right_bottom_retreat);
   
   left_top = temp_left_top;
   right_top = temp_right_top;
   left_bottom = temp_left_bottom;
   right_bottom = temp_right_bottom;
   
   left_top_retreat = temp_left_top_retreat;
   right_top_retreat = temp_right_top_retreat;
   left_bottom_retreat = temp_left_bottom_retreat;
   right_bottom_retreat = temp_right_bottom_retreat;
   
   cout<<"Grab Scene done!"<<"\n";
}

Transform3D< double > moverobot::ToWorld(Transform3D< double >& T)
{
  State tempState = _state;
  Transform3D<double> world2robotbase = Kinematics::worldTframe(_deviceUR1->getBase(),tempState);
  //Transfer Transformation in camera csys to world csys
  Transform3D<double> worldT = world2robotbase*T1*T2*T;
  return worldT;
}



vector< Q > moverobot::_IKSolver(const Transform3D< double > pose, State actualState)
{
  cout<<"_IKSolver()\n";
  rw::models::Device::Ptr _robot;
  std::vector<Q> solutions;
  std::vector<Q> finalsolutions;
  
  State tempState(actualState);
  _robot = _deviceUR1;
	
	  cout<<"1...\n";
  rw::trajectory::QPath res;
  rw::proximity::CollisionDetector::Ptr _detector = ownedPtr(new rw::proximity::CollisionDetector(_WorkCell,rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
  if (_detector->inCollision(actualState))
  {
    std::cout<<"collision at the start\n";
  }
    cout<<"2...\n";
  State invState = actualState;
  {
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(_detector,_robot,invState);
    cout<<"3...\n";
    rw::invkin::InvKinSolver::Ptr _iksolver = ownedPtr(new ClosedFormURSolver(_robot,invState));
        cout<<"4...\n";
    //if a transform is relative to WORLD frame, it has to be multiply by inverse transform from WORLD frame to robot.base frame; 
    std::vector<Q> solutionsSolver = _iksolver->solve(inverse(Kinematics::worldTframe(_robot->getBase(),invState))*pose,invState);
    //std::vector<Q> solutionsSolver = _iksolver->solve(pose,invState);
    cout<<"5...\n";
    std::cout<<"size of solutionsSolver = "<<solutionsSolver.size()<<"\n";
    if (solutionsSolver.size() <= 0)
    {
      std::cout<<"no solutions\n";
      return solutions;
    }
    Q curQ = _robot->getQ(invState);
    while(solutionsSolver.size() > 0) 
    {
      std::vector<Q>::iterator bestSolution;
      double bestDist = -1;
      for (std::vector<Q>::iterator it = solutionsSolver.begin(); it != solutionsSolver.end(); it++) 
      {
	State diffState = invState;
	_robot->setQ(*it,diffState);
	double dist = (*it-curQ).norm2();
	if (bestDist == -1) 
	{
	  bestSolution = it;
	  bestDist = dist;
	} 
	else if (dist < bestDist) 
	{
	  bestSolution = it;
	  bestDist = dist;
	}
      }
      if (!constraint.inCollision(*bestSolution)) 
      {
	std::cout<<"found solution\n";
	solutions.push_back(*bestSolution);
      }
      solutionsSolver.erase(bestSolution);
    }
  }
  
  for(unsigned int i=0; i<solutions.size(); i++) {
      State othertempstate(actualState);
      _robot->setQ(solutions.at(i), othertempstate);
      if(_detector->inCollision(othertempstate)) {
      std::cout<<"Find one collision solution, I will delete it.\n";
	  continue;
      }
      finalsolutions.push_back(solutions.at(i));
  }
  
  std::cout<<"size of final solutions = "<<finalsolutions.size()<<"\n";
  
  if (finalsolutions.size() <= 0) 
  {
    std::cout<<"inverse kinematic fail\n";
    return finalsolutions;
  }
  return finalsolutions;
}


rw::trajectory::QPath moverobot::_moveRobotT(const Transform3D< double >& T, State actualState)
{
    cout<< "Try to find a path with given T.\n";
    //Planning and moving (approach)
    std::vector<Q> solutionsApproach = _IKSolver(T, _state);
    rw::trajectory::QPath res;
    Q ApproachConfig;
    bool status = false;
    
    for(unsigned int i=0; i<solutionsApproach.size(); i++) {
	res = _moveRobotQ(solutionsApproach.at(i), _state);
	
	if(res.size()) {
	    status = true;
	    break; //select only one valid solution
	}
    }

    if(!status) {
	std::cout<<"No solutions\n";
	return res;
    }
    return res;
}


rw::trajectory::QPath moverobot::_moveRobotQ(const Q robot, State actualState)
{
  cout<< "Try to find a path with given Q.\n";
  //rw::models::Device::Ptr _robot;
  rw::trajectory::QPath res;
  rw::proximity::CollisionDetector::Ptr _detector = ownedPtr(new rw::proximity::CollisionDetector(_WorkCell,rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
  State tempState(actualState);
  if (_detector->inCollision(actualState))
  {
    std::cout<<"collision at the start\n";
    return res;
  }
  if (!_detector->inCollision(tempState)) 
  {
    tempState = actualState;
    
    rw::trajectory::QPath path;
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(_detector,_deviceUR1,actualState);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_deviceUR1),constraint.getQConstraintPtr());
    rw::math::Metric<rw::math::Q>::Ptr _metric = MetricFactory::makeEuclidean<Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTQToQPlanner::makeConnect(constraint,sampler,_metric,0.1);

    if (planner->query(_deviceUR1->getQ(tempState),robot,path,10.)) 
    {
      rwlibs::pathoptimization::PathLengthOptimizer optimizer(constraint, _metric);
      path = optimizer.pathPruning(path);
      if (path.size() > 2)
      {
	path = optimizer.partialShortCut(path,10,0.1,0.25);
      }
      path = optimizer.pathPruning(path);
      res.clear();
      for (unsigned int i = 0; i < path.size(); i++) 
      {
	res.push_back(path[i]);
      }
      std::cout<<"succeed in finding a path\n";
    }
  }
  //cout<<"path size = "<<res.size()<<"\n";
  return res;
}

void moverobot::_moveRobot(const rw::trajectory::QPath res)
{
  for(unsigned int i = 0;i<res.size();i++) {
    //cout<<"Q = "<<res.at(i)<<"\n";
    caros_control_msgs::SerialDeviceMovePTP srv;
    srv.request.q_targets.resize(1);
    srv.request.blends.resize(1);
    srv.request.speeds.resize(1);
    srv.request.q_targets[0] = caros::toRos(res[i]);
    srv.request.blends[0] = 20.0f;
    srv.request.speeds[0] = 20.0f;
    _clientUR1.call(srv);
    _UR1StateUpdate = true;
    while(!(!_UR1movingTopic && (_UR1State - res[i]).norm2() < 0.01)){
      sleep(0.5);
    }
  }
}


void moverobot::print_T(Transform3D< double >& T)
{
  Transform3D<double> tempT = T;
  std::cout<<tempT.R()(0,0)<<" "<<tempT.R()(0,1)<<" "<<tempT.R()(0,2)<<" "<<tempT.P()[0]<<"\n"<<tempT.R()(1,0)<<" "<<tempT.R()(1,1)<<" "<<tempT.R()(1,2)<<" "<<tempT.P()[1]<<"\n"<<tempT.R()(2,0)<<" "<<tempT.R()(2,1)<<" "<<tempT.R()(2,2)<<" "<<tempT.P()[2]<<"\n";
}


Q_EXPORT_PLUGIN2(moverobot, moverobot);
//Q_EXPORT_PLUGIN2(moverobot,moverobot);

