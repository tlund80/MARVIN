#ifndef UI_CALIBRATION_VERIFY_HPP
#define UI_CALIBRATION_VERIFY_HPP

//standard
#include <iostream>
#include <string>
#include <cstdlib>
#include <stdio.h>

//for cylinder crop
#include <stdlib.h>
#include <math.h>
#include <deque>
#include <vector>

//chess board corner detection
#include <opencv2/opencv.hpp>
#include <Mathematics/StereoCalibration.h>

//for get image from camera   //BUG we need to move around some header file to make boost work.
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <caros_common_msgs/StereoCalibration.h>

// UI
#include "ui_interface.h"

//QT
#include <QTimer>
//#include <QPushButton>

// Robwork
#include <rws/RobWorkStudioPlugin.hpp>
#include <RobWorkStudio.hpp>
#include <rw/loaders/WorkCellLoader.hpp> 
#include <rw/kinematics/MovableFrame.hpp> 
#include <rwlibs/opengl/DrawableFactory.hpp> 
#include <rw/kinematics/Kinematics.hpp> 
#include <rw/rw.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <sandbox/invkin/ClosedFormURSolver.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rw/math.hpp>
#include <rw/common/Timer.hpp>
#include <rwlibs/calibration.hpp>

//Ros to RW
#include <caros/common.hpp>

// Ros
#include "ros/ros.h"
#include "ros/package.h"
#include <sensor_msgs/PointCloud2.h>

// Ros message
#include <caros_control_msgs/RobotState.h>
// #include "marvin_common/SDHState.h"
// #include "marvin_common/SDH_DSAState.h"
// #include "marvin_common/SDHTemp.h"

//Ros service
#include <caros_control_msgs/SerialDeviceMovePTP.h>
// #include "marvin_common/SDHMoveQ.h"
// #include "marvin_common/SDHGripQ.h"
// #include "marvin_common/SDHStop.h"

//threads
#include <boost/thread.hpp>

//namespace
using namespace rw::common; 
using namespace rw::models;  
using namespace rw::math; 
using namespace rw::kinematics;

using namespace message_filters;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace cv;
using Calibration::StereoCalibration;

using namespace std;

class moverobot: public rws::RobWorkStudioPlugin, private Ui::Plugin {
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
  
  moverobot();
  virtual ~moverobot();

  virtual void open(rw::models::WorkCell* workcell); 
  virtual void close(); 
  virtual void initialize();

private slots:

  void drawLoop();
  //void moveLoop();
  void stateChangedListener(const rw::kinematics::State& state);
  
  //update function
  void _upUR();
  
  //control function
  //void callback(const marvin_common::RobotState::ConstPtr& state);
  void _updateScene();
  void clickEvent();
  
  Transform3D<double> ToWorld(Transform3D<double>& T);
  std::vector<Q> _IKSolver(const rw::math::Transform3D<> pose, State actualState);
  rw::trajectory::QPath _moveRobotT(const Transform3D<>& T, State actualState);
  rw::trajectory::QPath _moveRobotQ(const rw::math::Q robot, State actualState);
  void _moveRobot(const rw::trajectory::QPath res);
  
  void bottonCB(int flag);
  
  void print_T(Transform3D<double>& T);
  
  void Grab_scene();
  
protected:
  
  rw::models::WorkCell::Ptr _WorkCell;
  rw::kinematics::State _state;
  rw::models::Device::Ptr _deviceUR1;
  rwlibs::calibration::WorkCellCalibration::Ptr _workcellCalibration ;
  
  rw::math::Q Q_home;
  
  Transform3D<double> left_top;
  Transform3D<double> right_top;
  Transform3D<double> left_bottom;
  Transform3D<double> right_bottom;
  
  Transform3D<double> left_top_retreat;
  Transform3D<double> right_top_retreat;
  Transform3D<double> left_bottom_retreat;
  Transform3D<double> right_bottom_retreat;
  
  Transform3D<double> tcpTtip;
  
  double retreat_dist;
  
  int location;
  
  //Ros subscriber
  ros::Subscriber _poseUR1Subscriber;
  
  //Ros service client
  ros::ServiceClient _clientUR1;
  ros::ServiceClient _clientChessboard;
  
  //Ros handlers
  void handleUR1State(const caros_control_msgs::RobotState::ConstPtr& state);
  
  // RobWorkStudio Configurations
  rw::math::Q _UR1State;
  
  //update status
  bool _UR1StateUpdate;
  
  //close the windows or not
  bool _end;
  
  //moving status
  //bool _UR1MoveStatus;
  
  //move status from topic
  bool _UR1movingTopic;

  //Robot intrinsic calibration is used when true
  bool _useIntrinsicCalib;

  //Realworld robot is moved when true
  bool _move_Robot;

  //Ros
  rw::common::Ptr<ros::NodeHandle> _nodeHnd;
  //ros::NodeHandle _nodeHnd;
  
  //threads
  boost::thread* _drawThread;
  
  //
  std::string work_dir;
  
  //calibration 
  //from base to BB1
  Transform3D<double> T1;
  //from BB1 to BB1
  Transform3D<double> T2;
  
  
  QTimer *_timer1;
  
};

#endif
