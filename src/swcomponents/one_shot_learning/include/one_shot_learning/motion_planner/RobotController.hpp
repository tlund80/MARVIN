//-------------Qt-------------------------------
#include <QThread>
#include <QObject>

#include <iostream>
#include <qt4/QtCore/qstring.h>
#include <one_shot_learning/RosCommunication.hpp>

//Motion planning includes
#include <one_shot_learning/motion_planner/path_optimization.hpp>
#include <one_shot_learning/motion_planner/planner.hpp>


#include <rw/math.hpp>

namespace dti{
namespace one_shot_learning {
  namespace motion_planner {
  enum MotionState {IDLE = 0, MOVE_TO_GRASP, APPROACH, GRASP, RETRACT, MOVE_TO_RELEASE, RELEASE, HOME};
class RobotController : public QThread {
      Q_OBJECT
 
public:
     RobotController(one_shot_learning::RosCommunication *rosComm);
     virtual ~RobotController();
     
     void initialize(rw::models::WorkCell::Ptr workcell, std::string robot); 
     void stop();
     void graspObject(rw::math::Transform3D<double> pose);
     void updateState(rw::kinematics::State& state){_state = state;};
     void setSpeed(float speed){_speed = speed;};
     void setSimulation(bool simulate){ _sim = simulate; };

private:
    void run();

    int _motionState;
    bool _state_done;
    bool _isRunning;
    bool _sim;
    one_shot_learning::RosCommunication *_rosComm;
    
    rw::math::Transform3D<double> _object_pose;
    rw::models::WorkCell::Ptr _rwc;
    rw::trajectory::Path<rw::math::Q> _path;
    rw::math::Q _currentQ;
    rw::kinematics::State _state;
    float _speed;
    std::string _robot_name;
 
    boost::shared_ptr<one_shot_learning::motion_planner::Planner> planner;
    boost::shared_ptr<one_shot_learning::motion_planner::PRMConfig>  planner_config;
    boost::shared_ptr<dti::grasp_planning::Workcell> _planning_workcell;
    boost::shared_ptr<one_shot_learning::motion_planner::PathLengthConfig> _path_length_config;
    boost::shared_ptr<one_shot_learning::motion_planner::Path_optimization> _path_optimizer;

public Q_SLOTS:
    void motionDone();
    void updateRobotQ(rw::math::Q q, QString robot_name);

Q_SIGNALS:
    void simulate(rw::trajectory::Path<rw::math::Q> path, QString device);

public:
  
 
  
};

}
}
}