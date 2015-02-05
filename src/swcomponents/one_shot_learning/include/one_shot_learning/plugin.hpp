/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef ONE_SHOT_LEARNING_HPP
#define ONE_SHOT_LEARNING_HPP

#include "one_shot_learning/ui_plugin.hpp"
#include <rws/RobWorkStudioPlugin.hpp>

#include <QVTKWidget.h>
#include <QWidget>
#include <QDir>
#include <QMap>
#include <QMutex>
#include <QThreadPool>
#include <stdio.h>

#include <caros/common.hpp>

// Modelling includes
//#include <one_shot_learning/RosCommunication.h>
#include <one_shot_learning/ObjectModeller.hpp>
#include <one_shot_learning/SharedData.hpp>
#include <one_shot_learning/ModelData.hpp>
#include "grasp_planner/Workcell.hpp"

// Grasp planning includes
#include <one_shot_learning/grasp_planner/Workcell.hpp>
#include <one_shot_learning/grasp_planner/Grasp_sampler.hpp>
#include <one_shot_learning/grasp_planner/Grasp_simulator.hpp>

#include <one_shot_learning/motion_planner/PlayBack.hpp>
#include <one_shot_learning/motion_planner/RobotController.hpp>

//PCL includes
#include <pcl/visualization/pcl_visualizer.h>

namespace dti{
namespace one_shot_learning
{
class ROSCommonNode
{
protected:
  ROSCommonNode()
  {
    int argc = 0;
    char** argv = NULL;
    try{
    ros::init(argc,argv,"one_shot_learning");
    }catch(ros::InvalidNodeNameException &e){

    }
  } 
};

class One_shot_learning : public rws::RobWorkStudioPlugin, private Ui::plugin, public ROSCommonNode
{
   Q_OBJECT
   Q_INTERFACES(rws::RobWorkStudioPlugin)

public:
    One_shot_learning();

    virtual ~One_shot_learning();
    
    virtual void initialize();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();
    void model_data(int model_index);

private:
    QVTKWidget 					*qv;
    QAction 					*myAction;
    QAction 					*actionReload;
    QAction 					*myLoadAction;
    QAction 					*loadSolidGTAction;
 //   QThread 					*_modeller_thread;
    QMutex 					_mutexState;
    QMutex 					_mutexRWS;
    QMutex 					_mutexRobot;
    QMutex					_mutexRWSUp;
    
    QTimer					*_updateTimer;
    QThreadPool					*_threadpool;
    
    rw::models::WorkCell::Ptr 			_rwc;
    rw::kinematics::State 			_state;
    rw::models::Device::Ptr 			_robot;
    std::string 				_gripper_name;
    unsigned int 				_sensor_num;
    dti::grasp_planning::Grasp_sampler*	_sampler;
    grasp_planning::Workcell* 			_grasp_wc;
    
    one_shot_learning::RosCommunication 	*rosComm;
    one_shot_learning::ObjectModeller 		*modeller;
    one_shot_learning::SharedData 		*sharedData;
    one_shot_learning::motion_planner::PlayBack* _pb; 
    motion_planner::RobotController* 		_ctrl;
    
    pcl::visualization::PCLVisualizer 		*pviz;
    QString 					m_name;
    int 					modelling_cnt;
    bool 					loaded;
    
    QMap<int, ModelData> 			_models;
    
    //Variables for grasp planning
    dti::grasp_planning::GripperType   	type;
    dti::grasp_planning::Grasp_simulator* 	_sim;
    std::string					_object_name;
    std::string					_gtask_path;
    std::string 				_replay_path;
    bool					_grasp_scene_loaded;
    unsigned int				_rotation_count;
private:    
    void init();
    void addModel(ModelData model);
    void addModelChild(QTreeWidgetItem *parent, QString name, QString size, QString extension, bool hasGraspTable);
    void addAndSaveModels(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model, pcl::PolygonMesh::Ptr solid_model);
    bool loadModels();
    void updateGrasptableWidget(QTreeWidgetItem *item, bool hasGraspTable);
    bool removeDir(const QString & dirName);
    void stateChangedListener(const rw::kinematics::State& state);
    void setState(const rw::kinematics::State& state){
       QMutexLocker locker(&_mutexState);
       _state = state;};
    rw::kinematics::State getState(void){
       QMutexLocker locker(&_mutexState);
       return _state;};
       
    void setRobot(const rw::models::Device::Ptr robot){
       QMutexLocker locker(&_mutexRobot);
       _robot = robot;};
    rw::models::Device::Ptr getRobot(void){
       QMutexLocker locker(&_mutexState);
       return _robot;};
    
    rws::RobWorkStudio* getRobWorkStudioSafe(void){
       QMutexLocker locker(&_mutexRWS);
       return getRobWorkStudio();};
       
    bool event(QEvent *event);
    

private Q_SLOTS:
    void btnCreateModelClicked();
    bool btnEstimatePoseClicked();
    bool btnGraspClicked();
    bool btnLoadGraspSceneClicked();
    bool btnGraspGenerationClicked();
    bool btnRotateObjectClicked();
    void checkBoxLogPoseChecked(bool checked);
    void deleteModel();
    void loadGTModel();
    void loadSolidGTModel();
    void treeWidgetClicked(QTreeWidgetItem* item, int col);
    void comboBoxRobot_changed(int item);
    void comboBoxGripper_changed(int item);
    void comboBoxSensor_changed(int item);
    
    void modelCreatedCallBack();
    void solidModelCreatedCallBack();
    void turnModelCallBack();

    void consoleOut(QString msg);
    void closeEvent(QCloseEvent *event);
    void updateRobotQ(rw::math::Q q, QString robot_name);
    void updateRobWorkStudio(void);
    void updateGripperQ(rw::math::Q q, QString gripper_name);
    void simulate(rw::trajectory::Path<rw::math::Q> path, QString device);
    void RefreshTreeWidget();
    
    void btnTestClicked();
    
    void sampleStatus(double percent);
    void sampleFinish(bool status);

Q_SIGNALS:
    void consoleOutSig(QString msg);
    void createModelSig(QString model_name);

};
}
}
#endif /* ONE_SHOT_LEARNING_HPP */