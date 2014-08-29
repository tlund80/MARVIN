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

#include <boost/signals2.hpp>
#include <boost/make_shared.hpp>

#include "one_shot_learning/ui_plugin.hpp"
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwlibs/task/Motion.hpp>
#include <ros/ros.h>

#include <QVTKWidget.h>
#include <QWidget>
#include <QTreeWidgetItem>
#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QMap>
#include <QMutex>
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
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

namespace dti{
namespace one_shot_learning
{
class ROSCommonNode
{
protected:
  ROSCommonNode()
  {
    int argc;
    char** argv;
    ros::init(argc,argv,"one_shot_learning");
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
    QVTKWidget 				*qv;
    QAction 				*myAction;
    QAction 				*myLoadAction;
    QAction 				*loadSolidGTAction;
    
    QMutex 				_mutexState;
    QMutex 				_mutexRWS;
    QMutex 				_mutexRobot;
    QMutex				_mutexRWSUp;
    
    QTimer				*_updateTimer;
    
    rw::models::WorkCell::Ptr 		_rwc;
    rw::kinematics::State 		_state;
    rw::models::Device::Ptr 		_robot;
    std::string 			_gripper_name;
    dti::grasp_planning::Grasp_sampler* _sampler;
    grasp_planning::Workcell* 		_grasp_wc;
    
    one_shot_learning::RosCommunication *rosComm;
    one_shot_learning::ObjectModeller 	*modeller;
    one_shot_learning::SharedData 	*sharedData;
    one_shot_learning::motion_planner::PlayBack* _pb; 
    motion_planner::RobotController* 	_ctrl;
    
    pcl::visualization::PCLVisualizer 	pviz;
    QString 				m_name;
    int 				modelling_cnt;
    bool 				loaded;
    
    QMap<int, ModelData> 		_models;

private:    
    void init();
    void addModel(ModelData model);
    void addModelChild(QTreeWidgetItem *parent, QString name, QString size, QString extension, bool hasGraspTable);
    void addAndSaveModels(pcl::PointCloud<pcl::PointXYZRGBA> model, pcl::PolygonMesh solid_model);
    bool loadModels();
    void RefreshTreeWidget();
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
    void btnGraspClicked();
    void btnGraspGenerationClicked();
    void checkBoxLogPoseChecked(bool checked);
    void deleteModel();
    void loadGTModel();
    void loadSolidGTModel();
    void treeWidgetClicked(QTreeWidgetItem* item, int col);
    void comboBoxRobot_changed(int item);
    void comboBoxGripper_changed(int item);
    
    void modelCreatedCallBack(pcl::PointCloud<pcl::PointXYZRGBA> model);
    void solidModelCreatedCallBack(pcl::PointCloud<pcl::PointXYZRGBA> model,pcl::PolygonMesh solid_model);
    void turnModelCallBack();

    void consoleOut(QString msg);
    void closeEvent(QCloseEvent *event);
    void updateRobotQ(rw::math::Q q, QString robot_name);
    void updateRobWorkStudio(void);
    void updateGripperQ(rw::math::Q q, QString gripper_name);
    void simulate(rw::trajectory::Path<rw::math::Q> path, QString device);
    
    void btnTestClicked();

Q_SIGNALS:
    void consoleOutSig(QString msg);
    void createModelSig(QString model_name);

};
}
}
#endif /* ONE_SHOT_LEARNING_HPP */