/**
 * @file /src/main.cpp
 *
 * @brief Qt based RW plugin.
 *
 * @date November 2013
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <one_shot_learning/plugin.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudio.hpp>
#include <rw/common/Log.hpp>

#include <rw/loaders/model3d/LoaderOBJ.hpp>
#include <rw/loaders/model3d/STLFile.hpp>

// QT includes
#include <QStringList>
#include <QTreeWidgetItem>
#include <QMessageBox>
#include <QFileDialog>


// ROS includes
#include <ros/ros.h>

//PCL includes
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>

//VTK includes
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include <boost/signals2.hpp>
#include <boost/make_shared.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::models;

namespace dti{
namespace one_shot_learning
{
One_shot_learning::One_shot_learning(): rws::RobWorkStudioPlugin("plugin", QIcon(":/DTI_logo.png")){
  
      setupUi(this);
      
 /*     pviz = new  pcl::visualization::PCLVisualizer("", false);
      pviz->setBackgroundColor(0.5, 0.5, 0.5);
      qv = new QVTKWidget;
      vtkSmartPointer<vtkRenderWindow> renderWindow = pviz->getRenderWindow();
      qv->SetRenderWindow(renderWindow);
      qv->setFocusPolicy(Qt::StrongFocus);

      Ui_plugin::horizontalLayout->addWidget(qv);
      Ui_plugin::horizontalLayout->update();
      pviz->setWindowBorders(false);
     
      ///Registre Qt meta types
      qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA> >("pcl::PointCloud<pcl::PointXYZRGBA>");
      qRegisterMetaType<pcl::PolygonMesh>("pcl::PolygonMesh"); 
   
    */
     modelling_cnt = 0; // number of refinement steps
     type = dti::grasp_planning::SDH_PAR;
     _grasp_scene_loaded = false;
     _sensor_num = 0;
     _rotation_count = 0;

     std::cout << "Starting GUi in thread: " << QThread::currentThreadId() << std::endl;  
     
}

One_shot_learning::~One_shot_learning(){
  delete sharedData;
  delete modeller;
  delete rosComm;
  delete _ctrl;
  delete _grasp_wc;
  
  delete qv;
  delete pviz;
  delete myAction;
  delete actionReload;
  delete myLoadAction;
}

void One_shot_learning::initialize(){
   Q_EMIT consoleOutSig("Initializing workcell!");
      loaded =false;
    getRobWorkStudioSafe()->stateChangedEvent().add(boost::bind(&One_shot_learning::stateChangedListener,this,_1), this); 
 
}

void One_shot_learning::open(WorkCell* workcell){
   //Get or update workcell pointer
   _rwc =  workcell;
   setState(_rwc->getDefaultState());
   _pb = new motion_planner::PlayBack(_rwc,0.15);
   _pb->setRobworkStudio(getRobWorkStudioSafe());
    
}

bool One_shot_learning::event(QEvent *event){
 if((event->type() == QEvent::WindowActivate))
 {
   
   //Window is loaded for the first time
   if(!loaded)
   {
    loaded = true;
    init();
  
 //   QMessageBox msgBox(this); msgBox.setText("Loading avaliable models. Please wait!"); msgBox.setWindowTitle("Loading models");
 //   msgBox.setWindowModality(Qt::NonModal);
 //   msgBox.show();
    RefreshTreeWidget();
    rosComm->StartPPSubscriber();
    rosComm->StartRobotSubscriber();
    //TODO: recover SDH node if initializaion returns false
    rosComm->InitializeSDH();
    rosComm->StartSDHSubscriber();
    
   }
 }
  
  return true;
}

void One_shot_learning::close(){

}

void One_shot_learning::stateChangedListener(const rw::kinematics::State& state) {
  setState(state);
    //_state = state;
}

void One_shot_learning::init(){
       
     sharedData= new SharedData();
     rosComm = new RosCommunication(sharedData);
     modeller = new ObjectModeller(sharedData);
     _ctrl = new  motion_planner::RobotController(rosComm);
     _grasp_wc = new dti::grasp_planning::Workcell();
     _sampler = new dti::grasp_planning::Grasp_sampler();
      
    const int w = Ui_plugin::treeWidget->width();
    Ui_plugin::treeWidget->setColumnCount(4);
    Ui_plugin::treeWidget->setColumnWidth(0,w/4);
    Ui_plugin::treeWidget->setColumnWidth(1,w/4);
    Ui_plugin::treeWidget->setColumnWidth(2,w/4);
    Ui_plugin::treeWidget->setColumnWidth(3,w/4);
    Ui_plugin::treeWidget->headerItem()->setText(0,"Name");
    Ui_plugin::treeWidget->headerItem()->setText(1,"Points");
    Ui_plugin::treeWidget->headerItem()->setText(2,"Extension");
    Ui_plugin::treeWidget->headerItem()->setText(3,"Grasp table");
    Ui_plugin::treeWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
    Ui_plugin::treeWidget->setContextMenuPolicy(Qt::ActionsContextMenu);
    
    myAction = new QAction(tr("&Delete"), this);
    actionReload = new QAction(tr("&Reload models"), this);
    
   // myAction->setIcon(QIcon(":/aCool.png"));
    myAction->setShortcut(tr("Ctrl+D"));
    actionReload->setShortcut(tr("Ctrl+R"));
    myAction->setStatusTip(tr("Delete"));
    actionReload->setStatusTip(tr("Reload models"));
    connect(myAction, SIGNAL(triggered()), this, SLOT(deleteModel()));
    connect(actionReload, SIGNAL(triggered()), this, SLOT(RefreshTreeWidget()));    
  //  myLoadAction = new QAction(tr("&Load Point cloud Groud Truth Model"), this);
  //  loadSolidGTAction = new QAction(tr("&Load Solid Groud Truth Model"), this);

  //  myLoadAction->setShortcut(tr("Ctrl+L"));
  //  loadSolidGTAction->setShortcut(tr("Ctrl+S"));
  //  myLoadAction->setStatusTip(tr("Load Groud Truth Model"));
  //  loadSolidGTAction->setStatusTip(tr("Load Solid Groud Truth Model"));
  //  connect(myLoadAction, SIGNAL(triggered()), this, SLOT(loadGTModel()));
  //  connect(loadSolidGTAction, SIGNAL(triggered()), this, SLOT(loadSolidGTModel()));
    
  //  Ui_plugin::treeWidget->addAction(myLoadAction);
  //  Ui_plugin::treeWidget->addAction(loadSolidGTAction);
    Ui_plugin::treeWidget->addAction(myAction);
    Ui_plugin::treeWidget->addAction(actionReload);

  
    comboBoxRobot->clear();
    comboBoxGripper->clear();
    comboBoxGripperGraspGeneration->clear();
    comboBoxSelectSensor->clear();
    comboBoPEMethod->clear();
    //Add grippers
    comboBoxGripperGraspGeneration->addItem("SDH_PAR");
    comboBoxGripperGraspGeneration->addItem("SDH_PAR1");
    comboBoxGripperGraspGeneration->addItem("SDH_PAR2");
    comboBoxGripperGraspGeneration->addItem("SDH_PAR1_TABLE");
    comboBoxGripperGraspGeneration->addItem("SDH_PAR2_TABLE");
    comboBoxGripperGraspGeneration->addItem("SDH_BALL");
    comboBoxGripperGraspGeneration->addItem("SDH_CYL");
    comboBoxGripperGraspGeneration->addItem("PG70");
    comboBoxGripperGraspGeneration->addItem("SCUP");
    
    //Add sensors
    comboBoxSelectSensor->addItem("Kinect");
    comboBoxSelectSensor->addItem("Stereo");
    comboBoxSelectSensor->addItem("Both");
    
    //Add Pose estimation methods
    comboBoPEMethod->addItem("Halcon");
    comboBoPEMethod->addItem("CoViS");
    
    //Add robots
    std::vector<Device::Ptr> devices = _rwc->getDevices();
    setRobot(devices[1]);
    if(devices.size() < 1)
	RW_THROW("Can't find any device!!. Please load a workcell!!");
    
    for (std::vector<Device::Ptr>::iterator it = devices.begin(); it != devices.end(); ++it) {
      QString _name = QString::fromStdString((*it)->getName());
        if(_name.compare("SDH") == 0 || _name.compare("PG70") == 0) comboBoxGripper->addItem(_name);
	else comboBoxRobot->addItem(_name);
    }
   // _updateTimer = new QTimer();
   // _updateTimer->setInterval(100);
  
    QObject::connect(rosComm, SIGNAL(rosShutdown()), this, SLOT(close()));
    if(!connect(rosComm, SIGNAL(consoleSignal(QString)), this, SLOT(consoleOut(QString)), Qt::UniqueConnection))
       std::cerr << "Could not connect consoleSignal from RosCommunication class to main thread" << std::endl;
    if(!connect(rosComm, SIGNAL(robotPose(rw::math::Q,QString)), this, SLOT(updateRobotQ(rw::math::Q, QString)), Qt::DirectConnection))
       std::cerr << "Could not connect robotpose signal from RosCommunication class to main thread updateRobotQ()" << std::endl;
    if(!connect(rosComm, SIGNAL(gipperconfiguration(rw::math::Q,QString)), this, SLOT(updateGripperQ(rw::math::Q,QString)), Qt::DirectConnection))
      std::cerr << "Could not connect gipperconfiguration() signal from RosCommunication class to main thread updateGripperQ()" << std::endl;
  
     connect(modeller, SIGNAL(modelCreated()), this, SLOT(modelCreatedCallBack()));
	 
    // connect(&modeller, SIGNAL(consoleSignal(QString)), this, SLOT(consoleOut(QString)));
    //
 //    connect(&modeller, SIGNAL(consoleSignal(QString)), this, SLOT(consoleOut(QString)));
 /*   if(!connect(modeller, SIGNAL(consoleSignal(QString)), this, SLOT(consoleOut(QString)), Qt::UniqueConnection))
      std::cerr << "Could not connect consoleSignal() signal from modeller class to main thread consoleOut()" << std::endl;
      
 //   connect(modeller, SIGNAL(modelCreated()), this, SLOT(modelCreatedCallBack()), Qt::QueuedConnection);
    connect(modeller, SIGNAL(solidModelCreated()), this, SLOT(solidModelCreatedCallBack()), Qt::UniqueConnection);
    connect(modeller,SIGNAL(turnModel()),this,SLOT(turnModelCallBack()));
   */
   connect(_sampler,SIGNAL(status(double)),this,SLOT(sampleStatus(double)),Qt::DirectConnection);
   connect(_sampler,SIGNAL(finish_sampling(bool)),this,SLOT(sampleFinish(bool)),Qt::DirectConnection);
   
   
   /// Connect GUI elements
   connect(Ui_plugin::btnCreateModel,SIGNAL(pressed()), this, SLOT(btnCreateModelClicked()), Qt::UniqueConnection);
   connect(Ui_plugin::btnEstimatePose,SIGNAL(pressed()), this, SLOT(btnEstimatePoseClicked()));
   connect(Ui_plugin::btnGrasp,SIGNAL(pressed()), this, SLOT(btnGraspClicked()));
   connect(Ui_plugin::btnLoadGraspScene, SIGNAL(pressed()), this, SLOT(btnLoadGraspSceneClicked()));
   connect(Ui_plugin::btnGenerateGraspTable, SIGNAL( pressed()), this, SLOT(btnGraspGenerationClicked()));
   connect(Ui_plugin::btnRotateObject, SIGNAL( pressed()), this, SLOT(btnRotateObjectClicked()));
   connect(Ui_plugin::treeWidget,SIGNAL(itemClicked(QTreeWidgetItem*, int)),this,SLOT(treeWidgetClicked(QTreeWidgetItem*, int)));
   connect(Ui_plugin::checkBoxLogPose,SIGNAL(toggled(bool)), this, SLOT(checkBoxLogPoseChecked(bool)));
   //connect(Ui_plugin::checkBox,SIGNAL(toggled(bool)), this, SLOT(checkBoxLogPoseChecked(bool)));
   connect(Ui_plugin::comboBoxRobot,SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxRobot_changed(int)));
   connect(Ui_plugin::comboBoxGripper,SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxGripper_changed(int)));
   connect(Ui_plugin::comboBoxSelectSensor,SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxSensor_changed(int)));
   connect(Ui_plugin::btnTest,SIGNAL(pressed()), this, SLOT(btnTestClicked()));

   /// Connect main thread to console
   connect(this, SIGNAL(consoleOutSig(QString)), this, SLOT(consoleOut(QString)), Qt::UniqueConnection );
    
   
   connect(_ctrl, SIGNAL(simulate(rw::trajectory::Path<rw::math::Q>,QString)),this, SLOT(simulate(rw::trajectory::Path<rw::math::Q>,QString)));
   //connect(_updateTimer, SIGNAL(timeout()),this, SLOT(updateRobWorkStudio(void)));
    
   blockSignals(false);   
 //TODO:: Jeg har udkommenteret timeren
   //  _updateTimer->start();
}

void One_shot_learning::simulate(rw::trajectory::Path<rw::math::Q> _path,QString device){
  //Visulize the path
  rw::models::Device::Ptr _device = getRobWorkStudioSafe()->getWorkcell()->findDevice(device.toStdString());
  
  if(!_device) RW_THROW("No " << " device for simulation!");
   _pb->setDevice(_device);
   _pb->setMotion(_path);
   _pb->forward();
}

void One_shot_learning::updateRobWorkStudio(void){
  getRobWorkStudioSafe()->setState(getState());
  
}

void One_shot_learning::btnTestClicked(){ 
  
 // rosComm->ShowRandomDotPattern(2);
  /*  rw::math::Q openQ(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
    Q openQ1(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
    if(rosComm->SDHMoveQ(openQ)){
      for(int i = 0; i<2000000000; i++);
      rosComm->SDHMoveQ(openQ1);	    Q_EMIT modelCreated();
    }
  */
}

void One_shot_learning::updateGripperQ(rw::math::Q q, QString gripper_name){
  if(Ui_plugin::checkBoxLiveUpdate->isChecked()){
  std::string name;
  if(gripper_name.contains("/")){
  QStringList l = gripper_name.split("/");
  name = l[1].toStdString();
  }else name = gripper_name.toStdString();
  
  Device::Ptr _gripper;
  if(_rwc){
    try{
     //find the device to be sure to update the correct device
    _gripper = _rwc->findDevice(name);
    }catch(rw::common::Exception &e)
    {
      RW_THROW(e.what());
    }
    if(!_gripper) RW_THROW("Could not find device!");
  }
  
  if(_gripper && (q.size() == _gripper->getDOF())){;
    rw::kinematics::State s= getState();
    // std::cout << "hej" << std::endl;
     try{
      _gripper->setQ(q, s);
      setState(s);
     }catch(rw::common::Exception &e){
      RW_THROW(e.what());
    }
  }
  }
}

void One_shot_learning::updateRobotQ(rw::math::Q q, QString robot_name){
  if(Ui_plugin::checkBoxLiveUpdate->isChecked()){
  std::string name;
  if(robot_name.contains("/")){
  QStringList l = robot_name.split("/");
  name = l[1].toStdString();
  }else name = robot_name.toStdString();
 
  Device::Ptr _rob;
  if(_rwc){
    try{
     //find the device to be sure to update the correct device
    _rob = _rwc->findDevice(name);	    
    }catch(rw::common::Exception &e)
    {
      RW_THROW(e.what());
    }
    if(!_rob) RW_THROW("Could not find device!");
  }
  
  if(_rob && (q.size() == _rob->getDOF())){;
    rw::kinematics::State s= getState();
     try{
      _rob->setQ(q, s);
      setState(s);
     }catch(rw::common::Exception &e){
      RW_THROW(e.what());
    }
  }
  }
}

void One_shot_learning::loadSolidGTModel(){
  Q_EMIT consoleOutSig("Load Solid Ground Truth model!!\n");
/*  
  pcl::PolygonMesh::Ptr mesh_file(new pcl::PolygonMesh);
  
   QString selectedFilter;
   QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                 "",
                                                 tr("Files(*.obj *.stl *.vtk)"),
						 &selectedFilter);  
   
   //std::cout << selectedFilter.toStdString() << std::endl; 
   
  if(!fileName.isEmpty()){
  int model_index;
  QList<QTreeWidgetItem*> selected = Ui_plugin::treeWidget->selectedItems();
  
  for(int i = 0; i <= selected.length()-1; i++)
      {
	QTreeWidgetItem* item = selected[i];
   
	if(item->isExpanded() && item->childCount()==0)
	{ // Expanded items
	  QTreeWidgetItem* parent_ = item->parent();
	  model_index =   //model_data ->name.indexOf( parent_->text(0));
	}else
	{ //Parent item
	  model_index = model_data->name.indexOf( item->text(0));
	}
   
	if(model_index >= 0)
	{
	    std::cout << "model_index: " << model_index << std::endl;
	    //Store model in program memory
	    if(!fileName.isEmpty()) pcl::io::loadPolygonFileOBJ(fileName.toStdString(),*mesh_file);
	    QString str_name =  model_data->name[model_index];
	    model_data->addMeshGTModel(*mesh_file,model_index);
	    QString model_name = "    --ground_truth mesh";//"    --GT_" + str_name;
	    QList<QTreeWidgetItem*> selected = Ui_plugin::treeWidget->selectedItems();
	    
	    //Get corresponding scene model to compute coordinate system difference
	//    pcl::PointCloud<pcl::PointXYZRGBA> scene_model = model_data->pcdData[model_index];
	    
	    for(int i = 0; i <= selected.length()-1; i++)
	    {
	      QTreeWidgetItem* item = selected[i];
	      QString points = QString::number((int)mesh_file->polygons.size()) + " polygons";
	      addModelChild(item,model_name,points,".obj", model_data->hasGraspTable(model_index));
	        
	    }
	    
	  //  modeller->alignGTModelAndSceneModel(*pcd_file,scene_model);
	   // std::cout << "str_name: " << str_name.toStdString() << std::endl;
	}else
	{
	    Q_EMIT consoleOutSig("Error: could not add ground truth model\n");
	}
      }
  }
  */
}

void One_shot_learning::loadGTModel(){
  Q_EMIT consoleOutSig("Load Ground Truth model!!\n");
/*  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcd_file(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                 "",
                                                 tr("Files(*.pcd)"));
  if(!fileName.isEmpty()){
  int model_index;
  QList<QTreeWidgetItem*> selected = Ui_plugin::treeWidget->selectedItems();
  
  for(int i = 0; i <= selected.length()-1; i++)
      {
	QTreeWidgetItem* item = selected[i];
   
	if(item->isExpanded() && item->childCount()==0)
	{ // Expanded items
	  QTreeWidgetItem* parent_ = item->parent();
	  model_index = model_data->name.indexOf( parent_->text(0));
	}else
	{ //Parent item
	  model_index = model_data->name.indexOf( item->text(0));
	}
   
	if(model_index >= 0)
	{
	    std::cout << "model_index: " << model_index << std::endl;
	    //Store model in program memory
	    if(fileName != "") pcl::io::loadPCDFile(fileName.toStdString(),*pcd_file);
	    QString str_name =  model_data->name[model_index];
	    model_data->addGTModel(*pcd_file,model_index);
	    QString model_name = "    --ground_truth point cloud";//"    --GT_" + str_name;
	    QList<QTreeWidgetItem*> selected = Ui_plugin::treeWidget->selectedItems();
	    
	    //Get corresponding scene model to compute coordinate system difference
	//    pcl::PointCloud<pcl::PointXYZRGBA> scene_model = model_data->pcdData[model_index];
	    
	    for(int i = 0; i <= selected.length()-1; i++)
	    {
	      QTreeWidgetItem* item = selected[i];
	      QString points = QString::number((int)pcd_file->size()) + " points";
	      addModelChild(item,model_name,points,".pcd",model_data->hasGraspTable(model_index));
	        
	    }
	    
	  //  modeller->alignGTModelAndSceneModel(*pcd_file,scene_model);
	   // std::cout << "str_name: " << str_name.toStdString() << std::endl;
	}else
	{
	    Q_EMIT consoleOutSig("Error: could not add ground truth model\n");
	}
      }
  }
  */
}

void One_shot_learning::deleteModel(){
   Q_EMIT consoleOutSig("deleteModel!!\n");
   
   //Delete model from harddrive
   QString path = QString::fromStdString(rosComm->getAbsoluteNodePath());
   
   int model_index;
   int index;
   QList<QTreeWidgetItem*> selected = Ui_plugin::treeWidget->selectedItems();
   
   QMessageBox::StandardButton reply;
   reply = QMessageBox::question(this,"Delete model", "Do you want to delete the model from the harddrive?", QMessageBox::Yes|QMessageBox::No);
      
   if (reply == QMessageBox::Yes)
   {
      for(int i = 0; i <= selected.length()-1; i++)
      {
	QTreeWidgetItem* item = selected[i];
	
        if(item->parent()){
	  model_index = Ui_plugin::treeWidget->currentIndex().parent().row();
	  index = Ui_plugin::treeWidget->currentIndex().row();
	 // QTreeWidgetItem* parent_ = item->parent();
	 // delete parent_;
	}else{
	  model_index = Ui_plugin::treeWidget->currentIndex().row();
	  index = -1;
	  //delete item;
	}
	
	if(model_index >= 0)
	{
	    std::cout << "model_index: " << model_index << std::endl;
	    std::cout << "index: " << index << std::endl;
	 
	    if(index == -1)
	    {
	      // delete entire model 
	      reply = QMessageBox::question(this,
					    "Delete model",
					    "Your are about to delete all model related data including grasp table, ground truth models and estimated model from the harddrive! Do you want to continue?",
					    QMessageBox::Yes|QMessageBox::No);
      
	      if (reply == QMessageBox::Yes)
	      {
		  //Delete file from harddrive
	          QString str_name =  _models.find(model_index).value().getName();
		  QString remove_path = path + "/models/" + str_name + "/";
		  std::cout << remove_path.toStdString() << std::endl; 
		    if(removeDir(remove_path)){
		      //Delete the model from the treewidget
		      delete item;
		      if(!_models.remove(model_index)) QMessageBox::information(this,"Remove file", "Could not remove the model from memory!",QMessageBox::Ok);
		      
		    }else
		      QMessageBox::information(this,"Remove file", "Could not remove the model from the harddrive!",QMessageBox::Ok);	
	      }
	    }else
	    {
	      //delete only the exact model
		RW_THROW("Not implemented yet!!");
	 /*     ModelData data = _models.find(model_index).value();
	      if(index == 0) data.delete_PointCloud();
	      if(index == 1) data.delete_Mesh();
	      if(index == 2) data.delete_PointCloud_GT();
	      if(index == 3) data.delete_Mesh_GT();;
	      delete item;
	   */   
	    }
	}
      }
      
   }
   
    //RefreshTreeWidget();
    
}

void One_shot_learning::checkBoxLogPoseChecked(bool checked){

 // if(checked)	ui-> groupBoxPoseLogging->setEnabled(true);
//  else ui->groupBoxPoseLogging->setEnabled(false);
 
}

void One_shot_learning::btnCreateModelClicked(){
   m_name = Ui_plugin::lineModelName->text();
   if(m_name == ""){
       QMessageBox::information(this,"Model Name", "Please add a model name!");
   }
   else{
        Q_EMIT consoleOutSig("Creating new model from the scene!!\n");
	//sharedData->getPointCloud(*cloud);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	rosComm->getSceneModel(_sensor_num, 0.020,cloud);
	
	/*_modeller_thread = new QThread();
	modeller->moveToThread(_modeller_thread);
	
	connect(_modeller_thread, SIGNAL(started()), modeller, SLOT(run()));
	connect(_modeller_thread, SIGNAL(finished()), this, SLOT(modelCreatedCallBack()), Qt::UniqueConnection);
	connect(modeller, SIGNAL(finished()), _modeller_thread, SLOT(quit()));
        connect(modeller, SIGNAL(finished()), modeller, SLOT(deleteLater()));
	connect(_modeller_thread, SIGNAL(finished()), _modeller_thread, SLOT(deleteLater()));
	*/
	if(cloud->size() > 0){
	  modeller->createmodel(*cloud);
	}
	
	
   }
    Ui_plugin::lineModelName->clear(); 
}

bool One_shot_learning::btnEstimatePoseClicked(){
  
   using namespace rw::kinematics;
   using namespace rw::models;
   
   Q_EMIT consoleOutSig("Estimating pose of the object!!\n"); 
   
   if((Ui_plugin::checkBoxLogPose && Ui_plugin::lineSceneName->text().isEmpty()))
   {
     QMessageBox::information(this,"Scene name and instances", "Please provide number of object instances in the scene and a scene name!");
    
   }else if(Ui_plugin::treeWidget->selectedItems().count() < 1)
   {
     QMessageBox::information(this,"Select a model", "Please select a model in the model tree!");
   }
   else{
   
   int model_index;
   int index;
   QList<QTreeWidgetItem*> selected = Ui_plugin::treeWidget->selectedItems();
   
   
   std::vector<std::pair<int, ModelData> > data;
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > models;
   std::vector<QString> names;
   std::vector<std::string> id_vec;
   
   QList<int> selected_indexes;
   
   for(int i = 0; i <= selected.length()-1; i++)
   {
      QTreeWidgetItem* item = selected[i];
      
      if(!item->parent()){
	 QMessageBox::information(this,"Select a model", "Please select a model(s) from the child list. Not the parent list!!");
	 return false;
       }else{
	model_index = Ui_plugin::treeWidget->currentIndex().parent().row();
	index = Ui_plugin::treeWidget->currentIndex().row();
      }
    
      if(model_index >= 0)
      {
	  ModelData _data = _models.find(model_index).value();
	  QString  name = _data.getName();
	  data.push_back(std::pair<int, ModelData>(index,_data));
	  
	  if(selected_indexes.indexOf(model_index) == -1 && (index == 0 || index == 1)){
	    pcl::PointCloud<pcl::PointXYZRGBA> model = _data.getPointCloud();
	
	    models.push_back(model);
	    names.push_back(name);
	    selected_indexes.push_back(model_index);
	    
	  }else if(selected_indexes.indexOf(model_index) == -1 && (index ==2 || index == 3)){
	    pcl::PointCloud<pcl::PointXYZRGBA> model = _data.getGTPointCloud();
	    QString gt_name = "GT_" + name;

	    models.push_back(model);
	    names.push_back(gt_name);
	    selected_indexes.push_back(model_index);
	  }
      } 
      
    } //Finish adding models. Now estimate the pose of all objects and add them to the workcell
    
     std::vector<EstimationResult> res;
     //***************************************************************************  
     //************************** Covis method **********************************
     //***************************************************************************
     if(Ui_plugin::comboBoPEMethod->currentText().compare("CoViS") == 0)
     {
      
       if(rosComm->PreparePoseEstimation(models,names,id_vec)){
	 if(rosComm->EstimationPose(id_vec, res)){
	   Q_EMIT consoleOutSig("Finish..............!\n");
	   
	   for(size_t j = 0; j<= res.size()-1; j++){
	     geometry_msgs::Transform po = res[j].getPose();
	     data[j].second.addPoseHypothesis(caros::toRw(po));
	     //res[j].getInlierFraction();
	     Q_EMIT consoleOutSig("Pose:\n");
	     Q_EMIT consoleOutSig("Translation:\n");
	     Q_EMIT consoleOutSig("\t x: " + QString::number(po.translation.x) + " y: " + QString::number(po.translation.y) + " z: " + QString::number(po.translation.z) + " \n");
	     Q_EMIT consoleOutSig("Rotation:\n");
	     Q_EMIT consoleOutSig("\t x: " + QString::number(po.rotation.x) + " y: " + QString::number(po.rotation.y) + " z: " + QString::number(po.rotation.z) + " w: " + QString::number(po.rotation.w) + " \n");
		
	     //Save Estimation result for visualization in Matlab
	     if(Ui_plugin::checkBoxLogPose){ 
	       int scene_instance = Ui_plugin::spinBoxSceneInstance->value();
	       res[j].setSceneInstances(scene_instance);
	       res[j].setSceneName(Ui_plugin::lineSceneName->text());
	       res[j].addToLog("second_pose_estimation_result", true);
	    }		    
	  }
	}
	else{
	  QMessageBox::critical(this,"Error", "Could not estimate the pose of the model(s)!");
	  return false;
	}
      }else{
	QMessageBox::critical(this,"Error", "Could not add models or connect to the pose estimation node. Is the node running!!");
      }
	    
	  //***************************************************************************  
	  //************************** Halcon method **********************************
	  //***************************************************************************
	  }else if(Ui_plugin::comboBoPEMethod->currentText().compare("Halcon") == 0)
	  {
	    QMessageBox::critical(this,"Error", "Halcon Pose estimation method is not implemented yet!!"); 
	  }
    
    	// Iterate through all objects and insert object(s) into scene 	    
    	for(unsigned int k = 0; k<data.size(); k++){	 
	  MovableFrame::Ptr _frame;
	  Object::Ptr _obj;
	  if(data[k].first < 2){
	   _frame = MovableFrame::Ptr(new MovableFrame(data[k].second.getName().toStdString()));
	   _rwc->addFrame(_frame.get());
	   _state = _rwc->getDefaultState();
	   _frame->setTransform(data[k].second.getBestPose(), _state);
	   _obj = Object::Ptr(new Object(_frame.get()));
	   _obj->addGeometry(data[k].second.getRwGeometry().get());
	   _obj->addModel(data[k].second.getRwModel3D().get());
	  }else{
	   _frame = MovableFrame::Ptr(new MovableFrame("GT_" + data[k].second.getName().toStdString())); 
	   _rwc->addFrame(_frame.get());
	    _state = _rwc->getDefaultState();
	   _frame->setTransform(data[k].second.getBestPose(), _state);
	   _obj = Object::Ptr(new Object(_frame.get()));
	   _obj->addGeometry(data[k].second.getGTRwGeometry().get());
	   _obj->addModel(data[k].second.getGTRwModel3D().get());
	  }
	  _rwc->add(_obj);	  
	}	    
   }
   
   return true;
}

bool One_shot_learning::btnGraspClicked(){
   using namespace dti::one_shot_learning::motion_planner;
   using namespace rw::math;
    
   Q_EMIT consoleOutSig("Grasping the object!!\n");
   
   int model_index =  Ui_plugin::treeWidget->currentIndex().parent().row();
   ModelData _data = _models.find(model_index).value();
   
   if(!_data.has_gtask()){
     RW_THROW("There exists no grasp table for the choosen objects!");  
     return false;
   }
   std::string _choosen_robot = comboBoxRobot->currentText().toStdString();
   
   _ctrl->initialize(_rwc, _choosen_robot);
   _ctrl->start(QThread::HighestPriority);
   
  // Transform3D< double > pos = rw::kinematics::Kinematics::frameTframe(_robot->getBase(), _robot->getEnd(), _state);
  // rw::math::Vector3D<double> _p(0.381, -0.456, 0.526);
  // rw::math::RPY<double> _rpy(1.679, 0.165, -3.081);
  // rw::math::Transform3D<double> _pose(_p,_rpy.toRotation3D()); 

   if(Ui_plugin::checkBoxSimulate->isChecked()) _ctrl->setSimulation(true);
   else _ctrl->setSimulation(false);
   
   _ctrl->setSpeed(0.5);
   //_ctrl->graspObject(pos);
/*    if(!_ctrl->graspObject(_data)){
      RW_THROW("Something went wrong in parsing the grasp table for the object!!");  
      return false;
    }
  */  
     return true;
}

bool One_shot_learning::btnLoadGraspSceneClicked(){
   using namespace rw::math;
   using namespace dti::grasp_planning;
 
   delete _grasp_wc;
   _grasp_scene_loaded = false;
   _rotation_count = 0;
   
   _grasp_wc = new Workcell();
   
   Q_EMIT consoleOutSig("Loading the grasping scene!!\n");
   
    // Get the type of gripper use in the simulation
   std::string gripName =  Ui_plugin::comboBoxGripperGraspGeneration->currentText().toStdString();
   if(gripName.compare("SDH_PAR") == 0) type = SDH_PAR;
   else if(gripName.compare("SDH_PAR1") == 0) type = SDH_PAR1;
   else if(gripName.compare("SDH_PAR2") == 0) type = SDH_PAR2;
   else if(gripName.compare("SDH_PAR1_TABLE") == 0) type = SDH_PAR1_TABLE;
   else if(gripName.compare("SDH_PAR2_TABLE") == 0) type = SDH_PAR2_TABLE;
   else if(gripName.compare("SDH_BALL") == 0) type = SDH_BALL;
   else if(gripName.compare("SDH_CYL") == 0) type = SDH_CYL;
   else if(gripName.compare("PG70") == 0) type = PG70;
   else if(gripName.compare("SCUP") == 0) type = SCUP;
     
   int model_index =  Ui_plugin::treeWidget->currentIndex().parent().row();
   int index = Ui_plugin::treeWidget->currentIndex().row();
   if(index < 0 && model_index < 0){
    RW_WARN("Please select a model in the list");
    return false;
   }
   
   std::string abs_path = rosComm->getAbsoluteNodePath();
   std::stringstream ss;
   
   ModelData _data = _models.find(model_index).value();
   rw::geometry::Geometry::Ptr _geometry;
   rw::graphics::Model3D::Ptr _model3d;
 
   if(index == 0 || index == 1){
     //Use estimated model
     _geometry =  _data.getRwGeometry();
     _model3d = _data.getRwModel3D();
     ss << abs_path; ss << "/models/"; ss << _data.getName().toStdString(); ss << "/estimated/"; ss << gripName; ss << "_gtask_"; ss << _data.getName().toStdString(); ss << "_0"; ss << ".rwtask";   
     _gtask_path = ss.str(); ss.str("");
      ss << abs_path; ss << "/models/"; ss << _data.getName().toStdString(); ss << "/estimated/"; ss << gripName; ss << "_replay_"; ss << _data.getName().toStdString(); ss << "_0"; ss << ".rwplay";   
     _replay_path = ss.str(); ss.str("");
     if(_geometry.isNull() || _model3d.isNull()) return false;
   }else{
     //Use ground truth model 
     _geometry = _data.getGTRwGeometry();
     _model3d = _data.getGTRwModel3D();
      ss << abs_path; ss << "/models/"; ss << _data.getName().toStdString(); ss << "/ground_truth/"; ss << gripName; ss << "_gtask_"; ss << _data.getName().toStdString(); ss << "_0";  ss << ".rwtask";   
      _gtask_path = ss.str(); ss.str("");
      ss << abs_path; ss << "/models/"; ss << _data.getName().toStdString(); ss << "/ground_truth/"; ss << gripName; ss << "_replay_";ss << _data.getName().toStdString(); ss << "_0";  ss << ".rwplay";
      _replay_path = ss.str(); ss.str("");
      if(_geometry.isNull() || _model3d.isNull()) return false;
   }
   
   ss << abs_path; ss << "/geometry";
   std::string geometry_path; geometry_path = ss.str();
   ss.str("");   
  
   _object_name = "object";//_data.getName().toStdString();
 
   _grasp_wc->addGripper(type, geometry_path);
  
   Vector3D<double> _P(0.0, 0.0, 0.037063);
   Rotation3D<double> _R; _R.identity();
   rw::math::Transform3D<double> _transform(_P,_R);
   
   _grasp_wc->addModel(_object_name, _transform, _model3d, _geometry);
  //_grasp_wc->addModelFromFile(object_path, obj_name, _transform);
   _grasp_wc->setInitGripperState(type);

   //Update RobWorkStudio
   getRobWorkStudio()->setWorkCell(_grasp_wc->getWorkcell());
   getRobWorkStudio()->setState(_grasp_wc->getState());
     
   _grasp_scene_loaded = true;
   
   return true;
}

bool One_shot_learning::btnGraspGenerationClicked(){
  
  using namespace rw::kinematics;
  using namespace rw::models;
  using namespace dti::grasp_planning;
  using namespace rw::math;
  
  if(!_grasp_scene_loaded){
    RW_WARN("You must load a grasp scene!");
    return false;
  }
   Q_EMIT consoleOutSig("Generating grasp tables!!\n");

   int samples = Ui_plugin::spinBoxSamples->value();
   int targets = Ui_plugin::spinBoxTarget->value();
  
   
   std::string abs_path = rosComm->getAbsoluteNodePath();
   std::stringstream ss;ss << abs_path; ss << "/geometry/grippers/SDHDeviceInvKin.wc.xml";
   std::string sdhInverseKin_path = ss.str();
   ss.str("");

   //Create sampler object
   _sampler->setWorkcell(_grasp_wc);
   _sampler->setGripperInverseKinPath(sdhInverseKin_path);
   _sampler->setGripperType(type);
   _sampler->setObjectName(_object_name);
   _sampler->setNumberOfTargets(targets);
   _sampler->setSamples(samples);
   
   _sampler->start();

  // _sampler->SaveGraspTask(_gtask_path);

   //Clean up
  // delete _sampler;
  // delete _sim;
   
  return true;
}

bool One_shot_learning::btnRotateObjectClicked(){
  using namespace rw::kinematics;
  using namespace rw::math;
  
   if(!_grasp_scene_loaded){
    RW_WARN("You must load a grasp scene!");
    return false;
  }
  if(_rotation_count >= 16) _rotation_count = 0;
 
  RPY<> _RPY;
  if(_rotation_count < 4){
    _RPY = RPY<>(0.0,0.0, _rotation_count * 1.57);
  }else if(_rotation_count >= 4 && _rotation_count < 8) {
    _RPY = RPY<>((_rotation_count-4) * 1.57, 0.0, 0.0);
  }else if(_rotation_count >= 8 && _rotation_count < 12) {
    _RPY = RPY<>(0.0, (_rotation_count-8) * 1.57, 0.0);
  }else if(_rotation_count >= 12 && _rotation_count < 16) {
    _RPY = RPY<>((_rotation_count-12) * 1.57,0.0, 1.57);
  }
  
  
  Vector3D<> T(0,0,0.037063);
  Transform3D<> TR(T,_RPY);
  
  //Get the right model
  int model_index =  Ui_plugin::treeWidget->currentIndex().parent().row();
  ModelData _data = _models.find(model_index).value();
  Frame::Ptr _f = _grasp_wc->getWorkcell()->findFrame("object");  //();
  if(!_f) RW_THROW("Could not find frame 'object' ");
  MovableFrame::Ptr _frame = _f.cast<MovableFrame>();
  
  //Find the object 
  Object::Ptr obj = _grasp_wc->getWorkcell()->findObject(_data.getName().toStdString());
  if(!_f) RW_THROW("Could not find object: " << _data.getName().toStdString());
  
 // rw::math::Transform3D<> TRA = _f->wTf(_state);
 // std::cout << TRA << std::endl;
 // rw::math::Transform3D<> TRANSFORMATION = TRA *TR;
  _frame->setTransform(TR ,_state);
  getRobWorkStudio()->setState(_state);
 
  //Ensure that the modification is applied to the grasp planning scene
  _grasp_wc->setWorkcell(getRobWorkStudio()->getWorkcell());
   _rotation_count++;
   return true;
}

void One_shot_learning::modelCreatedCallBack(){
  modelling_cnt++;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
   std::cout << "modelling_cnt: " << modelling_cnt << std::endl;
  if(modelling_cnt < 2){ //Ui_plugin::spinRefinemantSteps->value()
    QMessageBox::information(this,"Turn object", "Please turn the object such that the bottom is visible!");
    std::cout << "Refining model!! "  << std::endl;
    Q_EMIT consoleOutSig("Refining model!!\n");
   // sharedData->getPointCloud(*cloud);
    if(!rosComm) std::cout << "Help!!" << std::endl;
      rosComm->getSceneModel(_sensor_num, 0.020,cloud);
    
    if(cloud->size() > 0) modeller->createmodel(*cloud);
    
  }else
  {
    Q_EMIT consoleOutSig("Reconstructing model and creating solid model!!\n");
     std::cout << "Reconstructing model and creating solid model!!" << std::endl;
    modeller->getCloudModel(cloud);
    modeller->createSolidmodel(*cloud);
  }
}

void One_shot_learning::solidModelCreatedCallBack(){
  Q_EMIT consoleOutSig("Solid model created!!\n");
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
   modeller->getCloudModel(cloud);
   modeller->getMeshModel(*mesh);
   
  addAndSaveModels(cloud, mesh); 
  RefreshTreeWidget();
  
//  int points = (int)cloud->size();
//  QString msg; msg.append("Model created with "); msg.append(QString::number(points)); msg.append(" points!!");
//  consoleOut(msg);
}

void One_shot_learning::turnModelCallBack(){
 
  
}

void One_shot_learning::treeWidgetClicked(QTreeWidgetItem* item, int col){
 //std::cout << "click!!" << std::endl;
 
   if(item->isExpanded() && item->childCount()==0)
   {
     int index = Ui_plugin::treeWidget->currentIndex().row();
     int model_index =  Ui_plugin::treeWidget->currentIndex().parent().row();
  
      if(index == 0)
      {// Show pointCloud
       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p = _models.find(model_index).value().getPointCloud().makeShared();
       if(p->points.size()<=0) RW_THROW("Failed to get the model!");
       pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(p);
       pviz->removeAllPointClouds();
       pviz->addPointCloud<pcl::PointXYZRGBA>(p,rgb);
      }
      else if(index == 1)
      { // Show polygone mesh
	pcl::PolygonMesh mesh = _models.find(model_index).value().getMesh();
	if(mesh.polygons.size()<=0) RW_THROW("Failed to get the model!");
	pviz->removeAllPointClouds();
	pviz->addPolygonMesh(mesh);
      }else
      {// Show ground truth model
      //Check if the model is a point cloud or a polygone mesh
      if(item->text(2) == ".pcd"){

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p = _models.find(model_index).value().getGTPointCloud().makeShared();
	if(p->points.size()<=0) RW_THROW("Failed to get the model!");
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(p);
        pviz->removeAllPointClouds();
        pviz->addPointCloud<pcl::PointXYZRGBA>(p,rgb);
      }else if(item->text(2) == ".obj" || item->text(2) == ".stl"|| item->text(2) == ".vtk"){
	pcl::PolygonMesh mesh = _models.find(model_index).value().getGTMesh();
	if(mesh.polygons.size()<=0) RW_THROW("Failed to get the model!");
	pviz->removeAllPointClouds();
	pviz->addPolygonMesh(mesh);
      }
      }
      
      pviz->addCoordinateSystem(0.1,0);
      
       //Update cloud view
       qv->update();
      pviz->resetCamera();  //Focus the object in the center of the viewer
      
      //pviz.resetCameraViewpoint();
       
   }else
   {
      item->setExpanded(true);
   } 
 
}

void One_shot_learning::RefreshTreeWidget(){
   Ui_plugin::treeWidget->clear();
     //Clear list with models
     _models.clear();
    
    if(loadModels())
    {
     // std::cout << "models: " << _models.size() << std::endl;
      for(int i = 0; i<=_models.size()-1; i++)
      {
	ModelData _data = _models.find(i).value();
	addModel(_data);
      }
    }
}

bool One_shot_learning::loadModels(){
  // Allocate objects
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcd_file(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
  
  QString path = QString::fromStdString(rosComm->getAbsoluteNodePath());
  QDir modelDir = QDir(path + "/models");
  log().info() << "Current model path: " << modelDir.absolutePath().toStdString() << "\n";
  int index = 0;
    
  if(modelDir.exists())
  {
   QStringList folders = modelDir.entryList(QDir::NoDotAndDotDot | 
						 QDir::Dirs,
						 QDir::Name);//(QDir::Filter::Files,QDir::SortFlag::NoSort)
	
   //iterate through all folders
   for(QStringList::iterator it = folders.begin(); it != folders.end(); it++)
   { 
      QString directory = *it;
      QDir subDir = QDir(path + "/models/" + directory +"/");
      ModelData* _data = new ModelData();
      _data->setName(directory);

      std::cout << "Loading model from: " << subDir.absolutePath().toStdString() << std::endl;
       
       QStringList _subsubdir = subDir.entryList(QDir::NoDotAndDotDot | 
						 QDir::Dirs,
						 QDir::Name);//(QDir::Filter::Files,QDir::SortFlag::NoSort)
						 
						 
      for(QStringList::iterator itr = _subsubdir.begin(); itr != _subsubdir.end(); itr++)
      {
        QString su = *itr;  
	QDir _subsubdir = QDir(path + "/models/" + directory +"/" + su);

	//std::cout << "subsubDir: " << _subsubdir.absolutePath().toStdString() << std::endl;
	QStringList _files = _subsubdir.entryList(QDir::NoDotAndDotDot | 
						QDir::Files,
						QDir::Name);//(QDir::Filter::Files,QDir::SortFlag::NoSort)
	//std::cout << "Number of files in folder: " << _files.size() << std::endl;				 

	if(su.compare("estimated", Qt::CaseInsensitive) == 0){
	  for(QStringList::iterator itra = _files.begin(); itra != _files.end(); itra++)
	  {
	    QString fileName = *itra;
	    QString file_path = _subsubdir.absolutePath() + "/" + fileName; 
	  //  std::cout << "estimated files: " << fileName.toStdString() << std::endl;
	    QStringList na = fileName.split(".");
	    
	    if(na[1].compare("pcd", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPCDFile(file_path.toStdString(),*pcd_file);
	      _data->setPointCloud(*pcd_file);
	    }/*else if(na[1].compare("vtk", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPolygonFileVTK(file_path.toStdString(), *mesh); 
	      _data->setMesh(*mesh);
	     */ 
	    /*  QString stl_path = subDir.absolutePath() + + "/estimated/" + na[0] + ".stl";
	      pcl::PolygonMesh p = *mesh;
	      std::cout << "stl_path: " << stl_path.toStdString() << std::endl;
	      pcl::io::savePolygonFileSTL(stl_path.toStdString(),p);
	      */
	    //}
	    else if(na[1].compare("stl", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPolygonFileSTL(file_path.toStdString(), *mesh); 
	      _data->setMesh(*mesh);
	      //Load rw models 
		rw::graphics::Model3D::Ptr model = rw::loaders::Model3DFactory::loadModel(file_path.toStdString(), na[0].toStdString());
		if(!model) RW_THROW("Could not load model from: " << file_path.toStdString());
		rw::geometry::Geometry::Ptr geom = rw::loaders::GeometryFactory::load(file_path.toStdString());
		geom->setName(na[0].toStdString());
		if(!geom) RW_THROW("Could not load geometry from: " << file_path.toStdString());
		
		_data->setRwModel3D(model);
		_data->setRwGeometry(geom);
	    }else if(na[1].compare("rwtask", Qt::CaseInsensitive)== 0){
	      
	       rwlibs::task::GraspTask::Ptr gtask = GraspTask::load(file_path.toStdString());
	       _data->addGraspTask(gtask);
	    }
	    
	  }
	}else if (su.compare("ground_truth", Qt::CaseInsensitive) == 0)
	{
	  for(QStringList::iterator itra = _files.begin(); itra != _files.end(); itra++)
	  {
	    QString fileName = *itra;
	    QString file_path = _subsubdir.absolutePath() + "/" + fileName; 
	    //std::cout << "estimated files: " << fileName.toStdString() << std::endl;
	    QStringList na = fileName.split(".");
	    
	    if(na[1].compare("pcd", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPCDFile(file_path.toStdString(),*pcd_file);
	      _data->setGTPointCloud(*pcd_file);	   
	    }/*else if(na[1].compare("vtk", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPolygonFileVTK(file_path.toStdString(), *mesh); 
	      _data->setGTMesh(*mesh);
	    }*/else if(na[1].compare("stl", Qt::CaseInsensitive)== 0){
	       pcl::io::loadPolygonFileSTL(file_path.toStdString(), *mesh); 
	      _data->setGTMesh(*mesh);
	    
	       //Load rw models 
	        rw::graphics::Model3D::Ptr model = rw::loaders::Model3DFactory::loadModel(file_path.toStdString(), na[0].toStdString());
		if(!model) RW_THROW("Could not load model from: " << file_path.toStdString());
		rw::geometry::Geometry::Ptr geom = rw::loaders::GeometryFactory::load(file_path.toStdString());
		geom->setName(na[0].toStdString());
		if(!geom) RW_THROW("Could not load geometry from: " << file_path.toStdString());
		
		_data->setGTRwModel3D(model);
		_data->setGTRwGeometry(geom);
		
	    }else if(na[1].compare("rwtask", Qt::CaseInsensitive)== 0){
	      
	       rwlibs::task::GraspTask::Ptr gtask = GraspTask::load(file_path.toStdString());
	       _data->addGTGraspTask(gtask);
	    }
	    
	    
	    /*else if(na[1].compare("obj", Qt::CaseInsensitive)== 0){
	       pcl::io::loadPolygonFileOBJ(file_path.toStdString(), *mesh); 
	      _data->setGTMesh(*mesh);      
	      //Load rw models 
	      try{
	  //    rw::graphics::Model3D::Ptr model = rw::loaders::Model3DFactory::loadModel(file_path.toStdString(), na[0].toStdString());
	  //    if(!model) RW_THROW("Could not load model from: " << file_path.toStdString());
	  //    rw::geometry::Geometry::Ptr geom = rw::loaders::GeometryFactory::load(file_path.toStdString());
	  //    geom->setName(na[0].toStdString());
	  //    if(!geom) RW_THROW("Could not load geometry from: " << file_path.toStdString());
	     
		
	      }catch(rw::common::Exception &e)
	      {
		RW_THROW("Could not load ground_truth model. EXCEPTION: " << e.what());
	      }
	      */
	      /*
	      _data->setRwModel3D(model);
	      _data->setRwGeometry(geom);
		*/
	    //}
	  }  
	}
      }
      _models.insert(index,*_data);
      delete _data;
      index++;					 
   }
    return true;
  }else
  { 
    QString err; err.append("Could not load models from "); err.append(path); err.append("/models\n");
    Q_EMIT consoleOutSig(err);
   
    if(modelDir.mkdir(path + "/models")){
       err.clear(); err.append("Could not create directory: "); err.append(path); err.append("/models\n");
       Q_EMIT consoleOutSig(err);
    }
      return false;
  } 
}

void One_shot_learning::addAndSaveModels(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model, pcl::PolygonMesh::Ptr solid_model){
    QString path = QString::fromStdString(rosComm->getAbsoluteNodePath());
    QDir modelDir = QDir(path + "/models");
    
  //  RW_THROW("You need to change the code for saving models!!");
   
    if(!modelDir.exists()){
         if(!modelDir.mkpath("models")) std::cout << "Could not create model directory!!!" << std::endl;
    }else{
      
      if(!modelDir.mkpath(m_name)) std::cout << "Could not create " << m_name.toStdString() <<" directory!!!" << std::endl;
      modelDir = QDir(path + "/models/" + m_name);
      if(!modelDir.mkpath("estimated")) std::cout << "Could not create estimated directory!!!" << std::endl;
      if(!modelDir.mkpath("ground_truth")) std::cout << "Could not create ground directory!!!" << std::endl;
      
      
      QString path_estimate_pcd = modelDir.absolutePath() + "/estimated/" + m_name + ".pcd";
      QString path_estimate_stl = modelDir.absolutePath() + "/estimated/" + m_name + ".stl";
      QString path_estimate_obj = modelDir.absolutePath() + "/estimated/" + m_name + ".obj";
      
      std::cout << path_estimate_pcd.toStdString() << std::endl;

    
    if(!QFile(path_estimate_pcd).exists() || !QFile(path_estimate_stl).exists() || !QFile(path_estimate_obj).exists())
    {
      try{
	    pcl::io::savePCDFile(path_estimate_pcd.toStdString(), *model);
	    pcl::io::savePolygonFileSTL(path_estimate_stl.toStdString(), *solid_model);
	    pcl::io::saveOBJFile(path_estimate_obj.toStdString(), *solid_model);
      }catch(pcl::IOException &e){
	    pcl::console::print_error(e.what());
      }
    }
    else
    {
  /*    QMessageBox::StandardButton reply;
      reply = QMessageBox::question(this,"Model already exists", "Do you want to overwrite the model!", QMessageBox::Yes|QMessageBox::No);
      
      if (reply == QMessageBox::Yes){
	 try{
		pcl::io::savePCDFile(path_estimate_pcd.toStdString(), *model);
		pcl::io::savePolygonFileSTL(path_estimate_stl.toStdString(), *solid_model);
		pcl::io::saveOBJFile(path_estimate_obj.toStdString(), *solid_model);
	    }catch(pcl::IOException &e){
		pcl::console::print_error(e.what());
	    }
      }
      */
    }
      
    }

   
}

void One_shot_learning::consoleOut(QString msg){
     log().info() << msg.toStdString() << "\n";
     log().error() << msg.toStdString() << "\n";
}

void One_shot_learning::closeEvent(QCloseEvent *event){
    //std::cout << "Closing Widget!!" << std::endl;
    modeller->shutdown();
    //Widget::closeEvent(event);
}

void One_shot_learning::addModel(ModelData model){
    QTreeWidgetItem *item = new QTreeWidgetItem(Ui_plugin::treeWidget);
    item->setText(0,model.getName());

    this->treeWidget->addTopLevelItem(item);
    
    addModelChild(item,QString("    --point cloud"),QString::number(int(model.getPointCloud().points.size())),QString(".pcd"), model.has_gtask());
    addModelChild(item,QString("    --mesh model "),QString::number(int(model.getMesh().polygons.size())),QString(".stl"), model.has_gtask());
    if(model.has_ground_truth_cloud()) addModelChild(item,QString("    --ground truth cloud"),QString::number(int(model.getGTPointCloud().points.size())),QString(".pcd"), model.has_ground_truth_gtask());  
    if(model.has_ground_truth_mesh()) addModelChild(item,QString("    --ground truth mesh"),QString::number(int(model.getGTMesh().polygons.size())),QString(".stl"), model.has_ground_truth_gtask());
  
  
}

void One_shot_learning::addModelChild(QTreeWidgetItem *parent, QString name, QString size, QString extension, bool hasGraspTable){
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0,name);
    item->setText(1,size);
    item->setText(2,extension);
  
    updateGrasptableWidget(item,hasGraspTable);

    parent->addChild(item);

}

void One_shot_learning::updateGrasptableWidget(QTreeWidgetItem *item, bool hasGraspTable){
    
  std::string path = rosComm->getAbsoluteNodePath();
  if(hasGraspTable)item->setIcon(3,QIcon(QString(QString::fromStdString(path) + "/ui/checkmark.png")));
  else item->setIcon(3, QIcon(QString(QString::fromStdString(path) + "/ui/red_cross.png")));
}

void One_shot_learning::comboBoxRobot_changed(int item){
  QString text = comboBoxRobot->itemText(item);
  setRobot(_rwc->findDevice(text.toStdString()));
  
 // if(!_robot) //RW_THROW("No device! ");
}

void One_shot_learning::comboBoxGripper_changed(int item){
  _gripper_name = comboBoxGripper->itemText(item).toStdString();
}

void One_shot_learning::comboBoxSensor_changed(int item){
  _sensor_num = item; 
  switch(_sensor_num){ 
  
    case 0: //Kinect
       rosComm->ShowBlack();
      break;
      
    case 1: //stereo
      rosComm->ShowRandomDotPattern(3);
      break;
      
    case 2: //both
      rosComm->ShowRandomDotPattern(3);
      break;
  }
}

void One_shot_learning::sampleStatus(double percent){
   std::cout << "\r";
   std::cout << "Status: " << percent << "%";
   std::cout << std::flush;

}

void One_shot_learning::sampleFinish(bool status){
  
  if(status){ 
    
    double sigma_a = Ui_plugin::lineEditsigmaRot->text().toDouble();
    double sigma_p = Ui_plugin::lineEditSigmaPos->text().toDouble();
    int pertubations = Ui_plugin::spinBoxPertubations->value();
   
    delete _sim;
   _sim = new dti::grasp_planning::Grasp_simulator(_sampler);
   _sim->setWorkcell(_grasp_wc);
    connect(_sim,SIGNAL(status(double)),this,SLOT(sampleStatus(double)),Qt::DirectConnection);
   // connect(_sim,SIGNAL(finish_sampling(bool)),this,SLOT(sampleFinish(bool)),Qt::DirectConnection);
   _sim->RecordStatePath(true, _replay_path);
   _sim->setObjectName(_object_name);
   _sim->setPertubationData(sigma_a,sigma_p,pertubations);
   _sim->setSavePath(_gtask_path, dti::grasp_planning::RWTASK);
   _sim->start();
  
   // _sim->SimulateGraspHypothesis(_object_name);

  // _sim->FilterGrasps();  
  // _sim->SimulateGraspPertubations(sigma_a, sigma_p,pertubations);
  // _sim->calcPerturbedQuality(pertubations);
 //  _sim->FilterGrasps(); 
  // _sim->SaveGraspTask();
  }else{
    
  } 
}

bool One_shot_learning::removeDir(const QString & dirName){
    bool result = false;
    QDir dir(dirName);

    if (dir.exists(dirName)) {
        Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst)) {
            if (info.isDir()) {
                result = removeDir(info.absoluteFilePath());
            }
            else {
                result = QFile::remove(info.absoluteFilePath());
            }

            if (!result) {
                return result;
            }
        }
        result = dir.rmdir(dirName);
    }
    return result;
}
Q_EXPORT_PLUGIN2(One_shot_learning, One_shot_learning)
}
}
