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
One_shot_learning::One_shot_learning(): rws::RobWorkStudioPlugin("plugin", QIcon(":/DTI_logo.png"))
{
      setupUi(this);
      pviz.setBackgroundColor(0.5, 0.5, 0.5);
      qv = new QVTKWidget;
      vtkSmartPointer<vtkRenderWindow> renderWindow = pviz.getRenderWindow();
      qv->SetRenderWindow(renderWindow);
      qv->setFocusPolicy(Qt::StrongFocus);
      Ui_plugin::horizontalLayout->addWidget(qv);
      Ui_plugin::horizontalLayout->update();
      pviz.setWindowBorders(false);

      
      
     
      //Registre Qt meta types
     qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA> >("pcl::PointCloud<pcl::PointXYZRGBA>");
     qRegisterMetaType<pcl::PolygonMesh>("pcl::PolygonMesh"); 
     QCoreApplication::processEvents();
         
    sharedData = new SharedData();
    rosComm = new RosCommunication(sharedData);
    modeller = new ObjectModeller(sharedData);
    _ctrl = new  motion_planner::RobotController(rosComm);
    _grasp_wc = new dti::grasp_planning::Workcell();
  
     modelling_cnt = 0; // number of refinement steps
}

One_shot_learning::~One_shot_learning()
{
  delete sharedData;
  delete modeller;
  delete rosComm;
  delete _ctrl;
  delete _grasp_wc;
  
  delete qv;
  delete myAction;
  delete myLoadAction;
}

void One_shot_learning::initialize()
{
   Q_EMIT consoleOutSig("Initializing workcell!");
      loaded =false;
  //  rw::models::WorkCell::Ptr rwc =  getRobWorkStudio()->getWorkcell();
    getRobWorkStudioSafe()->stateChangedEvent().add(boost::bind(&One_shot_learning::stateChangedListener,this,_1), this);
    
   
 
}

void One_shot_learning::open(WorkCell* workcell)
{
   //Get or update workcell pointer
   _rwc =  workcell;
   setState(_rwc->getDefaultState());
   _pb = new motion_planner::PlayBack(_rwc,0.15);
   _pb->setRobworkStudio(getRobWorkStudioSafe());
    
}

bool One_shot_learning::event(QEvent *event)
{
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
void One_shot_learning::close()
{

}

void One_shot_learning::stateChangedListener(const rw::kinematics::State& state) {
  setState(state);
    //_state = state;
}


void One_shot_learning::init()
{
    
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
   // myAction->setIcon(QIcon(":/aCool.png"));
    myAction->setShortcut(tr("Ctrl+D"));
    myAction->setStatusTip(tr("Delete"));
    connect(myAction, SIGNAL(triggered()), this, SLOT(deleteModel()));
    
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
    _updateTimer = new QTimer();
    _updateTimer->setInterval(100);
  
    QObject::connect(rosComm, SIGNAL(rosShutdown()), this, SLOT(close()));
    connect(rosComm, SIGNAL(consoleSignal(QString)), this, SLOT(consoleOut(QString)), Qt::UniqueConnection);
    connect(rosComm, SIGNAL(robotPose(rw::math::Q,QString)), this, SLOT(updateRobotQ(rw::math::Q, QString)), Qt::DirectConnection);
    connect(rosComm, SIGNAL(gipperconfiguration(rw::math::Q,QString)), this, SLOT(updateGripperQ(rw::math::Q,QString)), Qt::DirectConnection);
    connect(modeller, SIGNAL(consoleSignal(QString)), this, SLOT(consoleOut(QString)), Qt::UniqueConnection);
    connect(modeller, SIGNAL(modelCreated(pcl::PointCloud<pcl::PointXYZRGBA>)), this, SLOT(modelCreatedCallBack(pcl::PointCloud<pcl::PointXYZRGBA>)), Qt::UniqueConnection);
    connect(modeller, SIGNAL(solidModelCreated(pcl::PointCloud<pcl::PointXYZRGBA>,pcl::PolygonMesh)), this, SLOT(solidModelCreatedCallBack(pcl::PointCloud<pcl::PointXYZRGBA>,pcl::PolygonMesh)), Qt::UniqueConnection);
    connect(modeller,SIGNAL(turnModel()),this,SLOT(turnModelCallBack()));
    connect(Ui_plugin::btnCreateModel,SIGNAL(pressed()), this, SLOT(btnCreateModelClicked()), Qt::UniqueConnection);
    connect(Ui_plugin::btnEstimatePose,SIGNAL(pressed()), this, SLOT(btnEstimatePoseClicked()));
    connect(Ui_plugin::btnGrasp,SIGNAL(pressed()), this, SLOT(btnGraspClicked()));
    connect(Ui_plugin::btnGenerateGraspTable, SIGNAL(pressed()), this, SLOT(btnGraspGenerationClicked()));
    connect(Ui_plugin::treeWidget,SIGNAL(itemClicked(QTreeWidgetItem*, int)),this,SLOT(treeWidgetClicked(QTreeWidgetItem*, int)));
    connect(Ui_plugin::checkBoxLogPose,SIGNAL(toggled(bool)), this, SLOT(checkBoxLogPoseChecked(bool)));
    //connect(Ui_plugin::checkBox,SIGNAL(toggled(bool)), this, SLOT(checkBoxLogPoseChecked(bool)));
    connect(Ui_plugin::comboBoxRobot,SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxRobot_changed(int)));
    connect(Ui_plugin::comboBoxGripper,SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxGripper_changed(int)));

    // Connect main thread to console
    connect(this, SIGNAL(consoleOutSig(QString)), this, SLOT(consoleOut(QString)), Qt::UniqueConnection );
    
    connect(Ui_plugin::btnTest,SIGNAL(pressed()), this, SLOT(btnTestClicked()));
    connect(_ctrl, SIGNAL(simulate(rw::trajectory::Path<rw::math::Q>,QString)),this, SLOT(simulate(rw::trajectory::Path<rw::math::Q>,QString)));
    connect(_updateTimer, SIGNAL(timeout()),this, SLOT(updateRobWorkStudio(void)));
    
    _updateTimer->start();
}

void One_shot_learning::simulate(rw::trajectory::Path<rw::math::Q> _path,QString device)
{
  //Visulize the path
  rw::models::Device::Ptr _device = getRobWorkStudioSafe()->getWorkcell()->findDevice(device.toStdString());
  
  if(!_device) RW_THROW("No " << " device for simulation!");
   _pb->setDevice(_device);
   _pb->setMotion(_path);
   _pb->forward();
}

void One_shot_learning::updateRobWorkStudio(void)
{
  getRobWorkStudioSafe()->setState(getState());
}

void One_shot_learning::btnTestClicked()
{ 
  /*  rw::math::Q openQ(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
    Q openQ1(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
    if(rosComm->SDHMoveQ(openQ)){
      for(int i = 0; i<2000000000; i++);
      rosComm->SDHMoveQ(openQ1);
    }
  */
}

void One_shot_learning::updateGripperQ(rw::math::Q q, QString gripper_name)
{
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
void One_shot_learning::updateRobotQ(rw::math::Q q, QString robot_name)
{
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
void One_shot_learning::loadSolidGTModel()
{
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

void One_shot_learning::loadGTModel()
{
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

void One_shot_learning::deleteModel()
{
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

void One_shot_learning::checkBoxLogPoseChecked(bool checked)
{

 // if(checked)	ui-> groupBoxPoseLogging->setEnabled(true);
//  else ui->groupBoxPoseLogging->setEnabled(false);
 
}

void One_shot_learning::btnCreateModelClicked()
{
 // std::cout <<"btnCreateModelClicked" << std::endl;

   m_name = Ui_plugin::lineModelName->text();
   if(m_name == "")
   {
       QMessageBox::information(this,"Model Name", "Please add a model name!");
   }
   else
   {
        Q_EMIT consoleOutSig("Creating new model from the scene!!\n");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	sharedData->getPointCloud(*cloud);
	std::cout << "cloud size" << (int)cloud->size() << std::endl;
	if(cloud->size() > 0) modeller->createmodel(*cloud);
		
   }
    Ui_plugin::lineModelName->clear(); 
}

bool One_shot_learning::btnEstimatePoseClicked()
{
  
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
	  //std::cout << "model_index: " << model_index << std::endl;
	  ModelData _data = _models.find(model_index).value();
	  QString  name = _data.getName();
	  if(selected_indexes.indexOf(model_index) == -1 && (index == 0 || index == 1))
	  {
	    //std::cout << "using estimated model!!" << std::endl;
	    pcl::PointCloud<pcl::PointXYZRGBA> model = _data.getPointCloud();
	    models.push_back(model);
	    names.push_back(name);
	    selected_indexes.push_back(model_index);
	  }else if(selected_indexes.indexOf(model_index) == -1 && (index ==2 || index == 3))
	  {
	    //std::cout << "using ground truth model!!" << std::endl;
	    pcl::PointCloud<pcl::PointXYZRGBA> model = _data.getGTPointCloud();
	    QString gt_name = "GT_" + name;
	    models.push_back(model);
	    names.push_back(gt_name);
	    selected_indexes.push_back(model_index);
	  }
	 
	  if(Ui_plugin::comboBoPEMethod->currentText().compare("CoViS") == 0)
	  {
	    std::vector<EstimationResult> res;
	    if(rosComm->PreparePoseEstimation(models,names,id_vec))
	    {
	    
	      if(rosComm->EstimationPose(id_vec, res))
	      {
		Q_EMIT consoleOutSig("Finish..............!\n");
	       
		for(size_t j = 0; j<= res.size()-1; j++){
		    geometry_msgs::Transform po = res[j].getPose();
		    _data.addPoseHypothesis(caros::toRw(po));
		    //res[j].getInlierFraction();
		    Q_EMIT consoleOutSig("Pose:\n");
		    Q_EMIT consoleOutSig("Translation:\n");
		    Q_EMIT consoleOutSig("\t x: " + QString::number(po.translation.x) + " y: " + QString::number(po.translation.y) + " z: " + QString::number(po.translation.z) + " \n");
		    Q_EMIT consoleOutSig("Rotation:\n");
		    Q_EMIT consoleOutSig("\t x: " + QString::number(po.rotation.x) + " y: " + QString::number(po.rotation.y) + " z: " + QString::number(po.rotation.z) + " w: " + QString::number(po.rotation.w) + " \n");
		
		    if(Ui_plugin::checkBoxLogPose)
		    { //Save Estimation result for visualization in Matlab
		    int scene_instance = Ui_plugin::spinBoxSceneInstance->value();
		    res[j].setSceneInstances(scene_instance);
		    res[j].setSceneName(Ui_plugin::lineSceneName->text());
		    res[j].addToLog("second_pose_estimation_result", true);
		    }
	          }
	      }
	      else{

	        QMessageBox::critical(this,"Error", "ould not estimate the pose of the model(s)!");
		return false;
	      }
	      
	    } 
	    else
	    {
	      QMessageBox::critical(this,"Error", "Could not add models or connect to the pose estimation node. Is the node running!!");
	    }
	    
	    
	  }else if(Ui_plugin::comboBoPEMethod->currentText().compare("Halcon") == 0)
	  {
	    QMessageBox::critical(this,"Error", "Halcon Pose estimation method is not implemented yet!!"); 
	  }
  
      } 
      
    }
    
   }
   
   return true;
}

bool One_shot_learning::btnGraspClicked()
{
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

bool One_shot_learning::btnGraspGenerationClicked()
{
  
  using namespace rw::kinematics;
  using namespace rw::models;
  using namespace dti::grasp_planning;
  using namespace rw::math;
  
   Q_EMIT consoleOutSig("Generating grasp tables!!\n");
   
   int model_index =  Ui_plugin::treeWidget->currentIndex().parent().row();
   int index = Ui_plugin::treeWidget->currentIndex().row();
   if(index < 0 && model_index < 0){
    RW_THROW("Please select a model in the list");
    return false;
   }
   ModelData _data = _models.find(model_index).value();
   //rw::geometry::Geometry _geometry;
   //rw::graphics::Model3D _model3d;
  
   if(index == 0 || index == 1){
     //Use estimated model
     
   }else{
     //Use ground truth model 
   }
 
   int samples = Ui_plugin::spinBoxSamples->value();
   int pertubations = Ui_plugin::spinBoxPertubations->value();
   int targets = Ui_plugin::spinBoxTarget->value();
   double sigma_a = Ui_plugin::lineEditsigmaRot->text().toDouble();
   double sigma_p = Ui_plugin::lineEditSigmaPos->text().toDouble();
   
   // Get the type of gripper 
   dti::grasp_planning::GripperType type;
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
   
   std::string abs_path = rosComm->getAbsoluteNodePath();
 
   std::stringstream ss;ss << abs_path; ss << "/geometry/grippers/SDHDeviceInvKin.wc.xml";
   std::string sdhInverseKin_path = ss.str();
   ss.str("");
   
   ss << abs_path; ss << "/geometry";
   std::string geometry_path; geometry_path = ss.str();
   ss.str("");

   ss << abs_path; ss << "/geometry/objects/uibkmodel.stl";
   std::string object_path = ss.str();
   ss.str("");  
   
   ss << abs_path; ss << "/geometry/gtask.rwtask";
   std::string gtask_path; gtask_path = ss.str();
   ss.str("");
   
   std::string obj_name = "object";
 
   _grasp_wc->addGripper(type, geometry_path);
  
//   Vector3D<double> _P(0.0, 0.0, 0.037063);
//   Rotation3D<double> _R; _R.identity();
 //  rw::math::Transform3D<double> _transform(_P,_R);
   
  // _grasp_wc->addModelFromFile(object_path, obj_name, _transform);

   //Update RobWorkStudio
   getRobWorkStudio()->setWorkCell(_grasp_wc->getWorkcell());
   getRobWorkStudio()->setState(_grasp_wc->getState());
  
   //Create sampler object
   _sampler = new dti::grasp_planning::Grasp_sampler(_grasp_wc);
   _sampler->LoadGripperInverseKin(sdhInverseKin_path);
   _sampler->generateTask(type, obj_name, targets);
   _sampler->Sample(samples);
  // _sampler->SaveGraspTask(gtask_path);
 
   Grasp_simulator* _sim = new Grasp_simulator(_grasp_wc, _sampler);
   _sim->RecordStatePath(true, geometry_path);
 //  try{
   _sim->SimulateGraspHypothesis(obj_name);
 //  }catch(boost::exception &e){
    //std::cout <<  << std::endl; 
 //  }
 //  _sim->FilterGrasps();  
 //  _sim->SimulateGraspPertubations(sigma_a, sigma_p,pertubations);
 //  _sim->calcPerturbedQuality(pertubations);
 //  _sim->FilterGrasps(); 
 //  _sim->SaveGraspTask(gtask_path, dti::grasp_planning::RWTASK);
  return true;
}

void One_shot_learning::modelCreatedCallBack(pcl::PointCloud<pcl::PointXYZRGBA> model)
{
 // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m = model.makeShared();
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(m);
 // pviz.addPointCloud<pcl::PointXYZRGBA>(m,rgb);
 // pviz.addPolygonMesh(solid_model);
 // pviz.addCoordinateSystem(0.1,0);
 // pviz.resetCameraViewpoint();
 
  modelling_cnt++;
  
  if(modelling_cnt < Ui_plugin::spinRefinemantSteps->value()){
    QMessageBox::information(this,"Turn object", "Please turn the object such that the bottom is visible!");
    
    Q_EMIT consoleOutSig("Refining model!!\n");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sharedData->getPointCloud(*cloud);
    std::cout << "cloud size" << (int)cloud->size() << std::endl;

    if(cloud->size() > 0) modeller->createmodel(*cloud);
  }else
  {
    Q_EMIT consoleOutSig("Reconstructing model and creating solid model!!\n");
    std::cout << "cloud size" << (int)model.size() << std::endl;
    modeller->createSolidmodel(model);
  }
}

void One_shot_learning::solidModelCreatedCallBack(pcl::PointCloud<pcl::PointXYZRGBA> model,pcl::PolygonMesh solid_model)
{
  Q_EMIT consoleOutSig("Solid model created!!\n");
  addAndSaveModels(model, solid_model); 
  RefreshTreeWidget();
  
  int points = (int)model.size();
  QString msg; msg.append("Model created with "); msg.append(QString::number(points)); msg.append(" points!!");
  consoleOut(msg);
}

void One_shot_learning::turnModelCallBack()
{
 
  
}

void One_shot_learning::treeWidgetClicked(QTreeWidgetItem* item, int col)
{
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
       pviz.removeAllPointClouds();
       pviz.addPointCloud<pcl::PointXYZRGBA>(p,rgb);
      }
      else if(index == 1)
      { // Show polygone mesh
	pcl::PolygonMesh mesh = _models.find(model_index).value().getMesh();
	if(mesh.polygons.size()<=0) RW_THROW("Failed to get the model!");
	pviz.removeAllPointClouds();
	pviz.addPolygonMesh(mesh);
      }else
      {// Show ground truth model
      //Check if the model is a point cloud or a polygone mesh
      if(item->text(2) == ".pcd"){

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p = _models.find(model_index).value().getGTPointCloud().makeShared();
	if(p->points.size()<=0) RW_THROW("Failed to get the model!");
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(p);
        pviz.removeAllPointClouds();
        pviz.addPointCloud<pcl::PointXYZRGBA>(p,rgb);
      }else if(item->text(2) == ".obj" || item->text(2) == ".stl"|| item->text(2) == ".vtk"){
	pcl::PolygonMesh mesh = _models.find(model_index).value().getGTMesh();
	if(mesh.polygons.size()<=0) RW_THROW("Failed to get the model!");
	pviz.removeAllPointClouds();
	pviz.addPolygonMesh(mesh);
      }
      }
      
      pviz.addCoordinateSystem(0.1,0);
      
       //Update cloud view
       qv->update();
      pviz.resetCamera();  //Focus the object in the center of the viewer
      
      //pviz.resetCameraViewpoint();
       
   }else
   {
      item->setExpanded(true);
   } 
 
}

void One_shot_learning::RefreshTreeWidget()
{
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
bool One_shot_learning::loadModels()
{
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
	    //std::cout << "estimated files: " << fileName.toStdString() << std::endl;
	    QStringList na = fileName.split(".");
	    
	    if(na[1].compare("pcd", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPCDFile(file_path.toStdString(),*pcd_file);
	      _data->setPointCloud(*pcd_file);
	    }else if(na[1].compare("vtk", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPolygonFileVTK(file_path.toStdString(), *mesh); 
	      _data->setMesh(*mesh);
	    }else if(na[1].compare("obj", Qt::CaseInsensitive)== 0){
		//Load rw models 
	//	rw::graphics::Model3D::Ptr model = rw::loaders::Model3DFactory::loadModel(file_path.toStdString(), na[0].toStdString());
	//	if(!model) RW_THROW("Could not load model from: " << file_path.toStdString());
	//	rw::geometry::Geometry::Ptr geom = rw::loaders::GeometryFactory::load(file_path.toStdString());
	//	geom->setName(na[0].toStdString());
	//	if(!geom) RW_THROW("Could not load geometry from: " << file_path.toStdString());
		
		//_data->setRwModel3D(model);
		//_data->setRwGeometry(geom);
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
	      //model_data->addName(na[0]);
	      //model_data->addPointCloud(*pcd_file);
	   
	    }else if(na[1].compare("vtk", Qt::CaseInsensitive)== 0){
	      pcl::io::loadPolygonFileVTK(file_path.toStdString(), *mesh); 
	      _data->setGTMesh(*mesh);
	      //model_data->addMesh(*mesh);
	   
	    }else if(na[1].compare("obj", Qt::CaseInsensitive)== 0){
	       pcl::io::loadPolygonFileOBJ(file_path.toStdString(), *mesh); 
	      _data->setGTMesh(*mesh);
	      
		//TODO: Load rw models 
		
	    }else if(na[1].compare("stl", Qt::CaseInsensitive)== 0){
	       pcl::io::loadPolygonFileSTL(file_path.toStdString(), *mesh); 
	      _data->setGTMesh(*mesh);
	      
		//TODO: Load rw models 
		
	    }
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

void One_shot_learning::addAndSaveModels(pcl::PointCloud<pcl::PointXYZRGBA> model, pcl::PolygonMesh solid_model)
{
    QString path = QString::fromStdString(rosComm->getAbsoluteNodePath());
    QDir modelDir = QDir(path + "/models");
    
    RW_THROW("You need to change the code for saving models!!");
   
    if(!modelDir.exists())
    {
      
   //   if(!modelDir.mkpath("pcd")) std::cout << "Could not create pcd model directory!!!" << std::endl;
   //   if(!modelDir.mkpath("vtk")) std::cout << "Could not create vtk model directory!!!" << std::endl;
   //   if(!modelDir.mkpath("obj")) std::cout << "Could not create obj model directory!!!" << std::endl; 
    }

/*    QString path_pcd = path + "/models/pcd/" + m_name + ".pcd";
    QString path_vtk = path + "/models/vtk/"  + m_name + ".vtk";
    QString path_obj = path + "/models/obj/"  + m_name + ".obj";
    
    if(!QFile(path_pcd).exists() || !QFile(path_pcd).exists() || !QFile(path_pcd).exists())
    {
      pcl::io::savePCDFile(path_pcd.toStdString(), model);
      pcl::io::saveVTKFile(path_vtk.toStdString(), solid_model);
      pcl::io::saveOBJFile(path_obj.toStdString(), solid_model);
    }
    else
    {
      QMessageBox::StandardButton reply;
      reply = QMessageBox::question(this,"Model already exists", "Do you want to overwrite the model!", QMessageBox::Yes|QMessageBox::No);
      
      if (reply == QMessageBox::Yes)
      {
	QFile::remove(path_pcd);
	QFile::remove(path_vtk);
	QFile::remove(path_obj);
	pcl::io::savePCDFile(path_pcd.toStdString(), model);
        pcl::io::saveVTKFile(path_vtk.toStdString(), solid_model);
        pcl::io::saveOBJFile(path_obj.toStdString(), solid_model);  
      }
    }
  */
}

void One_shot_learning::consoleOut(QString msg)
{
     log().info() << msg.toStdString() << "\n";
     log().error() << msg.toStdString() << "\n";
}

void One_shot_learning::closeEvent(QCloseEvent *event)
{
    //std::cout << "Closing Widget!!" << std::endl;
    modeller->shutdown();
    //Widget::closeEvent(event);
}

void One_shot_learning::addModel(ModelData model)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(Ui_plugin::treeWidget);
    item->setText(0,model.getName());

    this->treeWidget->addTopLevelItem(item);
    
    addModelChild(item,QString("    --point cloud"),QString::number(int(model.getPointCloud().points.size())),QString(".pcd"), model.has_gtask());
    addModelChild(item,QString("    --mesh model "),QString::number(int(model.getMesh().polygons.size())),QString(".vtk"), model.has_gtask());
    if(model.has_ground_truth_cloud()) addModelChild(item,QString("    --ground truth cloud"),QString::number(int(model.getGTPointCloud().points.size())),QString(".pcd"), model.has_ground_truth_gtask());  
    if(model.has_ground_truth_mesh()) addModelChild(item,QString("    --ground truth mesh"),QString::number(int(model.getGTMesh().polygons.size())),QString(".vtk"), model.has_ground_truth_gtask());
  
  
}

void One_shot_learning::addModelChild(QTreeWidgetItem *parent, QString name, QString size, QString extension, bool hasGraspTable)
{
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0,name);
    item->setText(1,size);
    item->setText(2,extension);
    std::string path = rosComm->getAbsoluteNodePath();
    
    if(hasGraspTable)item->setIcon(3,QIcon(QString(QString::fromStdString(path) + "/ui/checkmark.png")));
    else item->setIcon(3, QIcon(QString(QString::fromStdString(path) + "/ui/red_cross.png")));

    parent->addChild(item);

}

void One_shot_learning::comboBoxRobot_changed(int item)
{
  QString text = comboBoxRobot->itemText(item);
  setRobot(_rwc->findDevice(text.toStdString()));
  
 // if(!_robot) //RW_THROW("No device! ");
}

void One_shot_learning::comboBoxGripper_changed(int item)
{
  _gripper_name = comboBoxRobot->itemText(item).toStdString();
}

bool One_shot_learning::removeDir(const QString & dirName)
{
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
