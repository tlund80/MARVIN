

#include <one_shot_learning/RosCommunication.hpp>
#include <ros/package.h>

namespace dti{
namespace one_shot_learning {
 
RosCommunication::RosCommunication(SharedData *data) : 
	_sharedData(data)

{
	init();
}

RosCommunication::~RosCommunication() {
    if(ros::isStarted()) {
      
      sub_pointCloud.shutdown();
      
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
	exit();
}

bool RosCommunication::init() {
	//ros::init(init_argc,init_argv,"object_modeller_gui");
	qRegisterMetaType<rw::math::Q>("rw::math::Q");
	if ( ! ros::master::check() ) {
		return false;
	}
		
	ROS_INFO("ROS Node is running!");
	emit consoleSignal("ROS Node is running!");
	start();
	//run();
	return true;
}

bool RosCommunication::MoveRobotLinear(std::vector<rw::math::Transform3D<double> >& path, std::vector<float>& blends, float speed)
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
 
  _moveL_srv = n.serviceClient<caros_control_msgs::SerialDeviceMoveLin>("universalrobots/URMoveL");
	
  caros_control_msgs::SerialDeviceMoveLinRequest _req; 
  caros_control_msgs::SerialDeviceMoveLinResponse _res;
  BOOST_FOREACH(rw::math::Transform3D<double>& target, path) {
    _req.targets.push_back(caros::toRos(target)); 
  }

  BOOST_FOREACH(float& blend, blends) {
    _req.blends.push_back(blend);
    _req.speeds.push_back(speed);
  }

  if(!_moveL_srv.call(_req,_res)) return false;
  
   Q_EMIT finish();
  
  return _res.success;
}

bool RosCommunication::MoveRobotJoint(std::vector<rw::math::Q>& q, std::vector<float>& blends, float speed)
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  
  _moveJ_srv = n.serviceClient<caros_control_msgs::SerialDeviceMovePTP>("universalrobots/URMoveQ");
  
  caros_control_msgs::SerialDeviceMovePTPRequest _req;
  caros_control_msgs::SerialDeviceMovePTPResponse _res;
 
  BOOST_FOREACH(rw::math::Q& target, q) {
     caros_common_msgs::Q _q;
     _q.data[0] = target[0];
     _q.data[1] = target[1];
     _q.data[2] = target[2];
     _q.data[3] = target[3];
     _q.data[4] = target[4];
     _q.data[5] = target[5];
    _req.q_targets.push_back(_q); 
  }

  BOOST_FOREACH(float& blend, blends) {
    _req.blends.push_back(blend);
    _req.speeds.push_back(speed);
  }


  if(!_moveJ_srv.call(_req,_res)) return false;
  
    Q_EMIT finish();
  
  return _res.success;
}

bool RosCommunication::StopRobot()
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  
  _stop_srv = n.serviceClient<caros_common_msgs::Stop>("universalrobots/Stop");
  
  caros_common_msgs::StopRequest _req;
  caros_common_msgs::StopResponse _res; 
  
  if(!_stop_srv.call(_req,_res)) return false;
  return _res.success;
}

bool RosCommunication::SDHStop()
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  
  _SDH_stop_srv = n.serviceClient<caros_common_msgs::Stop>("sdh/Stop");
	
  caros_common_msgs::StopRequest _req;
  caros_common_msgs::StopResponse _res; 
    
  if(!_SDH_stop_srv.call(_req,_res)) return false;
  return _res.success;
}
 
bool RosCommunication::SDHPause()
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  
  _SDH_pause_srv = n.serviceClient<caros_common_msgs::Pause>("sdh/Pause");
  
  caros_common_msgs::PauseRequest _req;
  caros_common_msgs::PauseResponse _res;
 
  if(!_SDH_pause_srv.call(_req,_res)) return false;
  return _res.success;
}
 
bool RosCommunication::SDHGrasp(rw::math::Q &q)
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  
  _SDH_grasp_srv = n.serviceClient<caros_control_msgs::GripperGripQ>("sdh/GripperGripQ");
  
  caros_control_msgs::GripperGripQRequest _req;
  caros_control_msgs::GripperGripQResponse _res;
 
  for(int i = 0;i==int(q.size()); i++)
    _req.q.data[i] = q[i];
  
  if(!_SDH_grasp_srv.call(_req,_res)) return false;
  
  Q_EMIT finish();
  
  return _res.success;
}

bool RosCommunication::SDHMoveQ(rw::math::Q &q)
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  
  _SDH_move_srv = n.serviceClient<caros_control_msgs::GripperMoveQ>("sdh/GripperMoveQ");
	
  caros_control_msgs::GripperMoveQRequest _req;
  caros_control_msgs::GripperMoveQResponse _res;
  
  for(int i = 0;i==int(q.size()); i++)
    _req.q.data[i] = q[i];
  
  if(!_SDH_move_srv.call(_req,_res)) return false;
  
  Q_EMIT finish();
  
  return _res.success;
}

std::string RosCommunication::getAbsoluteNodePath()
{
   ros::start(); // explicitly needed since our nodehandle is going out of scope.
   ros::NodeHandle n;
   return ros::package::getPath("one_shot_learning");  
}


void RosCommunication::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input) 
{
  //  std::cout << "cloudCallBack" << std::endl;
  // Convert input message to pointcloud
   pcl::PCLPointCloud2 out;
   pcl_conversions::toPCL(*input, out);
   pcl::fromPCLPointCloud2(out, cloud);
   _sharedData->setPointCloud(cloud);
}

void RosCommunication::robotCallBack(const caros_control_msgs::RobotStateConstPtr& msg) 
{
  //  std::cout << "robotCallBack" << std::endl;
  
  if(msg->isColliding) std::cout << "============ Robot is Colliding!! =============" << std::endl;
  if(msg->estopped) std::cout << "============ Robot is in emergency stop!! =============" << std::endl;
  //if(msg->securityStopped) std::cout << "============ Robot is in security stop!! =============" << std::endl; 
  
  rw::math::Q _q;
  for(int i = 0; i= msg->q.data.size(); i++)
     _q[i] = msg->q.data[i];
  
  rw::math::Q _vel;
  for(int i = 0; i= msg->dq.data.size(); i++)
     _vel[i] = msg->dq.data[i];
  
  _sharedData->setRobotMoving(msg->isMoving);
  _sharedData->setRobotQ(_q);
  _sharedData->setRobotJointVelocity(_vel);
  
  Q_EMIT robotPose(_q);
}

void RosCommunication::run() {
	ros::spin();
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void RosCommunication::StartRobotSubscriber()
{
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  _sub_robotState = n.subscribe("/universalrobots/RobotState",1,&RosCommunication::robotCallBack,this);
}

void RosCommunication::StartPPSubscriber() 
{
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	sub_pointCloud = n.subscribe("/global/cloud_pcd",1,&RosCommunication::cloudCallBack,this);
  
}
  
  
bool RosCommunication::PreparePoseEstimation(std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > model, std::vector<QString> name, std::vector<std::string> &id_vec)
{
   ros::start(); // explicitly needed since our nodehandle is going out of scope.
   ros::NodeHandle n;

   // Subscribe to prepare estimation service to create a surface model
   ROS_INFO("Subscribing to prepare pose estimation service...");
   if(!ros::service::waitForService("/pose_estimation_covis/prepareEstimation", ros::Duration(2))) return false;
   ros::ServiceClient prepare = n.serviceClient<pose_estimation_covis::prepareEstimation>("/pose_estimation_CoViS/prepareEstimation");
 
   sensor_msgs::PointCloud2 ros_model;
   pose_estimation_covis::prepareEstimation pMsg;
   
   for(size_t j = 0; j<= model.size()-1; j++)
   {
	pcl::PointCloud<pcl::PointXYZRGBA> m = model.back();
	pcl::toROSMsg(m,ros_model);

	pMsg.request.model.push_back(ros_model);
	pMsg.request.model_name.push_back(name.back().toStdString());
   }
   
  if(!prepare.call(pMsg)){
	  ROS_ERROR("Something went wrong when calling prepare pose estimation service");
	  return false;
  }
  //std::vector<std::string> id_vec;
  
  for(size_t i = 0; i<= pMsg.response.model_id.size()-1; i++)
  {
	  std::string model_id = pMsg.response.model_id[i];
	  ROS_INFO("Model id: %s", model_id.c_str());
	  id_vec.push_back(model_id);
  }
  return true;
}
  
  
bool RosCommunication::EstimationPose(std::vector<std::string> id_vec, std::vector<EstimationResult> &result)
{
   ros::start(); // explicitly needed since our nodehandle is going out of scope.
   ros::NodeHandle n;
   
   // Subscribe to global estimation service
   ROS_INFO("Subscribing to pose estimation service...");
   if(!ros::service::waitForService("/pose_estimation_covis/estimate", ros::Duration(2))) return false;
   ros::ServiceClient estimate = n.serviceClient<pose_estimation_covis::estimate>("/pose_estimation_CoViS/estimate");

   pose_estimation_covis::estimate eMsg;
  
   //Adding models to recognize
   for(size_t j = 0; j <= id_vec.size()-1; j++){
     // std::cout << "id_vec: " << id_vec[j] << std::endl;
	   eMsg.request.model_id.push_back(id_vec[j]);
   }
   eMsg.request.print = true;
   eMsg.request.local_refinement = true;

   if(!estimate.call(eMsg)){
   	  ROS_ERROR("Something went wrong when calling pose estimation service");
   	  return false;
    }
    
   for(size_t k = 0; k<= eMsg.response.detected_obj_id.size()-1; k++)
   {
         geometry_msgs::Transform t = eMsg.response.poses.back();

	 ROS_INFO("===================== %s model detected with id %s =================",eMsg.response.detected_obj_name[k].c_str(), eMsg.response.detected_obj_id[k].c_str());
	 ROS_INFO("Error: %f",eMsg.response.error[k]);
	 ROS_INFO("Inlier Fraction: %f",eMsg.response.inlier_fraction[k]);
	 ROS_INFO("Object Name: %s",eMsg.response.detected_obj_name[k].c_str());
	 ROS_INFO("Object Id: %s",eMsg.response.detected_obj_id[k].c_str());
	 ROS_INFO("Pose:\n\t X: %f Y: %f Z: %f",t.translation.x,t.translation.y,t.translation.z);
	 ROS_INFO("\n\t w: %f x: %f y: %f z: %f",t.rotation.w,t.rotation.x,t.rotation.y, t.rotation.z);
	 
	 EstimationResult res;
	 res.setError(eMsg.response.error[k]);
	 res.setInlierFraction(eMsg.response.inlier_fraction[k]);
	 res.setName(QString::fromStdString(eMsg.response.detected_obj_name[k]));
	 res.setId(QString::fromStdString(eMsg.response.detected_obj_id[k]));
	 res.setPose(t);
	 
	 result.push_back(res);


   }
  
  return true;
}
}
}