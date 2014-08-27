/**
 * @file /include/object_modeller_gui/rosInterface.hpp
 *
 * @brief Communications central!
 *
 * @date November 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

//#ifndef OBJECTMODELLERGUI_ROSINTERFACE_HPP_
//#define OBJECTMODELLERGUI_ROSINTERFACE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
   
#include <std_msgs/String.h>

#include <sstream>
#include <string>
#include <QThread>
#include <QStringListModel>

//Robwork
#include <rw/math/Transform3D.hpp>


//Marvin
//#include <marvin_common/URMoveL.h>
//#include <marvin_common/URMoveQ.h>
//#include <marvin_common/URStop.h>
//#include <marvin_common_rw/RwRos.hpp>
//#include <marvin_common/RobotState.h>

#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePTP.h>
#include <caros_common_msgs/Stop.h>
#include <caros_common_msgs/Pause.h>
#include <caros/common.hpp>
#include <caros_control_msgs/RobotState.h>



//SDH
#include <caros_control_msgs/GripperMoveQ.h>
#include <caros_control_msgs/GripperGripQ.h>
//#include <marvin_common/SDHStop.h>
//#include <marvin_common/SDHPause.h>

// Services
#include <pose_estimation_covis/estimate.h>
#include <pose_estimation_covis//prepareEstimation.h>

#include <one_shot_learning/SharedData.hpp>
#include <one_shot_learning/EstimationResult.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace dti{
namespace one_shot_learning {

/*****************************************************************************
** Class
*****************************************************************************/

class RosCommunication : public QThread {
    Q_OBJECT
public:
	RosCommunication(SharedData *data);
	virtual ~RosCommunication();
	bool init();
	void run();
	void StartPPSubscriber(); 
	void StartRobotSubscriber();
	bool MoveRobotLinear(std::vector<rw::math::Transform3D<double> >& path, std::vector<float>& blends, float speed);
	bool MoveRobotJoint(std::vector<rw::math::Q>& q, std::vector<float>& blends, float speed);
	bool StopRobot();
	bool SDHStop();
	bool SDHPause();
	bool SDHGrasp(rw::math::Q &q);
	bool SDHMoveQ(rw::math::Q &q);
	
	bool PreparePoseEstimation(std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > model, std::vector<QString> name, std::vector<std::string> &id_vec);
        bool EstimationPose(std::vector<std::string> id_vec, std::vector<EstimationResult> &result);
	std::string getAbsoluteNodePath();
Q_SIGNALS:
	//void updateImage(const QImage&, int);
	//void updateTCP(const tiv::pose&, int);
	//void updateImageLeft(const QImage&);
	//void updateImageRight(const QImage&);
	void consoleSignal(QString msg);
	void robotPose(rw::math::Q q);
	void rosShutdown();
	void finish();

private:
	SharedData *_sharedData;
	ros::Subscriber sub_pointCloud;
	ros::Subscriber _sub_robotState;
	
	ros::ServiceClient _moveL_srv;
	ros::ServiceClient _moveJ_srv;
	ros::ServiceClient _stop_srv;
	ros::ServiceClient _SDH_stop_srv;
	ros::ServiceClient _SDH_pause_srv;
	ros::ServiceClient _SDH_grasp_srv;
	ros::ServiceClient _SDH_move_srv;
	
	void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input); 
	void robotCallBack(const caros_control_msgs::RobotStateConstPtr& msg); 
	
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	//dti::SharedData *_sharedData;
};

}
}  // namespace object_modeller_gui

//#endif /* OBJECTMODELLERGUI_ROSINTERFACE_HPP_ */