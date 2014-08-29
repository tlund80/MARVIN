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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

   
#include <std_msgs/String.h>

#include <sstream>
#include <string>
#include <QString>
#include <QThread>
#include <QStringListModel>

//Robwork
#include <rw/math/Transform3D.hpp>

//Caros
#include <caros_control_msgs/RobotState.h>
#include <caros_control_msgs/GripperState.h>

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
	void StartSDHSubscriber(); 
	bool MoveRobotLinear(std::vector<rw::math::Transform3D<double> >& path, std::vector<float>& blends, float speed);
	bool MoveRobotJoint(std::vector<rw::math::Q>& q, std::vector<float>& blends, float speed);
	bool StopRobot();
	bool SDHStop();
	bool SDHPause();
	bool SDHGrasp(rw::math::Q &q);
	bool SDHMoveQ(rw::math::Q &q);
	bool InitializeSDH(void);
	
	bool PreparePoseEstimation(std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > model, std::vector<QString> name, std::vector<std::string> &id_vec);
        bool EstimationPose(std::vector<std::string> id_vec, std::vector<EstimationResult> &result);
	std::string getAbsoluteNodePath();
Q_SIGNALS:
	//void updateImage(const QImage&, int);
	//void updateTCP(const tiv::pose&, int);
	//void updateImageLeft(const QImage&);
	//void updateImageRight(const QImage&);
	void consoleSignal(QString msg);
	void robotPose(rw::math::Q q, QString robot_name);
	void gipperconfiguration(rw::math::Q q, QString gripper_name);
	void rosShutdown();
	void finish();

private:
	SharedData 				*_sharedData;
	ros::Subscriber 			sub_pointCloud;
	ros::Subscriber 			_sub_robotState;
	ros::Subscriber 			_sub_sdhState;
	
	ros::ServiceClient 			_moveL_srv;
	ros::ServiceClient 			_moveJ_srv;
	ros::ServiceClient 			_stop_srv;
	ros::ServiceClient 			_SDH_stop_srv;
	ros::ServiceClient 			_SDH_pause_srv;
	ros::ServiceClient 			_SDH_grasp_srv;
	ros::ServiceClient 			_SDH_move_srv;
	//std::string				_ur1_ns;
	//std::string				_ur2_ns;
	
	void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& input); 
	void robotCallBack(const caros_control_msgs::RobotStateConstPtr& msg); 
	void sdhCallBack(const caros_control_msgs::GripperStateConstPtr& msg); 
	
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	//dti::SharedData *_sharedData;
};

}
}  // namespace object_modeller_gui

//#endif /* OBJECTMODELLERGUI_ROSINTERFACE_HPP_ */