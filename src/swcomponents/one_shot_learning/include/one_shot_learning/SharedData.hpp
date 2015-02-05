/*
 * SharedData.h
 *
 *  Created on: Nov 11, 2013
 *      Author: Thomas Sølund
 */

#ifndef SHAREDDATA_H_
#define SHAREDDATA_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>

#include <rw/math/Q.hpp>

#include <cmath>
#include <vector>

#include <QObject>
#include <QThread>
#include <QMutex>

namespace dti{
namespace one_shot_learning {

class SharedData  : public QObject{
    Q_OBJECT
private:

	QMutex 					_mutexPointCloud;
	QMutex 					_mutexrobotQ;
	QMutex 					_mutexrobotVel;
	QMutex					_mutexisMoving;
	QMutex 					_mutexTurnModelFlag;
	
	pcl::PointCloud<pcl::PointXYZRGBA>	_cloud;
	rw::math::Q 				_robotQ;
	rw::math::Q 				_jointVel;
	bool 					_isMoving;
	
	QThread 				*_gui_instance;

Q_SIGNALS:
	void consoleSignal(QString msg);
	void modelCreated();
	void solidModelCreated();
	void turnModel();
	void finished();

public:
	SharedData() {
	
	}

	~SharedData() {
		
	}

	
	void setFlagTurnModel(void){
	  QMutexLocker locker(&_mutexTurnModelFlag);
	   std::cout << "Turn model flag set!" << std::endl;
	   std::cout << "SharedData thrad: " << QThread::currentThreadId() << std::endl;   
	 //  if(_gui_instance != this->thread()){
	       std::cout << "Hello!" << std::endl;
	 
	  // }
	    std::cout << "SharedData thrad: " << QThread::currentThreadId() << std::endl;   
	  Q_EMIT modelCreated();
	}
	
	void setPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloud){
	  QMutexLocker locker(&_mutexPointCloud);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _cloud = cloud;
	}	
	
	void getPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> &cloud){
	  QMutexLocker locker(&_mutexPointCloud);
	  cloud =  _cloud;
	}
	
	void setRobotQ(rw::math::Q q){
	  QMutexLocker locker(&_mutexrobotQ);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _robotQ = q;
	}
	
	void getRobotQ(rw::math::Q &q){
	  QMutexLocker locker(&_mutexrobotQ);
	  _robotQ =  q;
	}
	
	void setRobotJointVelocity(rw::math::Q vel){
	  QMutexLocker locker(&_mutexrobotVel);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _jointVel = vel;
	}
		
	void getRobotJointVelocity(rw::math::Q &vel){
	  QMutexLocker locker(&_mutexrobotVel);
	  _jointVel =  vel;
	}
		
	void setRobotMoving(bool isMoving){
	  QMutexLocker locker(&_mutexisMoving);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _isMoving = isMoving;
	}
		
	bool getIsRobotMoving(void){
	  QMutexLocker locker(&_mutexisMoving);
	  return _isMoving;
	}
};

}
} /* namespace dti */
#endif /* SHAREDDATA_H_ */
