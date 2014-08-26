/*
 * SharedData.h
 *
 *  Created on: Nov 11, 2013
 *      Author: Thomas Sølund
 */

#ifndef SHAREDDATA_H_
#define SHAREDDATA_H_

#include <pcl/point_types.h>

#include <rw/math/Q.hpp>

#include <QMutex>
#include <QDir>
#include <cmath>
#include <vector>

namespace dti{
namespace one_shot_learning {

class SharedData {

private:

	// Storage path for rectifiec imgs
	

	QMutex 				_mutexPointCloud;
	QMutex 				_mutexrobotQ;
	QMutex 				_mutexrobotVel;
	QMutex				_mutexisMoving;
	/*QMutex 				_mutexImgL, _mutexImgR;
	QMutex				_mutexBoardSize;
	QMutex				_mutexSquareSize;
	QMutex				_mutexRobotPose;
	QMutex				_mutexRobotPoseArray;
	QMutex				_mutexRobotPath;
	QMutex				_mutexRobotLivePath;*/
	
	pcl::PointCloud<pcl::PointXYZRGBA> _cloud;
	rw::math::Q _robotQ;
	rw::math::Q _jointVel;
	bool _isMoving;

public:
	SharedData() {

	}

	~SharedData() {
		
	}

	
	void setPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloud)
	{
	  QMutexLocker locker(&_mutexPointCloud);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _cloud = cloud;
	}
	
	
	void getPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
	{
	  QMutexLocker locker(&_mutexPointCloud);
	  cloud =  _cloud;
	}
	
	void setRobotQ(rw::math::Q q)
	{
	  QMutexLocker locker(&_mutexrobotQ);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _robotQ = q;
	}
	
	
	void getRobotQ(rw::math::Q &q)
	{
	  QMutexLocker locker(&_mutexrobotQ);
	  _robotQ =  q;
	}
	
	void setRobotJointVelocity(rw::math::Q vel)
	{
	  QMutexLocker locker(&_mutexrobotVel);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _jointVel = vel;
	}
	
	
	void getRobotJointVelocity(rw::math::Q &vel)
	{
	  QMutexLocker locker(&_mutexrobotVel);
	  _jointVel =  vel;
	}
	
	
	void setRobotMoving(bool isMoving)
	{
	  QMutexLocker locker(&_mutexisMoving);
	 // std::cout << "set" << std::endl;
	  //std::cout << "cloud size" << (int)cloud.size() << std::endl;
	  _isMoving = isMoving;
	}
	
	
	bool getIsRobotMoving(void)
	{
	  QMutexLocker locker(&_mutexrobotVel);
	  return _isMoving;
	}
};

}
} /* namespace dti */
#endif /* SHAREDDATA_H_ */
