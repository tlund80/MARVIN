#pragma once

//#include "../RWInterface.hpp"

#include <one_shot_learning/grasp_planner/Workcell.hpp>
#include <rw/models/Device.hpp>
#include <rwlibs/task/Task.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QTimer>
#include <QObject>

using namespace rw::models;
using namespace rwlibs::task;

/**
* \brief Brief description of class.
*
* Longer description of class.
*/

namespace dti{
namespace one_shot_learning {
namespace motion_planner {

class PlayBack : public QObject
{
	Q_OBJECT

private:
	//CartesianMotion::Ptr _motion;
	//RWInterface *_rwInterface;
	//boost::shared_ptr<dti::grasp_planning::Workcell> _rwInterface;
	rw::models::WorkCell::Ptr _rwInterface;
	rws::RobWorkStudio* _rws;
	Device::Ptr _robot;
	QTimer _timer;
	int _direction;
	int _tickInterval;
	rw::trajectory::QPath _path;
	int _indx;
	int _low;
	int _high;

public:
	//PlayBack(boost::shared_ptr<dti::grasp_planning::Workcell> rwInterface, int tickInterval);
	PlayBack(rw::models::WorkCell::Ptr rwInterface, int tickInterval);
	~PlayBack();

	//void setMotion(CartesianMotion::Ptr motion);
	void setMotion(rw::trajectory::Path<Q> path);
	void forward();
	void backward();
	void pause();
	void goto_start();
	void goto_end();

	void setDevice(Device::Ptr robot) { _robot = robot; }
	void setRobworkStudio(rws::RobWorkStudio* rws){_rws = rws; };


private slots:
	void tick();

private:
	void startTimer();
	void draw(rw::math::Q &q)
	{
		rw::kinematics::State state = _rwInterface->getDefaultState();
		_robot->setQ(q, state);
		if(!_rws) RW_THROW("PlayBack: RobWorkStudio Instance is NULL!");
		
		_rws->setState(state);
		//
	}
};

}
}
}
