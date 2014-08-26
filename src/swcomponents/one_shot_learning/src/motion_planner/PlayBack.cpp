#include <one_shot_learning/motion_planner/PlayBack.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::trajectory;

namespace dti{
namespace one_shot_learning {
namespace motion_planner {

PlayBack::PlayBack(rw::models::WorkCell::Ptr rwInterface, int tickInterval)
: //_motion(NULL),
  _timer(this),
  _tickInterval(tickInterval),
  _rwInterface(rwInterface)
{
	connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));
}

PlayBack::~PlayBack()
{}

/*void PlayBack::setMotion(CartesianMotion::Ptr motion)
{
    if (_timer.isActive())
        _timer.stop();
	_motion = motion;
	_indx = 0;
	if(_motion != NULL && _motion->getPropertyMap().has("Path"))
	{
		_path = _motion->getPropertyMap().get<QPath>("Path");
		_low = 0;
		_high = _path.size() - 1;
		draw(_path[0]);
	}
	else
	{
		_low = 1;
		_high = -1;
		Q _q = Q::zero(_robot->getDOF());
		draw(_q);
	}
}
*/

void PlayBack::setMotion(rw::trajectory::Path<Q> path)
{
    if (_timer.isActive()) _timer.stop();
    
	_path = path;
	_indx = 0;
    
	_low = 0;
	_high = _path.size() - 1;
	draw(_path[0]);
}

void PlayBack::forward()
{
	_direction = 1;
	startTimer();
}

void PlayBack::backward()
{
	_direction = -1;
	startTimer();
}

void PlayBack::pause()
{
    if (_timer.isActive())
        _timer.stop();
    else
        startTimer();
}

/*void PlayBack::goto_start()
{
	if(_motion != NULL && _motion->getPropertyMap().has("Path"))
	{
		_timer.stop();
		_indx = _low;
		draw(*_path.begin());
	}
}
*/
void PlayBack::goto_start()
{
  _timer.stop();
  _indx = _low;
  draw(*_path.begin());
}

/*void PlayBack::goto_end()
{
	if(_motion != NULL && _motion->getPropertyMap().has("Path"))
	{
		_timer.stop();
		_indx = _high;
		draw(*_path.rbegin());
	}
}
*/
void PlayBack::goto_end()
{
  _timer.stop();
  _indx = _high;
  draw(*_path.rbegin());
}

void PlayBack::tick()
{
	if(_low <= _indx && _indx <= _high)
	{
		draw(_path[_indx]);
		_indx += _direction;
	}
	else
	{
		if(_indx < _low)
			_indx = _low;
		if(_indx > _high)
			_indx = _high;
		_timer.stop();
	}
}

void PlayBack::startTimer()
{
	if(!_timer.isActive())
	{
		_timer.start(_tickInterval);
	}
}

}
}
}
