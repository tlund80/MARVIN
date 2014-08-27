/**/
#include <caros/PoseSensorSIProxy.hpp>

#include <fstream>
#include <rw/common/Ptr.hpp>
#include <boost/foreach.hpp>

#include <caros/common.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace std;

using namespace caros;

PoseSensorSIProxy::PoseSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle):
		_nodeHnd(nhandle)
{
	_poseSensorState = _nodeHnd->subscribe("poses", 1, &PoseSensorSIProxy::handlePoseSensorState, this);
}

PoseSensorSIProxy::PoseSensorSIProxy(const std::string& devname):
        _nodeHnd( rw::common::ownedPtr(new ros::NodeHandle(devname)) )
{
    _poseSensorState = _nodeHnd->subscribe(devname + "/poses", 1, &PoseSensorSIProxy::handlePoseSensorState, this);
}

PoseSensorSIProxy::~PoseSensorSIProxy() {
}

void PoseSensorSIProxy::handlePoseSensorState(const caros_sensor_msgs::PoseSensorState& state)
{
	boost::mutex::scoped_lock lock(_mutex);
	_poses.resize(state.poses.size());
	_stamp = state.header.stamp;
	for(int i=0;i<int(state.poses.size());i++){
		PoseData &pdata = _poses[i];
		pdata.pose = caros::toRw(state.poses[i]);
		pdata.id = state.ids[i];
		pdata.quality = state.qualities[i];
		pdata.stamp = state.header.stamp;
		pdata.frame = state.header.frame_id;
	}
}


std::vector<PoseSensorSIProxy::PoseData> PoseSensorSIProxy::getPoses() {
	boost::mutex::scoped_lock lock(_mutex);
	return _poses;
}

ros::Time PoseSensorSIProxy::getTimeStamp() {
	boost::mutex::scoped_lock lock(_mutex);
	return _stamp;
}

