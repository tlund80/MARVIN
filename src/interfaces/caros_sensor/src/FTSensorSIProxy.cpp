/*
 * FTSensorSIProxy.cpp
 *
 *  Created on: 15/05/2013
 *      Author: thomas
 */

#include <caros/common.hpp>

#include <caros/FTSensorSIProxy.hpp>

using namespace caros;

FTSensorSIProxy::FTSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle):
	_nodeHnd(nhandle)
{
	_ftState = _nodeHnd->subscribe("wrench", 1, &FTSensorSIProxy::handleFTState, this);
}

FTSensorSIProxy::FTSensorSIProxy(const std::string& name):
    _nodeHnd( rw::common::ownedPtr( new ros::NodeHandle(name)))
{
    _ftState = _nodeHnd->subscribe("wrench", 1, &FTSensorSIProxy::handleFTState, this);
}

FTSensorSIProxy::~FTSensorSIProxy() {
}

void FTSensorSIProxy::handleFTState(const geometry_msgs::WrenchStamped& state) {
	boost::mutex::scoped_lock lock(_mutex);
	_wrench = caros::toRw(state.wrench);
	_pFTState = state;
}

rw::math::Wrench6D<> FTSensorSIProxy::getWrench() {
	boost::mutex::scoped_lock lock(_mutex);
	return _wrench;
}

ros::Time FTSensorSIProxy::getTimeStamp() {
	boost::mutex::scoped_lock lock(_mutex);
	return _pFTState.header.stamp;
}
