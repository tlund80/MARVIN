/* */
/**
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include "ros/ros.h"
#include "marvin_common/WrenchData.h"
#include "marvin_common/TimeRequest.h"
#include "marvin_common/MarvinUtils.hpp"
#include "marvin_common_rw/RwRos.hpp"
#include <rw/common/PropertyMap.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>


#include <rwhw/netft/NetFTLogging.hpp>
#include <rwhw/netft/FTCompensation.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <rw/models/WorkCell.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rwhw;
using namespace rw::models;
using namespace rw::math;

rwhw::FTCompensation *_ftCompensation;
std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > _wrench;

Wrench3D _ft_current;
ros::Time _lastTime;
void handleRobotState(const marvin_common::RobotState& state){
	//
	Q q = RwRos::toRw(state.q);
	Q dq = RwRos::toRw(state.dq);
	ros::Time _currentTime = ros::Time::now();
    _ftCompensation->update(_wrench, q, dq, (_currentTime-_lastTime).toSec() );
    _lastTime = _currentTime;
}

// initialize the ros stuff for publishing raw wrench data
void initWrenchRaw();
// initialize the ros stuff for publishing unbiased wrench data
void initWrenchUnbiased();

int main(int argc, char **argv)
{
	// TODO: add boost options here if we want to parse command line options
	// for now we use parameter server

	ros::init(argc, argv, "netft");
	ros::NodeHandle nh("~");
	std::string name = nh.getNamespace();
	std::string propertyFile;
<<<<<<< .mine
	std::string host, robotName, robotState;
=======
	std::string host, devName, calibfile;

>>>>>>> .r1128
	int publishRate = 400;
	nh.getParam("properties", propertyFile);
	nh.param("host", host, std::string(""));
	nh.param("rate", publishRate, 400);
<<<<<<< .mine
	nh.param("robot", robotName, "");
	nh.param("robotState", robotState, "");

	bool unbiasedWrenchEnabled = false;


	// try and load the workcell, this will determine if the calibrated wrench is published or not
    WorkCell::Ptr workcell;
    Device::Ptr dev;
    try {
        workcell = RwRos::getWorkCell();
        dev = workcell->findDevice(robotName);
        if (dev == NULL) {
            ROS_INFO_STREAM("Unable to find device "<<robotName<<" in work cell");
        }

        // todo allso
        unbiasedWrenchEnabled = true;
=======
	nh.getParam("robot", devName);
	nh.getParam("calibfile",calibfile);
>>>>>>> .r1128


<<<<<<< .mine
    } catch (const Exception& exp) {
        ROS_WARN_STREAM("Unable to open file " << exp.what());
        ROS_WARN("Unbiased wrench data will not be available!");
    }

    ROS_INFO("WorkCell loaded!");
    ROS_INFO("WorkCell loaded! Starting ur component.");



=======
    // get the workcell
	WorkCell::Ptr _pWorkCell = RwRos::getWorkCell();
	if(_pWorkCell==NULL){
		ROS_ERROR("No workcell added to the parameter server!");
		ROS_BREAK();
	}

	// device
	Device::Ptr _dev = _pWorkCell->findDevice(devName);
	rw::kinematics::State tmpState = _pWorkCell->getDefaultState();
	if (_dev == NULL) {
		ROS_ERROR_STREAM("No device by name " << devName << " Possible devices are:");
		BOOST_FOREACH(rw::models::Device::Ptr dev, _pWorkCell->getDevices()){
			ROS_ERROR_STREAM("dev: " << dev->getName() );
		}
		ROS_BREAK();
	}

	_ftCompensation = new rwhw::FTCompensation(_dev, _pWorkCell->getDefaultState(), calibfile);
	_lastTime = ros::Time::now();
	ros::Subscriber robotState = nh.subscribe("/" + devName + "/RobotState", 1, &handleRobotState);
>>>>>>> .r1128

	try {

		if(!propertyFile.empty()){
			PropertyMap properties = XMLPropertyLoader::load(propertyFile);
			host = properties.get<std::string>("Host","");
			publishRate = properties.get<int>("PublishRate", 400);
		}

		// if there is no host then the sensor cannot start
		if(host.empty()){
			ROS_ERROR("No valid host defined for netft sensor: ");
			ROS_BREAK();
		}

		NetFTLogging netft(host);
		netft.start();

		ros::Publisher pub = nh.advertise<marvin_common::WrenchData>(name + "/WrenchData_raw", 10);
		ros::Publisher pub_calib = nh.advertise<marvin_common::WrenchData>(name + "/WrenchData", 10);

		ros::Rate pub_rate(publishRate);

//        marvinTime.initialize(nh);

        //ros::Duration driverTimeOffset = ros::Time::now() - ros::Time(netft.driverTime());
       // marvinTime.setDriverTime(netft.driverTime());
		int cnt = 0;
		std::vector<Wrench3D> biases;
        while (ros::ok())
        {
        	NetFTLogging::NetFTData data = netft.getAllData();
        	marvin_common::WrenchData wdata, wdata_calib;
        	wdata.header.frame_id = Marvin::FT_ID;
        	wdata.header.stamp = ros::Time::now();

        	wdata_calib.header.frame_id = Marvin::FT_ID;
        	wdata_calib.header.stamp = ros::Time::now();

        //	wdata.timestamp = marvinTime.convertDriverToMarvinTime(data.timestamp);//ros::Time(data.timestamp + driverTimeOffset.toSec()) + syncTimeOffset;

        	Wrench3D ft = data.data;
        	_ft_current = ft;
        	wdata.wrench.force.x = ft.first[0];
        	wdata.wrench.force.y = ft.first[1];
        	wdata.wrench.force.z = ft.first[2];

        	wdata.wrench.torque.x = ft.second[0];
        	wdata.wrench.torque.y = ft.second[1];
        	wdata.wrench.torque.z = ft.second[2];

        	ft = _ftCompensation->getFT();
        	wdata_calib.wrench.force.x = ft.first[0];
        	wdata_calib.wrench.force.y = ft.first[1];
        	wdata_calib.wrench.force.z = ft.first[2];

        	wdata_calib.wrench.torque.x = ft.second[0];
        	wdata_calib.wrench.torque.y = ft.second[1];
        	wdata_calib.wrench.torque.z = ft.second[2];
        	cnt++;
        	if( cnt > 2000  && cnt < 2100){
        		biases.push_back(ft);
        	}
        	if(cnt==2101){
        		ROS_INFO("BIASING");
        		_ftCompensation->updateBias(biases);
        	}

        	pub.publish(wdata);
        	pub_calib.publish( wdata_calib );
        	ros::spinOnce();
        	pub_rate.sleep();
        }

	} catch (const Exception& exp) {
		std::cout<<"Exception = "<<exp.what()<<std::endl;
	}

	return 0;
}
