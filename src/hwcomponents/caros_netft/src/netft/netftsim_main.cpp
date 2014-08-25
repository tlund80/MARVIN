/* */
/**
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include "ros/ros.h"
#include "marvin_common/WrenchData.h"
#include "marvin_common/TimeRequest.h"
#include "marvin_common/MarvinUtils.hpp"
#include <rw/common/PropertyMap.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>


#include <iostream>

using namespace rw::common;
using namespace rw::loaders;


MarvinTime marvinTime;
//ros::Duration syncTimeOffset;

/*void timeSignal(const common::TimeSignal::ConstPtr& time) {
	syncTimeOffset = time->timestamp - ros::Time::now() ;
	std::cout<<"Time Offset = "<<syncTimeOffset.toSec()<<std::endl;
}*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "netft");
	ros::NodeHandle nh;

	if (argc==1)
	{
		std::cerr << "usage : " << argv[0] << " propertyfile" << std::endl;
		return -1;
	}

	try {
		PropertyMap properties = XMLPropertyLoader::load(argv[1]);

		int publishRate = properties.get<int>("PublishRate");


		ros::Publisher pub = nh.advertise<marvin_common::WrenchData>(Marvin::FT_ID, 1000);
        ros::Rate pub_rate(publishRate);

        marvinTime.initialize(nh);

        //ros::Duration driverTimeOffset = ros::Time::now() - ros::Time(netft.driverTime());
        marvinTime.setDriverTime(ros::Time::now().toSec());
        while (ros::ok())
        {
			marvin_common::WrenchData wdata;
        	wdata.header.frame_id = Marvin::FT_ID;
        	wdata.header.stamp = marvinTime.convertDriverToMarvinTime(ros::Time::now().toSec());//ros::Time(data.timestamp + driverTimeOffset.toSec()) + syncTimeOffset;
        	pub.publish(wdata);
        	ros::spinOnce();
        	pub_rate.sleep();
        }

	} catch (const Exception& exp) {
		std::cout<<"Exception = "<<exp.what()<<std::endl;
	}

	return 0;
}
