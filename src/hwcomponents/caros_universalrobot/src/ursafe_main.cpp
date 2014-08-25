#include "UniversalRobots.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <boost/bind.hpp>




using namespace rwhw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::kinematics;


int main(int argc, char **argv)
{
	if (argc <= 2) {
		std::cout<<"Usage: ursafe <workcell> <propertyfile.prop.xml>"<<std::endl;
		return 0;
	}

	WorkCell::Ptr workcell = WorkCellLoader::load(argv[1]);
	if (workcell == NULL) {
		std::cout<<"Unable to load workcell: "<<argv[1]<<std::endl;
		return 0;
	}


	PropertyMap properties = XMLPropertyLoader::load(argv[2]);

	if (!properties.has("Name")) {
		std::cout<<"No property named 'Name' found in property file"<<std::endl;
		return 0;
	}

	std::string deviceName = properties.get<std::string>("Name");
	Device::Ptr dev = workcell->findDevice(deviceName);
	if (dev == NULL) {
		std::cout<<"Unable to find device "<<deviceName<<" in work cell"<<std::endl;
		return 0;
	}


	ros::init(argc, argv, properties.get<std::string>("Name").c_str());


	std::string host = properties.get<std::string>("NetFTHost");
	int updateRate = properties.get<int>("NetFTUpdateRate");

	std::string calibfile = properties.get<std::string>("NetFTCalibrationFile");
	std::cout<<"Calibration File = "<<calibfile<<std::endl;
	NetFTLogging netft(host);

	FTCompensation ftCompensation(dev, workcell->getDefaultState(), calibfile);

    UniversalRobots ur(workcell, properties, updateRate, &netft, &ftCompensation);

    ur.run();

    
//  while (ros::ok())
//  {
//    /**
//     * This is a message object. You stuff it with data, and then publish it.
//     */
//    sensor_data msg;
//
//    std::stringstream ss;
//    ss << "hello";// << count;
//    msg.name = ss.str();
//
//    ROS_INFO("%s", msg.name.c_str());
//
//    /**
//     * The publish() function is how you send messages. The parameter
//     * is the message object. The type of this object must agree with the type
//     * given as a template parameter to the advertise<>() call, as was done
//     * in the constructor above.
//     */
//    chatter_pub.publish(msg);
//
//    ros::spinOnce();
//
//    loop_rate.sleep();
//    ++count;
//  }



    return 0;
}

