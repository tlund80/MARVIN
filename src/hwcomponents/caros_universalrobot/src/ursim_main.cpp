#include "UniversalRobotsSim.hpp"
#include <boost/bind.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::common;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::kinematics;

int main(int argc, char **argv)
{

	if (argc <= 2) {
		std::cout<<"Usage: ./ursim nodename workcellfile.xml"<<std::endl;
		return 0;

	}

	ros::init(argc, argv, argv[1]);

	WorkCell::Ptr workcell;
	try {
		workcell = WorkCellLoader::load(argv[2]);

	} catch (const Exception& exp) {
		std::cout<<"Unable to open file '"<<argv[1]<<"'"<<std::endl;
		std::cout<<"Error: "<<exp.what()<<std::endl;
		return 0;
	}

	if (workcell == NULL) {
		std::cout<<"Unable to open file '"<<argv[1]<<"'"<<std::endl;
		return 0;
	}

	SerialDevice::Ptr sdevice = workcell->findDevice<SerialDevice>("UR1");

    UniversalRobotsSim ur(argc, argv, argv[1], sdevice, workcell->getDefaultState());



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

