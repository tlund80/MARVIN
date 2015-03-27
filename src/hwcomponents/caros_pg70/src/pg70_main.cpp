#include <caros/PG70Node.hpp>
#include <ros/ros.h>



int main(int argc, char **argv)
{
	ROS_INFO("Starting caros_pg70 node!");
	ros::init(argc, argv, "caros_pg70");
	ros::NodeHandle nh("~");
	try {
	
		PG70Node pg70(nh);
		pg70.start();
	} catch (const rw::common::Exception& exp) {
		std::cout<<"Exception = "<<exp.what()<<std::endl;
	}
    return 0;
}

