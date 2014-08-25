#include <caros/SDHNode.hpp>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sdh");

    ros::NodeHandle nh("~");
    
    SDHNode sdhNode(nh);

    sdhNode.start();

    return 0;
}
