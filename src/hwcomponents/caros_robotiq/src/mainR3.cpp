#include "ros/ros.h"

#include <caros/Robotiq3Node.hpp>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "robotiq3");

   ros::NodeHandle nh("~");
   std::string name = nh.getNamespace();
   Robotiq3Node hand(name);
   hand.start();

   return 0;
}
