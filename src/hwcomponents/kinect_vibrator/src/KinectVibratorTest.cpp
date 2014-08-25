#include "KinectVibrator.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

int main(int argc, char **argv)
{
  double loopRate=10;
  const std::string name("kinect_vibrator");

  ros::init(argc, argv, "kinect_vibrator");
  ros::NodeHandle n;
  KinectVibrator camTrig(name, loopRate);
  kinect_vibrator::StartRequest req;
  kinect_vibrator::StartResponse res;
  camTrig.start(req,res);
  camTrig.run();



  return 0;
}

