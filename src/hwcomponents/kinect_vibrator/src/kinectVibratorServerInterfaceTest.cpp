#include "KinectVibrator.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  char c;
  ros::init(argc, argv, "kinectVibratorServiceInterfaseTest");
  ros::NodeHandle n;
  ros::ServiceClient clientStart = n.serviceClient<kinect_vibrator::Start>("/StartTrigger");
  kinect_vibrator::Start srvStart;
  ros::ServiceClient clientStop = n.serviceClient<kinect_vibrator::Start>("/StopTrigger");
  kinect_vibrator::Stop srvStop;
  ROS_INFO("Press 'r' to run motors.");
  ROS_INFO("Press 's' to stop motors.");
  ROS_INFO("Press 'q' to quit test.");
  
  while (1) {
     c=getchar();
     if (c == 'r') {
         if (clientStart.call(srvStart))
         {
            ROS_INFO("Started");
         }
         else
         {
             ROS_ERROR("Failed to call service Start");
             return 1;
         }
    }
    if (c == 's') {
         if (clientStop.call(srvStop))
         {
            ROS_INFO("Stopped");
         }
         else
         {
             ROS_ERROR("Failed to call service Stop");
             return 1;
         }
    } 
    if (c == 'q') {
             break;
        }
  }

  return 0;

}
