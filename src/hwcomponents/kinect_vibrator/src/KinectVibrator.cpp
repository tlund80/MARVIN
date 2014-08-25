// Copyright (c) 2010, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 \author Thiusius Rajeeth Savarimuthu
 \file KinectVibrator.cpp
 \date 15/05/2013
 \This is a class that is used to control the hardware triggering of the pike and bumblebee cameras.
*/


#include "KinectVibrator.hpp"



KinectVibrator::KinectVibrator(const std::string& service_name,
 double loopRate):
    _service_name(service_name),
    _loopRate(loopRate)
{
	_dataH=0xFF;
	_dataL=0x00;
    _chatter_pub = _nodeHnd.advertise<std_msgs::Int32>("/triggerMsg", 1);
    _srvFrameCount = _nodeHnd.advertiseService("/Current_frame_number", &KinectVibrator::getFrameCount, this);
    _srvStart = _nodeHnd.advertiseService("/StartTrigger", &KinectVibrator::start, this);
    _srvStop = _nodeHnd.advertiseService("/StopTrigger", &KinectVibrator::stop, this);
    _srvReset = _nodeHnd.advertiseService("/ResetTrigger", &KinectVibrator::reset, this);
    _count=0;
    _runTrigger=false;
    init();
    ROS_INFO("KinectVibrator initialised");
    

}

KinectVibrator::~KinectVibrator(){

    ioctl(_parportfd,PPRELEASE);
    close(_parportfd);
    ROS_INFO("Parallelport closed");
}

int KinectVibrator::init(){
    ROS_INFO("Opening parallelport");
    int result=0;
    _parportfd = open("/dev/parport0", O_RDWR);
    if (_parportfd== -1)
    {
      ROS_INFO("Could not open parallel port");
      return false;
    }
    if (ioctl(_parportfd,PPCLAIM) != 0) ROS_INFO("Parallelport could not be claimed");

// Set the Mode
   int mode = IEEE1284_MODE_COMPAT;
   if (ioctl(_parportfd, PPNEGOT, &mode) == -1)
   {
      perror("Could not set mode");
      ioctl(_parportfd, PPRELEASE);
      close(_parportfd);
      return 1;
   }

   // Set data pins to output
   int dir = 0x00;
   if (ioctl(_parportfd, PPDATADIR, &dir))
   {
      perror("Could not set parallel port direction");
      ioctl(_parportfd, PPRELEASE);
      close(_parportfd);
      return 1;
   }
   ROS_INFO("Parallelport ready..");
   return 0;
}

bool KinectVibrator::getFrameCount(kinect_vibrator::FrameCountRequest  &req,
         kinect_vibrator::FrameCountResponse &res )
{
  res.frameCount = _count;
  return true;
}

bool KinectVibrator::stop(kinect_vibrator::StopRequest &req,
         kinect_vibrator::StopResponse &res){

	ioctl(_parportfd, PPWDATA,&_dataL);
    _runTrigger=false;
    return true;
}

bool KinectVibrator::start(kinect_vibrator::StartRequest &req,
         kinect_vibrator::StartResponse &res){
	int status=0;

    _runTrigger=true;
    status=ioctl(_parportfd, PPWDATA,&_dataH);
     ROS_INFO("KinectVibrator started %i", status);
    return true;
}
bool KinectVibrator::reset(kinect_vibrator::ResetRequest &req,
         kinect_vibrator::ResetResponse &res){
    _count=0;
    return true;
}

void KinectVibrator::run() {

    while (ros::ok())
    {
	if(_runTrigger){
            loop();
        }
        ros::spinOnce();
        _loopRate.sleep();
    }

}

void KinectVibrator::loop(){
    int status=0;
    unsigned char _dataH=0xFF;
    unsigned char _dataL=0x00;

    _msg.data=_count;
    ROS_INFO("%i", _msg.data);
   // status=ioctl(_parportfd, PPWDATA,&_dataH);
    usleep(100);
   // ioctl(_parportfd, PPWDATA,&_dataL);

    _chatter_pub.publish(_msg);
    _count++;
}


/*
int main(int argc, char **argv)
{

   ros::init(argc, argv, "KinectVibrator");
   ros::NodeHandle n;

   ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("/chatter", 1000);
   ros::ServiceServer service = n.advertiseService("/current_frame_number", getFrameCount);
 
   ros::Rate loop_rate(10);
   //set permissions to access port
   if (ioperm(BASEPORT, 3, 1)) {perror("ioperm"); exit(1);}
   int tem = fcntl(0, F_GETFL, 0);
      fcntl (0, F_SETFL, (tem | O_NDELAY));
	
   std_msgs::Int32 msg;
   count=0;
   while (ros::ok())
   {

  
      //std::stringstream ss;
      //ss << "frame id_" << count;
//      msg.data = ss.str();
      msg.data=count;
      ROS_INFO("%i", msg.data);
      //write to paralleport
      outb(255, BASEPORT);
      usleep(10);
      outb(0, BASEPORT);
  
      chatter_pub.publish(msg);
  
      ros::spinOnce();
  
      loop_rate.sleep();
      ++count;
    }
      fcntl(0, F_SETFL, tem);
      outb(0, BASEPORT);
	
      //take away permissions to access port
      if (ioperm(BASEPORT, 3, 0)) {perror("ioperm"); exit(1);}
  
    return 0;
}*/
