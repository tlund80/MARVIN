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
 \file KinectVibrator.hpp
 \date 15/05/2013
 \This is a class that is used to control the hardware triggering of the pike and bumblebee cameras.
*/

#ifndef KINECTVIBRATOR_HPP
#define KINECTVIBRATOR_HPP

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "kinect_vibrator/FrameCount.h"
#include "kinect_vibrator/Start.h"
#include "kinect_vibrator/Stop.h"
#include "kinect_vibrator/Reset.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/parport.h>
#include <linux/ppdev.h>
#include <sstream>


class KinectVibrator {

public:
	/**
       * Constructor of the class. 
       * A more elaborate description of the constructor.
	   * @param service_name not used at the moment
	   * @param looprate is the miliseconds the thread sleeps.
       */
	KinectVibrator(const std::string& service_name, double loopRate);
	
	/**
	  * Destructor of the class. 
      */
    ~KinectVibrator();
	
	/**
      * Starts the main loop of the node. Its a while loop that checks on ROS::OK() 
      */
    void run();
	
	/**
	  * Returns the current framecount
	  * @param req 
	  * @param res is the return message container. 
	  */
    bool getFrameCount(kinect_vibrator::FrameCountRequest  &req,
         kinect_vibrator::FrameCountResponse &res );
    /**
	  * Disables the triggering loop.
	  * @param req not used.
	  * @param res not used. 
	  */
	bool stop(kinect_vibrator::StopRequest &req,
         kinect_vibrator::StopResponse &res);
    
	/**
	  * Enables the triggering loop.
	  * @param req not used.
	  * @param res not used. 
	  */
	bool start(kinect_vibrator::StartRequest &req,
         kinect_vibrator::StartResponse &res);
    
	/**
	  * Resets the framecount. This method is called by the serviceserver, when _srvReset is signaled. 
	  * @param req not used.
	  * @param res not used. 
	  */
	bool reset(kinect_vibrator::ResetRequest &req,
         kinect_vibrator::ResetResponse &res);
private:
    int _count;
    int _parportfd;
    bool _runTrigger;

    std::string _service_name;
    std_msgs::Int32 _msg;

    ros::Rate _loopRate;
    ros::NodeHandle _nodeHnd;
    ros::ServiceServer _srvFrameCount;
    ros::ServiceServer _srvStart;
    ros::ServiceServer _srvStop;
    ros::ServiceServer _srvReset;
    ros::Publisher _chatter_pub;
	unsigned char _dataH;
	unsigned char _dataL;
	

    void loop();
    int init();


};
#endif //#ifndef KINECTVIBRATOR_HPP
