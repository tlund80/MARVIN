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
   \author Wail Mustafa
   \file frameSaver.cpp
   \brief
*/



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>

#include <tr1/memory>
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <sstream>


using namespace std;
using namespace tr1;
using namespace message_filters;
using namespace sensor_msgs;
// using namespace Modules;


int counter;
int objectID;
std::string folder;

vector<image_transport::SubscriberFilter *> subscribterVector;
    
image_transport::SubscriberFilter* sub_temp_1;
image_transport::SubscriberFilter* sub_temp_2;

vector< message_filters::TimeSynchronizer< Image, Image> *> timeSync;

void stereoSaver(image_transport::ImageTransport imageTransport,string name);
void kinectSaver(image_transport::ImageTransport imageTransport,string name);

void stereoCallback( const sensor_msgs::ImageConstPtr& msg_left_image,
                    const sensor_msgs::ImageConstPtr& msg_right_image) {
  
    cv_bridge::CvImagePtr leftImageCVPtr, rightImageCVPtr;


    try {
        leftImageCVPtr = cv_bridge::toCvCopy( msg_left_image, msg_left_image->encoding);
        rightImageCVPtr = cv_bridge::toCvCopy( msg_right_image, msg_right_image->encoding);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from encodings. %s", e.what());
    }

   std::stringstream LeftPath;
   LeftPath <<folder<<msg_left_image.get()->header.frame_id<<"_left_"<<msg_left_image.get()->header.seq<< ".png"; 

   std::stringstream RightPath;
   RightPath <<folder<<msg_right_image.get()->header.frame_id<<"_right_"<<msg_left_image.get()->header.seq<< ".png"; 
   
   cv::imwrite(LeftPath.str(),leftImageCVPtr->image); 
   cout << "Files saved: " << LeftPath.str() << endl;
   cv::imwrite(RightPath.str(),rightImageCVPtr->image);
   cout << "Files saved: " << RightPath.str() << endl;
}

void kinectCallback( const sensor_msgs::ImageConstPtr& msg_rgb_image,
                    const sensor_msgs::ImageConstPtr& msg_depth_image) {
  
    cv_bridge::CvImagePtr rgbImageCVPtr, depthImageCVPtr;


    try {
        rgbImageCVPtr = cv_bridge::toCvCopy( msg_rgb_image, msg_rgb_image->encoding);
        depthImageCVPtr = cv_bridge::toCvCopy( msg_depth_image, msg_depth_image->encoding);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from encodings. %s", e.what());
    }

   std::stringstream rgbPath;
   rgbPath <<folder<<msg_rgb_image.get()->header.frame_id<<"_RGB_"<<msg_rgb_image.get()->header.seq<< ".png"; 

   std::stringstream depthPath;
   depthPath <<folder<<msg_depth_image.get()->header.frame_id<<"_depth_"<<msg_rgb_image.get()->header.seq<<".png";
//    std::stringstream depthPathYml;
//    depthPathYml <<folder<<msg_depth_image.get()->header.frame_id<<"_depth_"<<counter<< ".yml";
   
   cv::Mat colorImage_BGR;
   cv::cvtColor(rgbImageCVPtr->image, colorImage_BGR, CV_RGB2BGR);
   
   cv::imwrite(rgbPath.str(),colorImage_BGR); 
//    cv::imwrite(rgbPath.str(),rgbImageCVPtr->image);
   cout << "Files saved: " << rgbPath.str() << endl;
   cv::imwrite(depthPath.str(),depthImageCVPtr->image);
   cout << "Files saved: " << depthPath.str() << endl;
   
//    cv::FileStorage fs;
//    fs.open(depthPathYml.str(), cv::FileStorage::WRITE);
//    fs << "data3D" << depthImageCVPtr->image;
//    fs.release();
//    depthPathYml.str("");
//    cout << "Files saved: " << depthPathYml.str() << endl;
   //cv::Mat test;
   //test=cv::imread(depthPath.str(),0);
   //cout << "format: " << test.type() << endl;
   //cv_bridge::CvImage cviDepthRGB;
   //cviDepthRGB.image=test;
   //cout << "image type: " <<  cv_bridge::getCvType(msg_depth_image->encoding) << endl;
   
}

int main(int argc, char **argv) {
  

   ros::init(argc, argv, "frame_saver");
   
   ros::NodeHandle nodeHandle = ros::NodeHandle("~");
   
   counter = 0;
   folder = "/home/trs/cameraData/";
   char choice;
   
   image_transport::ImageTransport imageTransport(nodeHandle);

   
 //   stereoSaver(imageTransport,"/bumblebeeLeft");
    stereoSaver(imageTransport,"/bumblebeeRight");    
 //   stereoSaver(imageTransport,"/bumblebeeCenter");
 //   kinectSaver(imageTransport,"/kinectLeft");
    kinectSaver(imageTransport,"/kinectRight");
 //   kinectSaver(imageTransport,"/kinectCenter");
 
    while (ros::ok()) {    

//       if (choice == 's'){ 
// 	counter++; 
	ros::spinOnce(); 
//       } 
    }

   return 0;
}

void stereoSaver(image_transport::ImageTransport imageTransport,string name) {
  

   std::stringstream LeftPath;
   LeftPath <<name<<"/left/image_rect_color";
   
   std::stringstream RightPath;
   RightPath <<name<<"/right/image_rect_color"; 
   
   subscribterVector.push_back(new image_transport::SubscriberFilter());
   sub_temp_1=subscribterVector.back();
   sub_temp_1->subscribe(imageTransport, LeftPath.str(), 1);

   subscribterVector.push_back(new image_transport::SubscriberFilter());
   sub_temp_2=subscribterVector.back();
   sub_temp_2->subscribe(imageTransport, RightPath.str(), 1);
   
   timeSync.push_back(new message_filters::TimeSynchronizer< Image, Image>(1) );
   timeSync.back()->connectInput(*sub_temp_1,*sub_temp_2);
   timeSync.back()->registerCallback(boost::bind(&stereoCallback, _1, _2));  

}

void kinectSaver(image_transport::ImageTransport imageTransport,string name) {
  

   std::stringstream rgbPath;
   rgbPath <<name<<"/kinect_RGB";
   
   std::stringstream DepthPath;
   DepthPath <<name<<"/kinect_depth"; 
   
   subscribterVector.push_back(new image_transport::SubscriberFilter());
   sub_temp_1=subscribterVector.back();

   sub_temp_1->subscribe(imageTransport, rgbPath.str(), 1);

   subscribterVector.push_back(new image_transport::SubscriberFilter());
   sub_temp_2=subscribterVector.back();

   sub_temp_2->subscribe(imageTransport, DepthPath.str(), 1);

   timeSync.push_back(new message_filters::TimeSynchronizer< Image, Image>(1) );

   timeSync.back()->connectInput(*sub_temp_1,*sub_temp_2);

   timeSync.back()->registerCallback(boost::bind(&kinectCallback, _1, _2));

}


