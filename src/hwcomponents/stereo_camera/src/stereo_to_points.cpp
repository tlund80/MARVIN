/**
 *
 * This component subscribes to a stereo camera and computes a
 * points cloud based on the stereo image.
 *
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <caros_common_msgs/CapturePoints2.h>

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 1

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <stdio.h>
#include <iostream>

//#include <PoseEstimation/PoseEstimation.h>
//using namespace PoseEstimation;

// Object detection
// #include <marvin_common/ObjectDetection.h>
// typedef marvin_common::ObjectDetection MsgT;

using namespace std;
using namespace cv;
using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;

vector<image_transport::SubscriberFilter *> subscribterVector;
image_transport::SubscriberFilter* sub_temp_1;
image_transport::SubscriberFilter* sub_temp_2;
vector<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *> timeSync;
cv::Mat imgL;
cv::Mat imgR;

std::string calib_intrinsic, calib_extrinsic, image_left, image_right;
sensor_msgs::PointCloud2 scene;
ros::Time stamp;
int default_numberOfDisparities, default_SADWindowSize;
bool publishPoints;

//Function declear
void stereoCallback(const sensor_msgs::ImageConstPtr& msg_left_image,
                       const sensor_msgs::ImageConstPtr& msg_right_image);

pcl::PointCloud<pcl::PointXYZRGB> toPCLpointcloud(const Mat& mat, const Mat& rgb);

bool toPointsCB(caros_common_msgs::CapturePoints2::Request &req, caros_common_msgs::CapturePoints2::Response &res);

// for calibration data
Rect roi1, roi2;
Mat_<double> M1, D1, M2, D2, R, T, R1, P1, R2, P2;


/**
 * defines a service to grab points and it publish a points2
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "stereo_to_points");
	ros::NodeHandle nodeHandle = ros::NodeHandle("~");
	image_transport::ImageTransport imageTransport(nodeHandle);

	nodeHandle.getParam("intrinsics", calib_intrinsic);
	nodeHandle.getParam("extrinsics", calib_extrinsic);
	nodeHandle.getParam("image_left", image_left);
	nodeHandle.getParam("image_right", image_right);

	// optional parameters
	nodeHandle.param("max_disparities", default_numberOfDisparities, 176); // should be dividable with 16
    nodeHandle.param("block_size", default_SADWindowSize, 11); // should be odd number between 3 and 11

	ROS_INFO_STREAM("Using extrinsics: "<< calib_extrinsic);
	ROS_INFO_STREAM("Using intrinsics: "<< calib_intrinsic);

    if (!calib_intrinsic.empty()) {
        // reading intrinsic parameters
        FileStorage fs(calib_intrinsic, CV_STORAGE_READ);
        if (!fs.isOpened()) {
            ROS_FATAL_STREAM("Failed to open file: " << calib_intrinsic);
        }
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;
    } else {
        ROS_FATAL("Please specify an intrinsic calibration file! use _intrinsics:=... ");
    }

    if(!calib_extrinsic.empty()){
        FileStorage fs(calib_extrinsic, CV_STORAGE_READ);
        if (!fs.isOpened()) {
            ROS_FATAL_STREAM("Failed to open file: " << calib_extrinsic);
        }
        fs["R"] >> R;
        fs["T"] >> T;
    } else {
        ROS_FATAL("Please specify an intrinsic calibration file! use _extrinsics:=... ");
    }

	image_transport::SubscriberFilter filter_left, filter_right;
	filter_left.subscribe(imageTransport, image_left, 1);
	filter_right.subscribe(imageTransport, image_right, 1);

	typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> StereoImageSyncMsgFilter;
	StereoImageSyncMsgFilter timeSync(1);
	timeSync.connectInput(filter_left, filter_right);
	timeSync.registerCallback( boost::bind(&stereoCallback, _1, _2) );

	// we provide a service for users to grab a poit cloud
	int countslash = 0, count = 0;
	std::string image_topic = image_left;
	for(string::iterator it= image_topic.begin(); it != image_topic.end(); it++) {
	  count ++;
	  if((*it) == '/') countslash ++;
	  if(countslash == 2) break;
	}
	image_topic = image_topic.substr(0, count-1);
	std::stringstream servicename, topicname;
	servicename << image_topic << "/GrabStereoPoints";
	topicname <<image_topic << "/points2";
	std::cout<<"image topic: " <<image_topic<<"\n";
	
	ros::ServiceServer service = nodeHandle.advertiseService(servicename.str(), toPointsCB);

	// we publis points whenever someone needs them
	ros::Publisher pointsPub = nodeHandle.advertise<sensor_msgs::PointCloud2>(topicname.str(), 1);

	ROS_INFO("Ready generate pcd of the scene.");

	ros::Rate rate(30);
	while( ros::ok() ){
	    // check if any have subscribed to the published node
	    publishPoints = pointsPub.getNumSubscribers()>0;
	    ros::spinOnce();
	    rate.sleep();
	}

	return 0;
}


bool toPoints(int numberOfDisparities, int SADWindowSize ) {

    cv::Mat imgL_grey;
    cv::Mat imgR_grey;
    cvtColor(imgL, imgL_grey, CV_RGB2GRAY);
    cvtColor(imgR, imgR_grey, CV_RGB2GRAY);

    StereoBM bm;
    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 10;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = -1;

    Mat disp, disp8;
    bm(imgL_grey, imgR_grey, disp);

    //disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    //imwrite("temp_disparity.png", disp8);

    // Get parameters for reconstruction
    float f = M1(0, 0); // Focal length
    float B = T(0, 0); // Baseline in the x direction
    float cx = M1(0, 2); // Center x coordinate
    float cy = M1(1, 2); // Center y coordinate

    float cx2 = M2(0, 2); // Center x coordinate of right image
    float dcx = cx - cx2; // Difference in center x coordinates

    int temp = disp.at<int16_t>(0, 0);
    for (int y = 0; y < disp.rows; ++y) {
        for (int x = 0; x < disp.cols; ++x) {
            if (temp > disp.at<int16_t>(y, x))
                temp = disp.at<int16_t>(y, x);
        }
    }

    Mat_<Vec3f> xyz(disp.rows, disp.cols, Vec3f(0, 0, 0)); // Resulting point cloud, initialized to zero
    for (int y = 0; y < disp.rows; ++y) {
        for (int x = 0; x < disp.cols; ++x) {
            Vec3f& p = xyz(y, x); // Point to write to

            // Avoid invalid disparities
            if (disp.at<int16_t>(y, x) == temp)
                continue;
            if (disp.at<int16_t>(y, x) == 0)
                continue;

            // Maths from here: http://answers.opencv.org/upfiles/13535653931527438.jpg
            float d = float(disp.at<int16_t>(y, x)) / 16.0f; // Disparity
            float W = B / (-d + dcx); // Weighting

            p[0] = (float(x) - cx) * W;
            p[1] = (float(y) - cy) * W;
            p[2] = f * W;
        }
    }
    stamp = ros::Time::now();
    pcl::PointCloud < pcl::PointXYZRGB > cloud = toPCLpointcloud(xyz, imgL);
    pcl::toROSMsg < pcl::PointXYZRGB > (cloud, scene);
    return true;
}

bool toPointsCB(caros_common_msgs::CapturePoints2::Request &req, caros_common_msgs::CapturePoints2::Response &res) {
    ROS_INFO("Get point cloud called");

    int numberOfDisparities = req.max_disparity;
    int SADWindowSize = req.blocksize;

    bool resLocal = toPoints(numberOfDisparities, SADWindowSize);
    if(resLocal)
        res.outputScene = scene;
    return resLocal;
}



void stereoCallback(const sensor_msgs::ImageConstPtr& msg_left_image,
		const sensor_msgs::ImageConstPtr& msg_right_image) {

	cv_bridge::CvImagePtr leftImageCVPtr, rightImageCVPtr;

	try {
		leftImageCVPtr = cv_bridge::toCvCopy(msg_left_image,
				msg_left_image->encoding);
		rightImageCVPtr = cv_bridge::toCvCopy(msg_right_image,
				msg_right_image->encoding);

	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from encodings. %s", e.what());
	}
	imgL = leftImageCVPtr->image;
	imgR = rightImageCVPtr->image;
	toPoints(default_numberOfDisparities, default_SADWindowSize);
}

pcl::PointCloud<pcl::PointXYZRGB> toPCLpointcloud(const Mat& mat, const Mat& rgb) {
	pcl::PointCloud < pcl::PointXYZRGB > cloud;
	cloud.reserve(mat.cols * mat.rows);

	const double max_z = 2e3; // Disregard points farther away than 2 m
	//FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++) {
		for (int x = 0; x < mat.cols; x++) {
			Vec3f point = mat.at<Vec3f>(y, x);

			// This omits zero points
			if (point[0] == 0 && point[1] == 0 && point[2] == 0)
				continue;

			// This omits points equal to or larger than max_z
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
				continue;
			//fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);

			// Point to write to
			pcl::PointXYZRGB p;

			// Scale position from mm to m
			p.x = 0.001 * point[0];
			p.y = 0.001 * point[1];
			p.z = 0.001 * point[2];

			// OpenCV reads in images in BGR order, so we must switch to BGR for PCL
			Vec3b pbgr = rgb.at<Vec3b>(y, x);
			p.b = pbgr[0];
			p.g = pbgr[1];
			p.r = pbgr[2];

			cloud.push_back(p);
		}
	}
	return cloud;
}
