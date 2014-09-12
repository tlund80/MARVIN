#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//srv
#include <projector_select_image/projector_select_image.h>

typedef projector_select_image::projector_select_image::Request ReqT;
typedef projector_select_image::projector_select_image::Response ResT;

static const char WINDOW[] = "Image window";

bool imageCb(ReqT& req, ResT& res)
{
    std::string Image = req.image_path;
    std::string workdir = ros::package::getPath("projector_select_image");
    
    cv::namedWindow(WINDOW); // CV_WND_PROP_FULLSCREEN

    std::stringstream ImagePath;
    ImagePath << workdir << "/images/" << Image;
    std::cout<<"show image path "<< ImagePath.str() << std::endl;
    cv::Mat img = cv::imread(ImagePath.str() ,1);

    cv::imshow(WINDOW,img);
    cv::waitKey(3);
    
    res.success = true;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "projector_select_image");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ros::ServiceServer service = nh.advertiseService("ChooseImage", imageCb);
    ROS_INFO("Ready to call projector_select_image service...");
  
    ros::spin();
    return 0;
}