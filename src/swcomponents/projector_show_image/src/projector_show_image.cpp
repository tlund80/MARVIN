#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::string INPUT_IMAGE_TOPIC;
  double PROJECTOR_WIDTH;
  double PROJECTOR_HEIGHT;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    ros::NodeHandle n("~");
    n.param<std::string> ("INPUT_IMAGE_TOPIC", INPUT_IMAGE_TOPIC, "left");
    std::cout << "Subscribing on topic: " << INPUT_IMAGE_TOPIC << std::endl;
    n.param<double> ("PROJECTOR_WIDTH", PROJECTOR_WIDTH, 1280);
    n.param<double> ("PROJECTOR_HEIGHT", PROJECTOR_HEIGHT, 800);
    std::cout << "Projector: " << PROJECTOR_WIDTH << " x " << PROJECTOR_HEIGHT << std::endl;

    image_sub_ = it_.subscribe(INPUT_IMAGE_TOPIC, 1, &ImageConverter::imageCb, this);
    cv::namedWindow(WINDOW); // CV_WND_PROP_FULLSCREEN
    cv::Mat img = cv::Mat::zeros(PROJECTOR_HEIGHT,PROJECTOR_WIDTH,CV_32F); // cv::imread("/home/lsorensen/pattern_1800X2400.png");
    imshow(WINDOW, img);
    cv::waitKey(3);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //cv::setWindowProperty(WINDOW, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "projector_show_image");
  ImageConverter ic;
  ros::spin();
  return 0;
}

