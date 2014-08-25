#include <ros/ros.h>
#include <nodelet/loader.h>
//#include <image_proc/advertisement_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_camera_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string camera_name = ros::this_node::getName();
  std::string left_image_raw_topic = ros::names::resolve(camera_name + "/left/image_raw");
  std::string right_image_raw_topic = ros::names::resolve(camera_name + "/right/image_raw");
  std::string left_image_rect_color_topic = ros::names::resolve(camera_name + "/left/image_rect_color");
  std::string right_image_rect_color_topic = ros::names::resolve(camera_name + "/right/image_rect_color");
  std::string calibration_raw_topic = ros::names::resolve(camera_name + "/stereo_calib_raw");
  std::string calibration_rect_topic = ros::names::resolve(camera_name + "/stereo_calib_rect");
  std::string point_cloud_topic = ros::names::resolve(camera_name + "/points");
  std::string point_cloud_service = ros::names::resolve(camera_name + "/grab_point_cloud");
  remap["left/image_raw"] = left_image_raw_topic;
  remap["right/image_raw"] = right_image_raw_topic;
  remap["left/image_rect_color"] = left_image_rect_color_topic;
  remap["right/image_rect_color"] = right_image_rect_color_topic;
  remap["stereo_calib_raw"] = calibration_raw_topic;
  remap["stereo_calib_rect"] = calibration_rect_topic;
  remap["points"] = point_cloud_topic;
  remap["grab_point_cloud"] = point_cloud_service;
  nodelet.load(camera_name.c_str(), "stereo_camera/StereoCamera", remap, nargv);
  nodelet.load("stereoPreporcess", "stereo_camera/StereoPreprocess", remap, nargv);
  nodelet.load("stereoPointCloud", "stereo_camera/StereoPointCloud", remap, nargv);

  ros::spin();
  return 0;
}
