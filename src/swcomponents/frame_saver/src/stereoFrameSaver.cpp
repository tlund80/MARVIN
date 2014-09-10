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
 \author Dirk Kraft
 \file stereoFrameSaver.cpp
 \brief Tool to save images from a single stereo camera
 Adapted from ROS's stereo_view.cpp
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <marvin_common/StereoCalibration.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroy(GtkWidget *widget, gpointer data)
{
  ros::shutdown();
}
#endif

namespace enc = sensor_msgs::image_encodings;


void increment(int* value)
{
  ++(*value);
}

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

// Note: StereoView is NOT nodelet-based, as it synchronizes the three streams.
class StereoView
{
private:
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<marvin_common::StereoCalibration> calib_sub_;
  typedef ExactTime<Image, Image> ExactPolicy;
  typedef ExactTime<Image, Image, marvin_common::StereoCalibration> ExactPolicyCalib;
  typedef ApproximateTime<Image, Image> ApproximatePolicy;
  typedef ApproximateTime<Image, Image, marvin_common::StereoCalibration> ApproximatePolicyCalib;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ExactPolicyCalib> ExactSyncCalib;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  typedef message_filters::Synchronizer<ApproximatePolicyCalib> ApproximateSyncCalib;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ExactSyncCalib> exact_sync_calib_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  boost::shared_ptr<ApproximateSyncCalib> approximate_sync_calib_;
  int queue_size_;
  std::string image_out_dir_;
  bool save_calibration_;

  ImageConstPtr last_left_msg_, last_right_msg_;
  cv::Mat last_left_image_, last_right_image_;
  marvin_common::StereoCalibration last_calib_;
  boost::mutex image_mutex_;

  boost::format filename_format_;
  int save_count_;

  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_, calib_received_, all_received_;

  enum {
    eRaw,
    eRawDebayer,
    eColor,
  } image_type_;

  int debayer_type_;

public:
  StereoView()
    : filename_format_(""), save_count_(0),
      left_received_(0), right_received_(0), calib_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    bool autosize;
    local_nh.param("autosize", autosize, true);

    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("%s%04i.png"));
    filename_format_.parse(format_string);

    local_nh.param("image_out_dir", image_out_dir_, std::string("images/"));

    if (!boost::filesystem::is_directory(image_out_dir_)) {
      ROS_ERROR_STREAM("The directory " << image_out_dir_ << " does not exist or is not a directory!");
    }

    // Do GUI window setup
    int flags = autosize ? CV_WINDOW_AUTOSIZE : 0;
    cv::namedWindow("left", flags);
    cv::namedWindow("right", flags);
    cvSetMouseCallback("left", &StereoView::mouseCb, this);
    cvSetMouseCallback("right", &StereoView::mouseCb, this);
#ifdef HAVE_GTK
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("left") ),
                     "destroy", G_CALLBACK(destroy), NULL);
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("right") ),
                     "destroy", G_CALLBACK(destroy), NULL);
#endif
    cvStartWindowThread();

    std::string image_type_string;
    local_nh.param("image_type", image_type_string, std::string("raw"));
    if (image_type_string == "raw") {
      image_type_ = eRaw;
    } else if (image_type_string == "rawd") {
      image_type_ = eRawDebayer;
    } else if (image_type_string == "colr") {
      image_type_ = eColor;
    } else {
      ROS_ERROR_STREAM(image_type_string << " is not a valid image type to capture! Switching to raw!");
      image_type_ = eRaw;
    }

    std::string image_topic_name;
    std::string calib_topic_name;
    switch (image_type_) {
      case eRaw:
      case eRawDebayer:
        image_topic_name = "image_raw";
        calib_topic_name = "stereo_calib_raw";
        break;
      case eColor:
        image_topic_name = "image_rect_color";
        calib_topic_name = "stereo_calib_rect";
        break;
    }

    std::string debayer_type_string;
    local_nh.param("debayer_type", debayer_type_string, std::string("GB"));

    if (debayer_type_string == "BG") {
      debayer_type_ = CV_BayerBG2BGR;
	} else if (debayer_type_string == "GB") {
      debayer_type_ = CV_BayerGB2BGR;
	} else if (debayer_type_string == "RG") {
	  debayer_type_ = CV_BayerRG2BGR;
	} else if (debayer_type_string == "GR") {
	  debayer_type_ = CV_BayerGR2BGR;
	} else {
	  ROS_ERROR_STREAM(debayer_type_string << " is not a valid bayer pattern type! Switching to GB!");
	  debayer_type_ = CV_BayerGB2BGR;
	}

    local_nh.param("save_calibration", save_calibration_, false);

    // Resolve topic names
    ros::NodeHandle nh;
    std::string stereo_ns = nh.resolveName("cam_base_name");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + image_topic_name);
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + image_topic_name);
    std::string calib_topic;
    if (save_calibration_) {
      calib_topic = ros::names::clean(stereo_ns + "/" + calib_topic_name);
      ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s", left_topic.c_str(), right_topic.c_str(), calib_topic.c_str());
    } else {
      ROS_INFO("Subscribing to:\n\t* %s\n\t* %s", left_topic.c_str(), right_topic.c_str());
    }


    // Subscribe to three input topics.
    image_transport::ImageTransport it(nh);
    std::string image_transport_type_string;
    local_nh.param("image_transport", image_transport_type_string, std::string("raw"));
    left_sub_.subscribe(it, left_topic, 1, image_transport_type_string);
    right_sub_.subscribe(it, right_topic, 1, image_transport_type_string);
    if(save_calibration_) {
      calib_sub_.subscribe(nh, calib_topic, 1);
    }

    // Complain every 30s if the topics appear unsynchronized
    left_sub_.registerCallback(boost::bind(increment, &left_received_));
    right_sub_.registerCallback(boost::bind(increment, &right_received_));
    if (save_calibration_) {
      calib_sub_.registerCallback(boost::bind(increment, &calib_received_));
    }
    check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0),
                                             boost::bind(&StereoView::checkInputsSynchronized, this));

    // Synchronize input topics. Optionally do approximate synchronization.
    local_nh.param("queue_size", queue_size_, 5);
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
      if (!save_calibration_) {
        approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size_),
                                                     left_sub_, right_sub_) );
        approximate_sync_->registerCallback(boost::bind(&StereoView::imageCb, this, _1, _2));
      } else {
        approximate_sync_calib_.reset( new ApproximateSyncCalib(ApproximatePolicyCalib(queue_size_),
                                                                left_sub_, right_sub_, calib_sub_) );
        approximate_sync_calib_->registerCallback(boost::bind(&StereoView::imageCalibCb, this, _1, _2, _3));
      }
    }
    else
    {
      if (!save_calibration_) {
        exact_sync_.reset( new ExactSync(ExactPolicy(queue_size_),
                                         left_sub_, right_sub_) );
        exact_sync_->registerCallback(boost::bind(&StereoView::imageCb, this, _1, _2));
      } else {
        exact_sync_calib_.reset( new ExactSyncCalib(ExactPolicyCalib(queue_size_),
                                                    left_sub_, right_sub_, calib_sub_) );
        exact_sync_calib_->registerCallback(boost::bind(&StereoView::imageCalibCb, this, _1, _2, _3));
      }
    }
  }

  ~StereoView()
  {
    cvDestroyAllWindows();
  }

  void imageCalibCb(const ImageConstPtr& left,
                    const ImageConstPtr& right,
                    const marvin_common::StereoCalibrationConstPtr & calib)
  {
    ++all_received_; // For error checking

    image_mutex_.lock();

    processImageCallback( left, right);

    last_calib_ = *calib;

    // Must release the mutex before calling cv::imshow, or can deadlock against
    // OpenCV's window mutex.
    image_mutex_.unlock();
    if (!last_left_image_.empty())
      cv::imshow("left", last_left_image_);
    if (!last_right_image_.empty())
      cv::imshow("right", last_right_image_);

  }

  void imageCb(const ImageConstPtr& left, const ImageConstPtr& right)
  {
    ++all_received_; // For error checking

    image_mutex_.lock();

    processImageCallback( left, right);
    // Must release the mutex before calling cv::imshow, or can deadlock against
    // OpenCV's window mutex.
    image_mutex_.unlock();
    if (!last_left_image_.empty())
      cv::imshow("left", last_left_image_);
    if (!last_right_image_.empty())
      cv::imshow("right", last_right_image_);
  }

  void processImageCallback(const ImageConstPtr& left, const ImageConstPtr& right) {
    // May want to view raw bayer data
    if (left->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<Image>(left)->encoding = "mono8";
    if (right->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<Image>(right)->encoding = "mono8";

    // Hang on to image data for sake of mouseCb
    last_left_msg_ = left;
    last_right_msg_ = right;

    std::string target_image_type = (image_type_ == eRawDebayer) ? "mono8" : "bgr8";

    try {
	  last_left_image_ = cv_bridge::toCvShare(left, target_image_type)->image;
	  last_right_image_ = cv_bridge::toCvShare(right, target_image_type)->image;
	}
	catch (cv_bridge::Exception& e) {
	  ROS_ERROR("Unable to convert one of '%s' or '%s' to '%s'",
				left->encoding.c_str(), right->encoding.c_str(), target_image_type.c_str());
	}

    if (image_type_ == eRawDebayer) {
	  cvtColor(last_left_image_, last_left_image_, debayer_type_);
	  cvtColor(last_right_image_, last_right_image_, debayer_type_);
    }
  }

  void saveImage(const char* prefix, const cv::Mat& image)
  {
    if (!image.empty()) {
      std::string filename = image_out_dir_ + "/" + (filename_format_ % prefix % save_count_).str();
      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save %s image, no data!", prefix);
    }
  }

  static void mouseCb(int event, int x, int y, int flags, void* param)
  {
    if (event != CV_EVENT_RBUTTONDOWN)
      return;

    StereoView *sv = (StereoView*)param;
    boost::lock_guard<boost::mutex> guard(sv->image_mutex_);

    sv->saveImage("left", sv->last_left_image_);
    sv->saveImage("right", sv->last_right_image_);
    if (sv->save_calibration_) {
      std::string filename = sv->image_out_dir_ + "/" + (sv->filename_format_ % "calib" % sv->save_count_).str() + ".txt";
      CovisRos::toCovis(sv->last_calib_).saveToOpenCVFile(filename);
      ROS_INFO("Saved calibration as %s", filename.c_str());
    }

    sv->save_count_++;
  }

  void checkInputsSynchronized() {

    int threshold = (save_calibration_ ? 3 : 2) * all_received_;
    if (left_received_ >= threshold || right_received_ >= threshold || calib_received_ >= threshold) {
      ROS_WARN("[stereo_view] Low number of synchronized left/right/disparity triplets received.\n"
               "Left images received: %d (topic '%s')\n"
               "Right images received: %d (topic '%s')\n"
               "Calibration messages received: %d (topic '%s')\n"
               "Synchronized triplets: %d\n"
               "Possible issues:\n"
               "\t Does `rosnode info %s` show any connections?\n"
               "\t* The cameras are not synchronized.\n"
               "\t Try restarting stereo_frame_saver with parameter _approximate_sync:=True\n"
               "\t* The network is too slow. One or more messages are dropped from each triplet.\n"
               "\t Try restarting stereo_frame_saver, increasing parameter 'queue_size' (currently %d)",
               left_received_, left_sub_.getTopic().c_str(),
               right_received_, right_sub_.getTopic().c_str(),
               calib_received_, calib_sub_.getTopic().c_str(),
               all_received_, ros::this_node::getName().c_str(), queue_size_);
    }
  }
};

int main(int argc, char **argv)
{

  // ?Names?:
  // cam_base_name

  // Paramters:
  // image_type - raw   (raw)
  //            - rawd  (raw+debayer)
  //            - colr  (color_rect)
  // debayer_type - BG
  //              - GB
  //              - RG
  //              - GR
  // save_calibration
  // image_out_dir
  // filename_format
  // autosize
  // filename_format
  // image_transport
  // approximate_sync
  // queue_size


  ros::init(argc, argv, "stereo_frame_saver");
  if (ros::names::remap("cam_base_name") == "cam_base_name") {
    ROS_WARN("'cam_base_name' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun frame_saver stereo_frame_saver cam_base_name:=/bumblebeeLeft");
  }

  StereoView view;

  ros::spin();
  return 0;
}
