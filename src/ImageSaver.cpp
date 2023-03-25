#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "openhab_msgs/ImageState.h"

bool VERBOSE = false;

class ImageSaver
{
public:
  ImageSaver(std::string item_name)
    : item_name_(item_name)
    , enable_(false)
    , data_(nullptr)
    , i_(0)
    , nh_("~")
  {
    // Create a subscriber with appropriate topic, custom message and name of callback function.
    sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &ImageSaver::callback, this);
    name_ = item_name_ + "_" + ros::Time::now().toSec() + "_" + std::to_string(i_) + ".jpg";

    if (enable_)
    {
      start();
    }
    else
    {
      stop();
    }
  }

  void start()
  {
    enable_ = true;
    sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &ImageSaver::callback, this);
  }

  void callback(const openhab_msgs::ImageState::ConstPtr& msg)
  {
    // Simply print out values in our custom message.
    data_ = msg;

    if (data_->isnull == false)
    {
      std::stringstream ss;
      ss << "Received " << data_->item;

      try
      {
        image_ = cv_bridge::toCvCopy(data_->state, sensor_msgs::image_encodings::BGR8)->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      i_++;
      cv::imwrite(name_, image_);
    }
    else
    {
      std::stringstream ss;
      ss << "Received " << data_->item << " with NULL";
    }
    ROS_INFO_STREAM(ros::this_node::getName() << ": " << ss.str());
  }

  void stop()
  {
    // Turn off subscriber.
    enable_ = false;
    sub_.shutdown();
  }

private:
  std::string item_name_;
  bool enable_;
  openhab_msgs::ImageState::ConstPtr data_;
  cv::Mat image_;
  int i_;
  std::string name_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  // Initialize the node and name it.
  std::string node_name = "ImageSaverNode";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  ImageSaver imageSaver("testImage");

  // Wait for messages on topic, go to callback function when new messages arrive.
  // spin() simply keeps cpp from exiting until this node is stopped
  try
  {
    imageSaver.start();
    ros::spin();
  }
  catch (...)
  {
    imageSaver.stop();
  }

  return 0;
}
