#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <openhab_msgs/DateTimeState.h>

class DateTimeSubscriber
{
public:
  DateTimeSubscriber(const std::string& item_name)
    : item_name_(item_name), enable_(false)
  {
    sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &DateTimeSubscriber::callback, this);
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
    sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &DateTimeSubscriber::callback, this);
  }

  void callback(const openhab_msgs::DateTimeState& data)
  {
    data_ = data;
    if (!data_.isnull)
    {
      ROS_INFO_STREAM(ros::this_node::getName() << " Received " << data_.item << " with state " << data_.state);
    }
    else
    {
      ROS_INFO_STREAM(ros::this_node::getName() << " Received " << data_.item << " with NULL");
    }
  }

  void stop()
  {
    enable_ = false;
    sub_.shutdown();
  }

private:
  std::string item_name_;
  bool enable_;
  openhab_msgs::DateTimeState data_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  // Initialize the node and name it
  const std::string node_name = "DateTimeSubscriberNode";
  ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
  DateTimeSubscriber dateTimeSubscriber("testDateTime");
  try
  {
    dateTimeSubscriber.start();
    // Wait for messages on topic, go to callback function when new messages arrive.
    // spin() simply keeps C++ from exiting until this node is stopped
    ros::spin();
  }
  // Stop with Ctrl + C
  catch (const std::exception&)
  {
    dateTimeSubscriber.stop();
    std::system(("rosnode kill " + node_name).c_str());
    std::cout << "Node stopped" << std::endl;
  }
  return 0;
}
