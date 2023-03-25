#include <ros/ros.h>
#include <openhab_msgs/ContactState.h>

class ContactSubscriber
{
public:
  ContactSubscriber(std::string item_name)
  {
    item_name_ = item_name;
    enable_ = false;

    // Create a subscriber with appropriate topic, custom message and name of
    // callback function.
    sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &ContactSubscriber::callback, this);

    if (enable_)
      start();
    else
      stop();
  }

  void start()
  {
    enable_ = true;
    sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &ContactSubscriber::callback, this);
  }

  void callback(const openhab_msgs::ContactState::ConstPtr& msg)
  {
    // Simply print out values in our custom message.
    data_ = *msg;
    if (!data_.isnull)
    {
      std::string msg_str = "Received " + data_.item + " with state " + (data_.state ? "true" : "false");
      ROS_INFO_STREAM(ros::this_node::getName() + msg_str);
    }
    else
      ROS_INFO_STREAM(ros::this_node::getName() + "Received " + data_.item + " with NULL");
  }

  void stop()
  {
    // Turn off subscriber.
    enable_ = false;
    sub_.shutdown();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string item_name_;
  bool enable_;
  openhab_msgs::ContactState data_;
};

int main(int argc, char **argv)
{
  // Initialize the node and name it.
  std::string node_name = "ContactSubscriberNode";
  ros::init(argc, argv, node_name);

  ContactSubscriber contactSubscriber("testContact");

  // Go to the main loop
  try
  {
    contactSubscriber.start();
    // Wait for messages on topic, go to callback function when new messages arrive.
    // spin() simply keeps python from exiting until this node is stopped
    ros::spin();
  }
  // Stop with Ctrl + C
  catch (...)
  {
    contactSubscriber.stop();

    std::vector<std::string> nodes;
    std::string command = "rosnode list";
    FILE* output = popen(command.c_str(), "r");
    if (output)
    {
      char buffer[128];
      while (!feof(output))
      {
        if (fgets(buffer, 128, output) != nullptr)
          nodes.push_back(std::string(buffer).substr(0, std::string(buffer).length() - 1));
      }
      pclose(output);
    }

    for (const auto& node : nodes)
      system(("rosnode kill " + node_name).c_str());

    ROS_INFO_STREAM("Node stopped");
  }

  return 0;
}
