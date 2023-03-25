#include <ros/ros.h>
#include <openhab_msgs/StringState.h>

class StringSubscriber
{
public:
    StringSubscriber(std::string item_name)
        : item_name_(item_name), enable_(false)
    {
        // Create a subscriber with appropriate topic, custom message and name of callback function.
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &StringSubscriber::callback, this);
    }

    void start()
    {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &StringSubscriber::callback, this);
    }

    void stop()
    {
        // Turn off subscriber.
        enable_ = false;
        sub_.shutdown();
    }

private:
    void callback(const openhab_msgs::StringState::ConstPtr& msg)
    {
        // Simply print out values in our custom message.
        if (!msg->isnull)
        {
            ROS_INFO("Received %s with state %s", msg->item.c_str(), msg->state.c_str());
        }
        else
        {
            ROS_INFO("Received %s with NULL", msg->item.c_str());
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string item_name_;
    bool enable_;
};

int main(int argc, char** argv)
{
    // Initialize the node and name it.
    std::string node_name = "StringSubscriberNode";
    ros::init(argc, argv, node_name);

    StringSubscriber stringSubscriber("testString");

    // Go to the main loop.
    try
    {
        stringSubscriber.start();
        // Wait for messages on topic, go to callback function when new messages arrive.
        // spin() simply keeps cpp from exiting until this node is stopped.
        ros::spin();
    }
    // Stop with Ctrl + C.
    catch (const std::exception& e)
    {
        stringSubscriber.stop();

        std::vector<std::string> nodes;
        ros::master::getNodes(nodes);
        for (auto node : nodes)
        {
            ros::NodeHandle nh(node);
            std::string node_namespace = nh.getNamespace();
            if (node_namespace.find(node_name) != std::string::npos)
            {
                ROS_INFO("Killing node %s", node_namespace.c_str());
                ros::service::waitForService(node_namespace + "/set_logger_level");
                ros::NodeHandle node_handle(node_namespace);
                ros::ServiceClient set_logger_level_client = node_handle.serviceClient<rosgraph_msgs::SetLoggerLevel>(node_namespace + "/set_logger_level");
                rosgraph_msgs::SetLoggerLevel set_logger_level_srv;
                set_logger_level_srv.request.logger = "";
                set_logger_level_srv.request.level = rosgraph_msgs::SetLoggerLevelRequest::LEVEL_DEBUG;
                set_logger_level_client.call(set_logger_level_srv);
                ros::service::waitForService(node_namespace + "/set_logger_level");
                ros::service::waitForService(node_namespace + "/rosout");
                ros::service::waitForService(node_namespace + "/rosout/get_loggers");
                ros::service::waitForService(node_namespace + "/rosout/set_logger_level");
                ros::ServiceClient set_logger_level_client2 = node_handle.serviceClient<rosgraph_msgs::SetLoggerLevel>(node_namespace + "/set_logger_level");
                rosgraph_msgs::SetLoggerLevel set_logger_level_srv2;
                set_logger_level_srv2.request.logger = "";
                set_logger_level_srv2.request.level = rosgraph_msgs::SetLoggerLevelRequest::LEVEL_FATAL;
                set_logger_level_client2.call(set_logger
