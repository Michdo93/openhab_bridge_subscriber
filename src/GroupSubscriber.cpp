#include <ros/ros.h>
#include <openhab_msgs/GroupState.h>

class GroupSubscriber {
public:
    GroupSubscriber(std::string item_name) : item_name_(item_name), enable_(false) {
        // Create a subscriber with appropriate topic, custom message and name of callback function.
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &GroupSubscriber::callback, this);
    }

    void start() {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &GroupSubscriber::callback, this);
    }

    void callback(const openhab_msgs::GroupState::ConstPtr& data) {
        // Simply print out values in our custom message.
        data_ = data;
        std::string msg;
        if (!data_->isnull) {
            msg = "Received " + data_->item;
        } else {
            msg = "Received " + data_->item + " with NULL";
        }
        ROS_INFO_STREAM(ros::this_node::getName() << ": " << msg);
    }

    void stop() {
        enable_ = false;
        sub_.shutdown();
    }

private:
    std::string item_name_;
    bool enable_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    openhab_msgs::GroupState::ConstPtr data_;
};

int main(int argc, char** argv) {
    // Initialize the node and name it.
    std::string node_name = "GroupSubscriberNode";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    GroupSubscriber groupSubscriber("Static");

    // Go to the main loop
    try {
        groupSubscriber.start();
        // Wait for messages on topic, go to callback function when new messages arrive.
        // spin() simply keeps C++ from exiting until this node is stopped
        ros::spin();
    } catch (const ros::Exception& e) {
        groupSubscriber.stop();

        std::vector<std::string> nodes;
        ros::master::getNodes(nodes);
        for (const auto& node : nodes) {
            system(("rosnode kill " + node_name).c_str());
        }

        ROS_INFO_STREAM(ros::this_node::getName() << ": Node stopped");
    }

    return 0;
}
