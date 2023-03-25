#include <ros/ros.h>
#include "openhab_msgs/DimmerState.h"

class DimmerSubscriber {

public:
    DimmerSubscriber(const std::string& item_name) :
        item_name_(item_name),
        enable_(false)
    {
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &DimmerSubscriber::callback, this);
    }

    void start() {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &DimmerSubscriber::callback, this);
    }

    void stop() {
        enable_ = false;
        sub_.shutdown();
    }

private:
    void callback(const openhab_msgs::DimmerState& msg) {
        data_ = msg;
        std::string msg_str;
        if (!data_.isnull) {
            msg_str = "Received " + data_.item + " with state " + std::to_string(data_.state);
        } else {
            msg_str = "Received " + data_.item + " with NULL";
        }
        ROS_INFO_STREAM(ros::this_node::getName() << msg_str);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string item_name_;
    bool enable_;
    openhab_msgs::DimmerState data_;

};

int main(int argc, char** argv) {
    // Initialize the node and name it
    std::string node_name = "DimmerSubscriberNode";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string item_name;
    nh_private.param<std::string>("item_name", item_name, "testDimmer");

    DimmerSubscriber dimmer_subscriber(item_name);

    // Go to the main loop
    try {
        dimmer_subscriber.start();
        ros::spin();
    } catch (const ros::Exception& e) {
        dimmer_subscriber.stop();
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
    }

    return 0;
}
