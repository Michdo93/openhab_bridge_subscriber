#include <ros/ros.h>
#include <openhab_msgs/ColorState.h>

class ColorSubscriber {
public:
    ColorSubscriber(std::string item_name) : item_name_(item_name) {
        // Create a subscriber with appropriate topic, custom message and name of
        // callback function.
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 10, &ColorSubscriber::callback, this);

        // Initialize message variables.
        enable_ = false;
        data_ = NULL;

        if (enable_) {
            start();
        } else {
            stop();
        }
    }

    void start() {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 10, &ColorSubscriber::callback, this);
    }

    void stop() {
        enable_ = false;
        sub_.shutdown();
    }

    void callback(const openhab_msgs::ColorState& data) {
        // Simply print out values in our custom message.
        data_ = data;
        if (!data_.isnull) {
            ROS_INFO("Received %s with hue %f, saturation %f and brightness %f", data_.item.c_str(), data_.hue, data_.saturation, data_.brightness);
        } else {
            ROS_INFO("Received %s with NULL", data_.item.c_str());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string item_name_;
    bool enable_;
    openhab_msgs::ColorState data_;
};

int main(int argc, char** argv) {
    // Initialize the node and name it.
    std::string node_name = "ColorSubscriberNode";
    ros::init(argc, argv, node_name);

    ColorSubscriber colorSubscriber("testColor");

    // Wait for messages on topic, go to callback function when new messages arrive.
    // spin() simply keeps C++ from exiting until this node is stopped
    ros::spin();

    return 0;
}
