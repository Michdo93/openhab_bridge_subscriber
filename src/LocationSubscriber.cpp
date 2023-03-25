#include <ros/ros.h>
#include "openhab_msgs/LocationState.h"

class LocationSubscriber {
public:
    LocationSubscriber(const std::string& item_name)
        : item_name_(item_name), enable_(false)
    {
        // Create a subscriber with appropriate topic, custom message and name of callback function.
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 10, &LocationSubscriber::callback, this);
    }

    void start()
    {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 10, &LocationSubscriber::callback, this);
    }

    void callback(const openhab_msgs::LocationState& data)
    {
        // Simply print out values in our custom message.
        if (data.isnull == false) {
            ROS_INFO_STREAM("Received " << data.item << " with longitude " << data.longitude << ", latitude " << data.latitude << " and altitude " << data.altitude);
        } else {
            ROS_INFO_STREAM("Received " << data.item << " with NULL");
        }
    }

    void stop()
    {
        enable_ = false;
        sub_.shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string item_name_;
    bool enable_;
};

int main(int argc, char** argv)
{
    // Initialize the node and name it.
    ros::init(argc, argv, "LocationSubscriberNode");
    ros::NodeHandle nh;

    LocationSubscriber location_subscriber("testLocation");

    // Go to the main loop
    try {
        location_subscriber.start();
        // Wait for messages on topic, go to callback function when new messages arrive.
        // spin() simply keeps C++ from exiting until this node is stopped
        ros::spin();
    }
    // Stop with Ctrl + C
    catch (const std::exception& e) {
        location_subscriber.stop();

        system("rosnode list > /tmp/rosnode_list");
        FILE* f = fopen("/tmp/rosnode_list", "r");
        char node_name[1024];
        while (fgets(node_name, sizeof(node_name), f)) {
            node_name[strlen(node_name)-1] = '\0';
            system(std::string("rosnode kill " + std::string(node_name)).c_str());
        }
        fclose(f);

        ROS_INFO_STREAM("Node stopped");
    }

    return 0;
}
