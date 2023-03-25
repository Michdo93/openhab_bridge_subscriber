#include <ros/ros.h>
#include <openhab_msgs/RollershutterState.h>

class RollershutterSubscriber {
public:
    RollershutterSubscriber(std::string item_name) : item_name_(item_name), enable_(false) {
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &RollershutterSubscriber::callback, this);
    }

    void start() {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &RollershutterSubscriber::callback, this);
    }

    void callback(const openhab_msgs::RollershutterState::ConstPtr& msg) {
        data_ = msg;
        if (!data_->isnull) {
            std::string state = "";
            if (data_->isstate) {
                state = data_->state;
            } else if (data_->ispercentage) {
                state = "percentage " + std::to_string(data_->percentage);
            }
            std::string log_msg = "Received " + data_->item + " with state " + state + ", isstate " + std::to_string(data_->isstate) + ", ispercentage " + std::to_string(data_->ispercentage) + " and percentage " + std::to_string(data_->percentage);
            ROS_INFO("%s", log_msg.c_str());
        } else {
            std::string log_msg = "Received " + data_->item + " with NULL";
            ROS_INFO("%s", log_msg.c_str());
        }
    }

    void stop() {
        enable_ = false;
        sub_.shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string item_name_;
    bool enable_;
    openhab_msgs::RollershutterState::ConstPtr data_;
};

int main(int argc, char** argv) {
    // Initialize the node and name it.
    ros::init(argc, argv, "RollershutterSubscriberNode");
    ros::NodeHandle nh("~");

    RollershutterSubscriber rollershutter_subscriber("testRollershutter");

    // Go to the main loop
    try {
        rollershutter_subscriber.start();
        // Wait for messages on topic, go to callback function when new messages arrive.
        // spin() simply keeps ros from exiting until this node is stopped
        ros::spin();
    // Stop with Ctrl + C
    } catch (const std::exception& e) {
        rollershutter_subscriber.stop();

        system("rosnode list > nodes.txt");
        std::ifstream nodes_file("nodes.txt");
        std::string node;
        while (std::getline(nodes_file, node)) {
            system(("rosnode kill " + node).c_str());
        }
        nodes_file.close();
        system("rm nodes.txt");

        ROS_INFO("Node stopped");
    }

    return 0;
}
