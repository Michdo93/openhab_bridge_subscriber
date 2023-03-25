#include <ros/ros.h>
#include <string>
#include <openhab_msgs/SwitchState.h>

class SwitchSubscriber {

public:
    SwitchSubscriber(const std::string& item_name) {
        item_name_ = item_name;

        /* Configure subscriber */
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &SwitchSubscriber::callback, this);

        /* Initialize message variables */
        enable_ = false;
        data_ = nullptr;

        if (enable_) {
            start();
        } else {
            stop();
        }
    }

    void start() {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &SwitchSubscriber::callback, this);
    }

    void callback(const openhab_msgs::SwitchState::ConstPtr& data) {
        /* Handle subscriber data */
        data_ = data;
        std::string msg;
        if (data_->isnull == false) {
            msg = "Received " + data_->item + " with state " + data_->state;
        } else {
            msg = "Received " + data_->item + " with NULL";
        }
        ROS_INFO("%s", msg.c_str());
    }

    void stop() {
        /* Turn off subscriber */
        enable_ = false;
        sub_.shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string item_name_;
    bool enable_;
    openhab_msgs::SwitchState::ConstPtr data_;
};

int main(int argc, char** argv) {
    /* Initialize the node and name it */
    const std::string node_name = "SwitchSubscriberNode";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    SwitchSubscriber switch_subscriber("testSwitch");

    /* Go to the main loop */
    try {
        switch_subscriber.start();
        /* Wait for messages on topic, go to callback function when new messages arrive.
        spin() simply keeps cpp from exiting until this node is stopped */
        ros::spin();
    } 
    /* Stop with Ctrl + C */
    catch (const std::exception& e) {
        switch_subscriber.stop();

        std::vector<std::string> nodes;
        std::string command = "rosnode list";
        FILE* stream = popen(command.c_str(), "r");
        if (stream) {
            const int max_buffer = 256;
            char buffer[max_buffer];
            while (!feof(stream)) {
                if (fgets(buffer, max_buffer, stream) != NULL) {
                    nodes.push_back(buffer);
                }
            }
            pclose(stream);
        }

        for (const auto& node : nodes) {
            std::string kill_command = "rosnode kill " + node_name;
            system(kill_command.c_str());
        }

        ROS_INFO("Node stopped");
    }

    return 0;
}
