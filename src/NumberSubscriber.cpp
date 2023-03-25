#include <ros/ros.h>
#include <openhab_msgs/NumberState.h>

class NumberSubscriber {
public:
    NumberSubscriber(std::string item_name) : item_name_(item_name), enable_(false), data_(nullptr) {
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &NumberSubscriber::callback, this);
        if (enable_) {
            start();
        } else {
            stop();
        }
    }

    void start() {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &NumberSubscriber::callback, this);
    }

    void callback(const openhab_msgs::NumberState::ConstPtr& msg) {
        data_ = msg;
        if (data_->isnull == false) {
            ROS_INFO_STREAM("Received " << data_->item << " with state " << data_->state);
        } else {
            ROS_INFO_STREAM("Received " << data_->item << " with NULL");
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
    openhab_msgs::NumberState::ConstPtr data_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "NumberSubscriberNode", ros::init_options::AnonymousName);

    NumberSubscriber number_subscriber("testNumber");

    try {
        number_subscriber.start();
        ros::spin();
    } catch (const ros::Exception& e) {
        ROS_ERROR_STREAM("Error occurred: " << e.what());
        number_subscriber.stop();

        system(("rosnode kill " + ros::this_node::getName()).c_str());
        ROS_INFO_STREAM("Node stopped");
    }

    return 0;
}
