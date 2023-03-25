#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <openhab_msgs/ImageState.h>

bool VERBOSE = false;

class ImageSubscriber
{
public:
    ImageSubscriber(const std::string& item_name) : item_name_(item_name), enable_(false)
    {
        // Create a subscriber with appropriate topic, custom message and name of
        // callback function.
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &ImageSubscriber::callback, this);
    }

    void start()
    {
        enable_ = true;
        sub_ = nh_.subscribe("/openhab/items/" + item_name_ + "/state", 1, &ImageSubscriber::callback, this);
    }

    void callback(const openhab_msgs::ImageState::ConstPtr& data)
    {
        if (data->isnull == false)
        {
            ROS_INFO_STREAM("Received " << data->item);
            try
            {
                image_ = cv_bridge::toCvCopy(data->state, "bgr8")->image;
            }
            catch (const cv_bridge::Exception& e)
            {
                ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
                return;
            }

            cv::imshow(item_name_, image_);
            cv::waitKey(25);
        }
        else
        {
            ROS_INFO_STREAM("Received " << data->item << " with NULL");
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
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    cv::Mat image_;
};

int main(int argc, char** argv)
{
    // Initialize the node and name it.
    std::string node_name = "ImageSubscriberNode";
    ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ImageSubscriber imageSubscriber("testImage");

    // Go to the main loop
    try
    {
        imageSubscriber.start();
        // Wait for messages on topic, go to callback function when new messages arrive.
        // spin() simply keeps ros from exiting until this node is stopped
        ros::spin();
    }
    // Stop with Ctrl + C
    catch (const std::exception&)
    {
        imageSubscriber.stop();

        std::vector<std::string> nodes;
        ros::master::getNodes(nodes);
        for (const auto& node : nodes)
        {
            ros::NodeHandle nh(node);
            std::string node_ns = nh.getNamespace();
            if (node_ns == ros::this_node::getNamespace())
            {
                ros::service::waitForService("rosout/get_loggers");
                ros::ServiceClient client = nh.serviceClient<rosgraph_msgs::Log>("rosout/get_loggers");
                rosgraph_msgs::Log log;
                log.request.level = rosgraph_msgs::Log::DEBUG;
                client.call(log);
                if (log.response.loggers[0].name == "/rosout")
                {
                    system(("rosnode kill " + node).c_str());
                }
            }
        }
        ROS_INFO("Node stopped");
    }

    return 0;
}
