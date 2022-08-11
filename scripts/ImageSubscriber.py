#!/usr/bin/python
import rospy
from openhab_msgs.msg import ImageCommand
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# OpenCV
import cv2

VERBOSE = False

class ImagePublisher(object):
    """Node example class."""

    def __init__(self, item_name):

        self.item_name = item_name
        self.pub = rospy.Publisher("/openhab/items/" + self.item_name + "/command", ImageCommand, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # Initialize message variables.
        self.enable = False
        self.message = None

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher."""
        self.enable = True
        self.pub = rospy.Publisher("/openhab/items/" + self.item_name + "/command", ImageCommand, queue_size=10)

        while not rospy.is_shutdown():

            self.message = ImageCommand()

            self.message.command = "Hello World %s" % rospy.get_time()

            self.message.isnull = False

            self.message.header.stamp = rospy.Time.now()
            self.message.header.frame_id = "/base_link"
            self.message.item = self.item_name

            message = "Publishing to %s at %s: command = %s" % (self.message.item, rospy.get_time(), self.message.command)
            rospy.loginfo(message)

            self.pub.publish(self.message)
            self.rate.sleep()

    def stop(self):
        """Turn off publisher."""
        self.enable = False
        self.pub.unregister()

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("ImagePublisherNode", anonymous=True)
    # Go to class functions that do all the heavy lifting.

    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=str, required=False,
                        help="Path to image file you want to publish to openHAB. If not given an Image.jpg in your current path is expected. If there is no Image NULL will be published.")
    args = parser.parse_args()

    image_path = str(args.port)

    if image_path is None:
        image_path = "Image.jpg"

    imagePublisher = ImagePublisher("testImage")

    try:
        imagePublisher.start()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
