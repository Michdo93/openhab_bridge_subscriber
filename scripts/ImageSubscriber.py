#!/usr/bin/python
import os
import rospy
from openhab_msgs.msg import ImageCommand
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# OpenCV
import cv2

VERBOSE = False

class ImagePublisher(object):
    """Node example class."""

    def __init__(self, item_name, image_path):
        self.image_path = image_path
        self.item_name = item_name
        self.pub = rospy.Publisher("/openhab/items/" + self.item_name + "/command", ImageCommand, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # Initialize message variables.
        self.enable = False
        self.message = None
        self.bridge = CvBridge()
        self.image = None

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

            if os.path.isfile(self.image_path):
                self.message.isnull = False
                img = cv2.imread(self.image_path)
            else:
                self.message.isnull = True
                self.image_path = "NULL"
                img = numpy.zeros([200,200,3])

                img[:,:,0] = numpy.ones([200,200])*255
                img[:,:,1] = numpy.ones([200,200])*255
                img[:,:,2] = numpy.ones([200,200])*0

            # finally convert RGB image to BGR for opencv
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            try:
                self.image = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            except CvBridgeError as e:
                print(e)

            self.message.command = self.image
            self.message.header.stamp = rospy.Time.now()
            self.message.header.frame_id = "/base_link"
            self.message.item = self.item_name

            message = "Publishing %s to %s at %s" % (self.image_path, self.message.item, rospy.get_time())
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

    image_path = str(args.image)

    if image_path is None:
        image_path = "Image.jpg"

    imagePublisher = ImagePublisher("testImage", image_path)

    try:
        imagePublisher.start()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
