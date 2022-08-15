#!/usr/bin/python
import os
import rospy
from openhab_msgs.msg import ImageState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# OpenCV
import cv2

VERBOSE = False

class ImageSaver(object):

    def __init__(self, item_name):
        self.item_name = item_name

        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.sub = rospy.Subscriber("/openhab/items/" + self.item_name + "/state", ImageState, self.callback)

        # Initialize message variables.
        self.enable = False
        self.data = None
        self.bridge = CvBridge()
        self.image = None
        self.i = 0
        self.name = "%s_%s_%s.jpg" % (self.item_name, rospy.Time.now(), self.i)

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber("/openhab/items/" + self.item_name + "/state", ImageState, self.callback)

    def callback(self, data):
        """Handle subscriber data."""

        self.data = data

        if self.data.isnull == False:
            msg = "Received %s" % self.data.item

            try:
                self.image = self.bridge.imgmsg_to_cv2(self.data.state, 'bgr8')
            except CvBridgeError as e:
                print(e)

            self.i = self.i + 1
            cv2.imwrite(self.name, self.image)
        else:
            msg = "Received %s with NULL" % self.data.item
        rospy.loginfo(rospy.get_caller_id() + msg)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = "ImageSaverNode"
    rospy.init_node(node_name, anonymous=True)

    imageSaver = ImageSaver("testImage")

    # Go to the main loop
    try:
        imageSaver.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        imageSaver.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)


        print("Node stopped")
