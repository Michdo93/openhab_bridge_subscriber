#!/usr/bin/python
import os
import rospy
from openhab_msgs.msg import StringState

class StringSubscriber(object):

    def __init__(self, item_name):
        self.item_name = item_name

        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.sub = rospy.Subscriber("/openhab/items/" + self.item_name + "/state", StringState, self.callback)

        # Initialize message variables.
        self.enable = False
        self.data = None

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber("/openhab/items/" + self.item_name + "/state", StringState, self.callback)

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data
        if self.data.isnull == False:
            msg = "Received %s with state %s" % (self.data.item, self.data.state)
        else:
            msg = "Received %s with NULL" % self.data.item
        rospy.loginfo(rospy.get_caller_id() + msg)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = "StringSubscriberNode"
    rospy.init_node(node_name, anonymous=True)

    stringSubscriber = StringSubscriber("testString")

    # Go to the main loop
    try:
        stringSubscriber.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        stringSubscriber.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)


        print("Node stopped")
