#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped

class Publisher():
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)

        #publisher
        self.pub_s = rospy.Publisher("/test/", String, queue_size=1)

        self.string = "Hello world!!"
        self.rate = rospy.Duration(0.1)

        while not rospy.is_shutdown():
            self.run()
            rospy.sleep(self.rate)
            
    def run(self):
        self.pub_s.publish(self.string)

if __name__=='__main__':
    rospy.init_node("Test_Publisher")
    publisher = Publisher()
    rospy.spin()