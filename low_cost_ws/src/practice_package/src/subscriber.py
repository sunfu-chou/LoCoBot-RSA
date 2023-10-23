#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped

class Subscriber():
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)

        #subscriber
        self.sub_s = rospy.Subscriber("/test/", String, self.cb_test)

        self.string = "Publisher says : "
        self.rate = rospy.Duration(1)

        while not rospy.is_shutdown():
            rospy.sleep(self.rate)

    def cb_test(self, msg):
        print(self.string + msg.data + '\n')

if __name__=='__main__':
    rospy.init_node("Test_Subscriber")
    subscriber = Subscriber()
    rospy.spin()