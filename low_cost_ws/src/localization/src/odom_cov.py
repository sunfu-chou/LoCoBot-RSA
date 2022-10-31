#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def cb(msg):
  msg.twist.covariance = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, 0.0, 0.0, 0.0, 0.0, 5.0]
  odom_pub.publish(msg)
  
if __name__ == '__main__':
  rospy.init_node('uwb_localization',anonymous=False)
  odom_sub = rospy.Subscriber('odom', Odometry, cb, queue_size=10)
  odom_pub = rospy.Publisher('odom/cov', Odometry, queue_size=10)
  rospy.spin()
