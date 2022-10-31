#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def cb(msg):
  msg.linear_acceleration_covariance = [0.5, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0]
  msg.angular_velocity_covariance = [0.0, 0.0, 0.0, 
                                     0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.1]
  odom_pub.publish(msg)
  
if __name__ == '__main__':
  rospy.init_node('uwb_localization',anonymous=False)
  odom_sub = rospy.Subscriber('mobile_base/sensors/imu_data', Imu, cb, queue_size=10)
  odom_pub = rospy.Publisher('mobile_base/sensors/imu_data/cov', Imu, queue_size=10)
  rospy.spin()
