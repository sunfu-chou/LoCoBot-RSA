#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray

from uwb import UWB
import time

uwb = UWB()

pose = PoseWithCovarianceStamped()
distances = Float64MultiArray()

# while True:
#   uwb.localize_2_5D()
#   uwb.range_all()
#   print(uwb.pose[0], end=", ")
#   print(uwb.pose[1], end=", ")
#   print(uwb.pose[2], end=", ")
#   print([devicerange.distance for devicerange in uwb.ranges_all])
#   time.sleep(0.01)

def timer_callback(e):
  now = rospy.Time.now()
  
  uwb.localize_2_5D()
  uwb.range_all()
  pose.header.stamp = now
  pose.header.frame_id = 'map'
  pose.pose.pose.position.x = uwb.pose[0] / 1000.0
  pose.pose.pose.position.y = uwb.pose[1] / 1000.0
  pose.pose.pose.position.z = uwb.pose[2] / 1000.0
  pose.pose.pose.orientation.w = 1.0
  pose.pose.covariance = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]
  
  pose_pub.publish(pose)

  distances.data = [devicerange.distance for devicerange in uwb.ranges_all]

  distances_pub.publish(distances)
  
if __name__ == '__main__':
  rospy.init_node('uwb_localization',anonymous=False)
  uwb.load_env_config(rospy.get_param('~config_file_path'))
  uwb.connect()
  uwb.height = 950
  uwb.write_env_config()
  pose_pub = rospy.Publisher('uwb_pose', PoseWithCovarianceStamped, queue_size=10)
  distances_pub = rospy.Publisher('uwb_distances', Float64MultiArray, queue_size=10)
  localization_timer = rospy.Timer(rospy.Duration(0.05), timer_callback)
  rospy.spin()
