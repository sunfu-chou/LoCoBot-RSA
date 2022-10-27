#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped


class pose_copy(object):
	def __init__(self):
		topic_name = rospy.get_param("~veh")
		topic_name = topic_name + '/robot_pose'
		self.I_am_base_station = rospy.get_param("~I_am_base_station")
		if not self.I_am_base_station:
			self.pub_pose = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
			self.sub_pose = rospy.Subscriber("/slam_pose", PoseStamped, self.cb_pose, queue_size=1)

	def cb_pose(self, msg):
		if not self.I_am_base_station:
			self.pub_pose.publish(msg)


if __name__ == "__main__":
	rospy.init_node("pose_copy")
	pc = pose_copy()
	rospy.spin()
