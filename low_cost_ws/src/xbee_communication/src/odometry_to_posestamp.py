#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
import struct
import roslib
import pickle
import time
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from datetime import datetime


class Odom2Pose(object):
	def __init__(self):
		self.sub_odom = rospy.Subscriber(rospy.get_param("~odom"), Odometry, self.cb_odom)
		self.pub_pose = rospy.Publisher(rospy.get_param("~pose"), PoseStamped, queue_size=1)


	def cb_odom(self, data):
		pose = PoseStamped()
		pose.header.frame_id = "map"
		pose.pose.position.x = float(data.pose.pose.position.x)
		pose.pose.position.y = float(data.pose.pose.position.y)
		pose.pose.position.z = float(data.pose.pose.position.z)
		pose.pose.orientation.x = float(data.pose.pose.orientation.x)
		pose.pose.orientation.y = float(data.pose.pose.orientation.y)
		pose.pose.orientation.z = float(data.pose.pose.orientation.z)
		pose.pose.orientation.w = float(data.pose.pose.orientation.w)

		self.pub_pose.publish(pose)




if __name__ == "__main__":
	rospy.init_node("Odom2Pose")
	Odom22222222222Pose = Odom2Pose()
	rospy.spin()
