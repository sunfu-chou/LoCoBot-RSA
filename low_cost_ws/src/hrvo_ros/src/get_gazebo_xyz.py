#!/usr/bin/env python3
import rospy
import time
import numpy as np
import math
import random
import sys
import os
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState, GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest
import csv

class xyz():
	def __init__(self):
		super().__init__()
		rospy.init_node('GazeboXYZ')
		self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.sub_joy = rospy.Subscriber("joy", Joy, self.cb_joy, queue_size=1)
		self.list = []
		self.flag = False



	def cb_joy(self, msg):
		if (msg.axes[-1] == -1) :
			if not self.flag :
				self.flag = True
				agent = ModelState()
				rospy.wait_for_service('/gazebo/get_model_state')
				try:
					agent = self.get_model('X1', '')
				except (rospy.ServiceException) as e:
					print(e)
					return

				print(agent.pose.position.x, agent.pose.position.y, agent.pose.position.z)
				self.list.append([agent.pose.position.x, agent.pose.position.y, agent.pose.position.z])
				with open('GazeboXYZ.csv', 'w', newline='') as csvFile:
					writer = csv.writer(csvFile)
					for i in range(len(self.list)):
						writer.writerow(self.list[i])
				time.sleep(1)

		else : self.flag = False
		

if __name__ == "__main__":
	getxyz = xyz()
	rospy.spin()