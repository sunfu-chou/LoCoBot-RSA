#!/usr/bin/env python3
import roslib
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import *
from digi.xbee.models.address import *
from digi.xbee.packets.base import DictKeys
from digi.xbee.packets.common import ATCommPacket, ATCommResponsePacket
from digi.xbee.models.mode import OperatingMode
from datetime import datetime


class xbee_image_receiver:
	def __init__(self):
		self.PORT = rospy.get_param("~port", "/dev/ttyUSB1")
		self.BAUD_RATE = 115200
		self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
		self.device.open(force_settings=True)
		self.sourceAddr = str(self.device.get_64bit_addr())

		self.bridge = CvBridge()
		self.check = 0
		self.last_receive = -9999
		self.get_register = bytearray()
		self.height, self.width = 0, 0
		self.image_pub = rospy.Publisher(rospy.get_param("~output_topic","image_resize"),Image,queue_size=1)
		self.device.add_data_received_callback(self.xbee_callback_and_decode)
		self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_check)
		print("xbee_image_receiver node initialized, I am ", self.sourceAddr[8:])

	def timer_check(self,event):
		if time.time() - self.last_receive > 1 :
			rospy.logwarn("No new xbee mesage")
			if len(self.get_register)>0:
				image = self.register2image(self.height, self.width)
				self.cvbridge_pub_image(image)
				self.check = 0
				self.get_register = bytearray()
				rospy.logwarn("have gotten a part package and pub it to ROS topic")

	def register2image(self, height, width):
		image = np.zeros((height, width, 3))
		index = 0
		for i in range(height):
			for j in range(width):
				for k in range(3):
					image[i][j][k] = self.get_register[index]
					index += 1
					if index >= len(self.get_register):
						print(image.shape)
						return image

		print(image.shape)
		return image

	def cvbridge_pub_image(self,image):
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(image.astype(np.uint8), encoding="bgr8"))
		except CvBridgeError as e:
			rospy.logerr(e)

	def xbee_callback_and_decode(self, xbee_message):
		self.last_receive = time.time()
		pack = xbee_message.data
		self.height = int.from_bytes(xbee_message.data[4:6],byteorder="big",signed=False)
		self.width = int.from_bytes(xbee_message.data[6:8],byteorder="big",signed=False)

		self.check = 0xff & (self.check + pack[-2])
		if not self.check == pack[-1] :
			self.check = 0
			self.get_register = bytearray()
			rospy.logerr("xbee pack checksum error")
			try :
				self.device.send_data_64(xbee_message.remote_device.get_64bit_addr(), "remote error")
			except :
				rospy.logerr("xbee disconnect")

			return
		# print("callback",self.check, pack[-1])

		self.get_register.extend(xbee_message.data[8:-1])
		# print(len(self.get_register), int.from_bytes(xbee_message.data[:4],byteorder="big",signed=False))
		if len(self.get_register)==int.from_bytes(xbee_message.data[:4],byteorder="big",signed=False):
			image = self.register2image(self.height, self.width)
			self.cvbridge_pub_image(image)
			self.check = 0
			self.get_register = bytearray()
			rospy.loginfo("have gotten a full package and pub it to ROS topic")




if __name__ == '__main__':
	rospy.init_node('xbee_image_receiver', anonymous=True)
	receiver = xbee_image_receiver()
	rospy.spin()
