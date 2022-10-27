#!/usr/bin/env python3
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import *
from digi.xbee.models.address import *
from digi.xbee.packets.base import DictKeys
from digi.xbee.packets.common import ATCommPacket, ATCommResponsePacket
from digi.xbee.models.mode import OperatingMode
from datetime import datetime



class xbee_image_sender:
	def __init__(self):
		self.resize = rospy.get_param("~resize", 0.15)
		self.use_text = rospy.get_param("~use_text", True)
		self.remoteAddr = rospy.get_param("~remoteAddr", "4109910A")
		self.bridge = CvBridge()
		self.stop = False
		
		self.PORT = rospy.get_param("~port", "/dev/ttyUSB0")
		self.BAUD_RATE = 115200
		self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
		self.device.open(force_settings=True)
		self.sourceAddr = str(self.device.get_64bit_addr())

		self.device.add_data_received_callback(self.xbee_callback)
		self.image_sub = rospy.Subscriber(rospy.get_param("~input_topic","/camera/color/image_raw"),Image,self.callback,queue_size=1, buff_size=2**24)
		print("xbee image node initialized, I am ", self.sourceAddr[8:])

	def callback(self,data):
		text = time.strftime("%H:%M:%S", time.localtime())
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			return

		res = cv2.resize(cv_image, (int(cv_image.shape[1] * self.resize) ,int(cv_image.shape[0] * self.resize)), interpolation=cv2.INTER_AREA)
		if self.use_text : cv2.putText(res, text, (int(res.shape[0]*0.15), int(res.shape[1]*0.15)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
		print("callback image_raw", datetime.now())
		print("origin shape",cv_image.shape[0],cv_image.shape[1], "resize shape: ",res.shape[0],res.shape[1])
		self.xbee_encode_and_send(self.remoteAddr, res)
		print('Sending Done', datetime.now())


	def xbee_encode_and_send(self, ADDRESS_L, image_data):
		ADDRESS_H = '0013A200'
		ADDRESS = ADDRESS_H + ADDRESS_L

		# send data to bytearray
		byte_arr = bytearray()
		for i in range(image_data.shape[0]):
			for j in range(image_data.shape[1]):
				for k in range(image_data.shape[2]):
					byte_arr.extend( int(image_data[i][j][k]).to_bytes(1, byteorder='big', signed=False) )

		length, index, check= int(len(byte_arr)), 0, 0
		print("bytearray len: ",length)

		for index in range(0,length,247) :
			if self.stop == True:
				rospy.logerr("remote error")
				self.stop = False
				time.sleep(1)
				return

			pack = bytearray()
			pack.extend(length.to_bytes(4, byteorder='big', signed=False) )  # bytes
			pack.extend(image_data.shape[0].to_bytes(2, byteorder='big', signed=False) )  # image height
			pack.extend(image_data.shape[1].to_bytes(2, byteorder='big', signed=False) )  # image width
			index_end = index+247 if index+247 < length else length
			pack.extend( byte_arr[index:(index_end)] ) #data

			check = 0xff & (check + pack[-1])
			pack.extend(check.to_bytes(1, byteorder='big', signed=False)) # checksum

			try:
				self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
			except XBeeException:
				rospy.logerr('digi.xbee.exception.XBeeException: XBee devices serial port closed')
				length, index, check= int(len(byte_arr)), 0, 0
				return False
			except TransmitException:
				rospy.logerr('digi.xbee.exception.TransmitException: There was a problem with a transmitted packet response')
				length, index, check= int(len(byte_arr)), 0, 0
				return False
			except TimeoutException:
				rospy.logerr('digi.xbee.exception.TimeoutException: Response not received in the configured timeout.')
				length, index, check= int(len(byte_arr)), 0, 0
				return False
			except :
				rospy.logerr('send data_via_xbee fail')
				length, index, check= int(len(byte_arr)), 0, 0
				return False

		return True

	def xbee_callback(self, xbee_message):
		if xbee_message.data == bytearray(b'remote error'):
			if self.stop == False: self.stop = True


if __name__ == '__main__':
	rospy.init_node('xbee_image_sender', anonymous=True)
	sender = xbee_image_sender()
	rospy.spin()
