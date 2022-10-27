#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import  Joy
from xbee_communication.srv import xbee
import struct
import roslib
import pickle
import time
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import *
from digi.xbee.models.address import *
from digi.xbee.packets.base import DictKeys
from digi.xbee.packets.common import ATCommPacket, ATCommResponsePacket
from digi.xbee.models.mode import OperatingMode
from datetime import datetime


class XBeeJoy(object):
	def __init__(self):
		self.PORT = rospy.get_param("~port")
		self.BAUD_RATE = 115200
		self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
		self.device.open(force_settings=True)
		self.sourceAddr = str(self.device.get_64bit_addr())

		IS_MASTER = rospy.get_param("~I_am_master")
		if IS_MASTER:
			self.address_list = {	"A":rospy.get_param("/xbee_joy/A"),\
								 	"B":rospy.get_param("/xbee_joy/B"),\
									"C":rospy.get_param("/xbee_joy/C"),\
									"D":rospy.get_param("/xbee_joy/D"),\
									"E":rospy.get_param("/xbee_joy/E"),\
									"F":rospy.get_param("/xbee_joy/F")	}

			self.remote_address = self.address_list[ rospy.get_param("~remote_robot") ]
			self.sub_joy = rospy.Subscriber(rospy.get_param("~input_topic"), Joy, self.cb_joy, queue_size=1)
			print("xbee node initialized (Master), I am ", self.sourceAddr[8:], 'remote ',self.remote_address)
		if not IS_MASTER :
			self.pub_joy = rospy.Publisher(rospy.get_param("~output_topic"), Joy, queue_size=1)
			self.device.add_data_received_callback(self.xbee_callback)
			self.count = 0
			print("xbee node initialized (Slave), I am ", self.sourceAddr[8:])

	def cb_joy(self,msg):
		ADDRESS_H = '0013A200'
		ADDRESS = ADDRESS_H + self.remote_address
		print(ADDRESS)
		axes = msg.axes
		buttons = msg.buttons
		rospy.loginfo(axes)
		rospy.loginfo(buttons)
		pack = []
		for i in range(len(axes)) : pack.append(axes[i])
		for i in range(len(buttons)) : pack.append(buttons[i])
		pack = pickle.dumps( pack )
		try:
			self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
		except XBeeException:
			rospy.logerr('digi.xbee.exception.XBeeException: XBee devices serial port closed')
		except TransmitException:
			rospy.logerr('digi.xbee.exception.TransmitException: There was a problem with a transmitted packet response')
		except TimeoutException:
			rospy.logerr('digi.xbee.exception.TimeoutException: Response not received in the configured timeout.')
		except :
			rospy.logerr('send data_via_xbee fail')

	def xbee_callback(self,xbee_message):
		rospy.loginfo('xbeejoy callback')
		self.count += 1
		get_msg = pickle.loads(xbee_message.data)
		axes = get_msg[0:8]
		buttons = get_msg[8:]
		rospy.loginfo(axes)
		rospy.loginfo(buttons)

		msg = Joy()
		msg.header.seq = self.count
		msg.header.frame_id = "/dev/input/js0"
		msg.header.stamp = rospy.Time.now()
		msg.axes = axes
		msg.buttons = buttons
		self.pub_joy.publish(msg)



if __name__ == "__main__":
	rospy.init_node("xbeeJoy")
	xbeeJoy = XBeeJoy()
	rospy.spin()
