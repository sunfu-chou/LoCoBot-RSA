#!/usr/bin/env python3
import rospy
import unittest
import rostest
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

class XBeeJoyTester(unittest.TestCase):
	def setup(self):
		# Setup the node
		rospy.init_node('XBeeTester_node', anonymous=False)
		# Setup xbee device
		try :
			self.PORT = rospy.get_param("~port_test")
			self.BAUD_RATE = 115200
			self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
			self.device.open(force_settings=True)
			self.sourceAddr = str(self.device.get_64bit_addr())
			test_Setup_xbee_device = True
		except:
			test_Setup_xbee_device = True
		self.assertEqual(test_Setup_xbee_device, True, "Tester Setup xbee device fail")


	def test_xbee_Joy_msg(self):
		self.setup()
		self.axes = [0.7212857007980347, 0.546527087688446, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0]
		self.buttons = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]

		pack = []
		for i in range(len(self.axes)) : pack.append(self.axes[i])
		for i in range(len(self.buttons)) : pack.append(self.buttons[i])
		pack = pickle.dumps( pack )


		try :
			self.device.send_data_broadcast(pack)
			test_xbee_send_result = True
		except:
			test_xbee_send_result = False
		self.assertEqual(test_xbee_send_result, True, "Tester XBee send massege fail")

if __name__ == '__main__':
	rostest.rosrun('xbee_communication', 'XBeeTester_node', XBeeJoyTester)
