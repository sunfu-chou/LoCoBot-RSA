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

class XBeeTest(unittest.TestCase):
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
			time.sleep(0.5)
		except:
			test_Setup_xbee_device = True
		self.assertEqual(test_Setup_xbee_device, True, "Tester Setup xbee device fail")


	def test_nano2(self):
		self.setup()
		ADDRESS_H = '0013A200'
		ADDRESS = ADDRESS_H + rospy.get_param("/xbee_joy/A")
		pack = pickle.dumps( [] )
		try :
			self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
			test_xbee_send_result = True
		except:
			test_xbee_send_result = False
		self.assertEqual(test_xbee_send_result, True,  "nano2 xbee connect fail (Joy)" + rospy.get_param("/xbee_joy/A"))
		time.sleep(0.5)

		ADDRESS = ADDRESS_H + rospy.get_param("/xbee_address/husky1")
		pack = pickle.dumps( [] )
		try :
			self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
			test_xbee_send_result = True
		except:
			test_xbee_send_result = False
		self.assertEqual(test_xbee_send_result, True,  "nano2 xbee connect fail (Pose)" + rospy.get_param("/xbee_address/husky1"))
		time.sleep(0.5)

	def test_nano3(self):
		self.setup()
		ADDRESS_H = '0013A200'
		ADDRESS = ADDRESS_H + rospy.get_param("/xbee_joy/B")
		pack = pickle.dumps( [] )
		try :
			self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
			test_xbee_send_result = True
		except:
			test_xbee_send_result = False
		self.assertEqual(test_xbee_send_result, True, "nano3 xbee connect fail (Joy) " + rospy.get_param("/xbee_joy/B"))
		time.sleep(0.5)

		ADDRESS = ADDRESS_H + rospy.get_param("/xbee_address/husky2")
		pack = pickle.dumps( [] )
		try :
			self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
			test_xbee_send_result = True
		except:
			test_xbee_send_result = False
		self.assertEqual(test_xbee_send_result, True,  "nano2 xbee connect fail (Pose) " + rospy.get_param("/xbee_address/husky2"))
		time.sleep(0.5)

if __name__ == '__main__':
	rostest.rosrun('xbee_communication', 'XBeeTest', XBeeTest)