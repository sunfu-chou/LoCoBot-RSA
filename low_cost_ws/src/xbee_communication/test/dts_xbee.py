#!/usr/bin/env python3

import numpy as np
from xbee_communication.srv import xbee
import struct
import roslib
import pickle
import time
import yaml
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import *
from digi.xbee.models.address import *
from digi.xbee.packets.base import DictKeys
from digi.xbee.packets.common import ATCommPacket, ATCommResponsePacket
from digi.xbee.models.mode import OperatingMode
from datetime import datetime

class XBeeTest():
	def __init__(self):
		# Setup the node
		path = "/home/bory/duckiepond-nctu/catkin_ws/src/communication/xbee_communication/config/dts_xbee.yaml"
		#print(path)
		with open (path,"r") as stream:
			xbee_list = yaml.load(stream)
		# Setup xbee device
		self.PORT = "/dev/ttyUSB0"
		self.BAUD_RATE = 115200
		self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
		self.device.open(force_settings=True)
		self.sourceAddr = str(self.device.get_64bit_addr())
		test_Setup_xbee_device = True
		time.sleep(0.5)
        #self.xbee_test(xbee_list["boat1"])
        #self.xbee_test(xbee_list["boat2"])   
        #self.xbee_test(xbee_list["boat3"])
		self.xbee_test("41AF1BEE")    
      




	def xbee_test(self,robot_address):
		ADDRESS_H = '0013A200'
		ADDRESS = ADDRESS_H + robot_address
		pack = pickle.dumps( [] )
		try :
			self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
			test_xbee_send_result = True
		except XBeeException:
			print('digi.xbee.exception.XBeeException: XBee devices serial port closed')
			return False
		except TransmitException:
			print('digi.xbee.exception.TransmitException: There was a problem with a transmitted packet response')
			return False
		except TimeoutException:
			print('digi.xbee.exception.TimeoutException: Response not received in the configured timeout.')
			return False
		except :
			print('send data_via_xbee fail')
			return False
		

		print(test_xbee_send_result)
		if (test_xbee_send_result):
			print(ADDRESS)
			print("xbee ok")
		time.sleep(0.5)


if __name__ == '__main__':
	xbee_test = XBeeTest()
	