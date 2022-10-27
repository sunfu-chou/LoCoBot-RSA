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
        
		self.device.add_data_received_callback(self.xbee_callback_and_decode)
 
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


	def xbee_callback_and_decode(self, xbee_message):
		ADDRESS = str(xbee_message.remote_device.get_64bit_addr()) # msg from who
		if not self.last_remote == ADDRESS : self.clear()
		self.last_remote = ADDRESS

		if not xbee_message.data[0:1] == b'\xAB' : # Header wrong
			self.clear()
			print('get xbee_message with wrong Header')
			return

		if not ((xbee_message.data[1:2] == b'\x00') or (xbee_message.data[1:2] == b'\x01') or (xbee_message.data[1:2] == b'\x02') or (xbee_message.data[1:2] == b'\x03')):
			self.clear()
			print('get xbee_message with wrong type')
			return

		self.get_register.extend(xbee_message.data[6:])
		self.data_bytes = int.from_bytes(xbee_message.data[2:6], byteorder="big",signed=False)


		if (self.data_bytes == len(self.get_register) -1): # the last one of data pkgs
                    #print(xbee_message.data[1:2])
                    if xbee_message.data[1:2] == b'\x00': # type: string msg
                        self.sending_break = True
                        get_msg = pickle.loads(self.get_register[:-1])
                        self.clear()
                        print('get string msg= ',get_msg)
                        if get_msg == "AskTwist":
                            self.get_new_ask = True
                            print(" AskTwist from ", ADDRESS[8:])
                            if self.data_twist == []:
                                self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
                            else :
                                self.xbee_encode_and_send(ADDRESS[8:],'OK', data_type=b'\x00') #send string msg
                                self.xbee_encode_and_send(ADDRESS[8:], self.data_twist, data_type=b'\x01') #send twist
                                self.data_twist = []
                        elif get_msg == "AskPose":
                            self.get_new_ask = True
                            print(" AskPose from ", ADDRESS[8:])
                            if self.data_odom == []:
                                self.xbee_encode_and_send(ADDRESS[8:],'Not OK', data_type=b'\x00') #send string msg
                            else :
                                self.xbee_encode_and_send(ADDRESS[8:],'OK',data_type=b'\x00')
                                self.xbee_encode_and_send(ADDRESS[8:],self.data_odom,data_type=b'\x02')
                                self.data_odom=[]
                                self.ask_robot_data("master","AskTwist")
                        elif get_msg == "OK": self.get_ACK = True
                        elif get_msg == "Not OK": self.get_ACK = "Not OK"
                    elif xbee_message.data[1:2] == b'\x01': # type: points
                        get_points = pickle.loads(self.get_register[:-1])
                        self.clear()
                        pub_msg = self.np_array_to_twist(get_points)
                        print(pub_msg)
                        self.pub_Robot_cmd.publish(pub_msg)
                        self.get_array_checksum = True

                    elif xbee_message.data[1:2] == b'\x02': # type: pose
                        get_odom = pickle.loads(self.get_register[:-1])
                        self.clear()
                        pub_msg = self.np_array_to_Odometry(get_odom)
                        (self.all_Publisher[self.address_to_robot[ADDRESS[8:]]]["AskPose"]).publish(pub_msg)
                        print(get_odom)
                        self.get_array_checksum = True


if __name__ == '__main__':
	xbee_test = XBeeTest()
	