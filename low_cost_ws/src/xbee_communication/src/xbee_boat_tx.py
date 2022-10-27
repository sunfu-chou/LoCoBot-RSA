#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import  Joy
from xbee_communication.srv import xbee
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
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
import subprocess
import yaml
from nav_msgs.msg import Odometry

class XBeeboatTx(object):
    def __init__(self):
        self.PORT = rospy.get_param("~port")
        self.BAUD_RATE = 115200
        self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
        self.device.open(force_settings=True)
        self.sourceAddr = str(self.device.get_64bit_addr())

        #with open('duckiepond-devices-machine.yaml', 'r') as f:
        #        data = yaml.load(f)

        #self.machine,self.compute_unit = self.who_am_I(data)
        #self.xbee_address = self.get_xbee_address(data,self.machine)
        self.machine = "boat6"
        self.xbee_address = "41AF1C2C"

        self.sub_robot_odom = rospy.Subscriber("/wamv3/localization_gps_imu/odometry",Odometry,self.cb_robot_odom,queue_size = 1)
        self.sub_robot_status = rospy.Subscriber("/"+self.machine+"/robotstatus",Float32MultiArray,self.cb_robot_status,queue_size = 1)
        self.robot_status = Float32MultiArray()
        for i in range (10):
            self.robot_status.data.extend([0])
        rospy.Timer(rospy.Duration(1), self.heartbeat)
        print("xbee node initialized (Master), I am ", self.sourceAddr[8:], 'remote ',self.xbee_address)


    def who_am_I(self,data):

            ret_byte = subprocess.check_output(['ifconfig'])
            ret_str = ret_byte.decode('utf-8')
            # Cut string from 'equal symbol' to 'degree C symbol', then convert to float
            en = ret_str[ret_str.find('eno1:'): ret_str.find('lo')]
            ip = en[en.find('inet')+5: en.find('netmask')]
            machine,device = self.get_key(data,ip)
            return machine,device

    def get_key(self,dict,value):
        for k, v in dict.items():
            for v,v1 in v.items():
                for v1,v2 in v1.items():
                    if v2 == value:
                        return k,v
    
    def get_xbee_address(self,dict,value):
        pair_device = dict[value]["xbee"]["xbee_pair"]
        address = dict[pair_device]["rpi_2"]["xbee_rx"]
        return address

    def heartbeat(self,event):
        self.data_status = "6@"+str(self.robot_status.data[2])+":"+str(self.robot_status.data[3])+":"+\
            str((self.robot_status.data[4]+self.robot_status.data[5]+self.robot_status.data[6]+self.robot_status.data[7]))+":"+\
                str(self.robot_status.data[8])+":"+str(self.robot_status.data[1])
        
        print(self.data_status)
        pack = self.xbee_encode( self.data_odom, data_type = b'\x00')
        self.xbee_send(self.xbee_address, pack,data_type = b'\x00')
        print("send status to anchor")

    def cb_robot_status(self,msg):
        self.robot_status.data = msg.data
        print(self.robot_status)

    def cb_robot_odom(self,msg):
        self.data_odom = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,\
        msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, \
                            msg.pose.pose.orientation.w,msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z,\
                            msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]\
                            ,dtype=np.float16)

        pack = self.xbee_encode( self.data_odom, data_type = b'\x02')
        self.xbee_send(self.xbee_address, pack,data_type = b'\x02')


    def xbee_encode(self,data_via_xbee, data_type):
        data = data_via_xbee

        # send data
        byte_arr = pickle.dumps( data )
        length, index, check= int(len(byte_arr)), 0, 0

        for index in range(0,length,250) :
            pack = bytearray(b'\xAB') #Header
            pack.extend(bytearray(data_type)) #Type
            pack.extend( length.to_bytes(4, byteorder='big') ) #bytes
            index_end = index+250 if index+250 < length else length
            pack.extend( byte_arr[index:(index_end)] ) #data

            if index_end == length : pack.extend(check.to_bytes(1, byteorder='big')) # checksum
            else: check = 0xff & (check + pack[-1])

        return pack    

    def xbee_send(self, ADDRESS_L,pack,data_type):
        ADDRESS_H = '0013A200'
        ADDRESS = ADDRESS_H + ADDRESS_L
        if data_type == b'\x03' : self.device.send_data_broadcast(pack) # all robot have to know where others are
        else:
            try:
                print("send odom")
                self.device.send_data_64( XBee64BitAddress.from_hex_string(ADDRESS), pack)
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




if __name__ == "__main__":
	rospy.init_node("XBeeboatTx")
	XBeeboatTx = XBeeboatTx()
	rospy.spin()
