#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32,String
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
from geometry_msgs.msg import Twist
from datetime import datetime
import subprocess
import yaml
from nav_msgs.msg import Odometry


class XBeeanchorrx(object):
    def __init__(self):
        self.PORT = rospy.get_param("~port")
        self.BAUD_RATE = 115200
        self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
        self.get_register = bytearray()
        self.device.open(force_settings=True)
        self.sourceAddr = str(self.device.get_64bit_addr())

        #with open('/home/argduckiepond/duckiepond-nctu/duckiepond-devices/duckiepond-devices-machine.yaml', 'r') as f:
        #    data = yaml.load(f)

        #self.machine,self.compute_unit = self.who_am_I(data)
        self.machine = "anchor6"
        self.pub_robot_status = rospy.Publisher( "/"+self.machine+"/status", String, queue_size=1)
        self.pub_odom = rospy.Publisher( "/"+self.machine+"/pose", Odometry, queue_size=1)
        self.device.add_data_received_callback(self.xbee_callback_and_decode)
        self.count = 0
        print("xbee node initialized (Slave), I am ", self.sourceAddr[8:])

    def who_am_I(self,data):
            ret_byte = subprocess.check_output(['ifconfig'])
            ret_str = ret_byte.decode('utf-8')
            # Cut string from 'equal symbol' to 'degree C symbol', then convert to float
            en = ret_str[ret_str.find('eno1:'): ret_str.find('192.168.1.255')]
            ip = en[en.find('inet')+5: en.find('netmask')]
            machine,device = self.get_key(data,ip)
            return machine,device

    def get_key(self,dict,value):
        for k, v in dict.items():
            for v,v1 in v.items():
                for v1,v2 in v1.items():
                    if v2 == value:
                        return k,v


    def xbee_decode(self, xbee_message):
        ADDRESS = str(xbee_message.remote_device.get_64bit_addr()) # msg from who

        #print("get_msg")
        #print(xbee_message.data[1:2])

        if not xbee_message.data[0:1] == b'\xAB' : # Header wrong
            print('get xbee_message with wrong Header')
            return

        if not ((xbee_message.data[1:2] == b'\x00') or (xbee_message.data[1:2] == b'\x01') or (xbee_message.data[1:2] == b'\x02') or (xbee_message.data[1:2] == b'\x03')):
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
            return msg


        self.get_register.extend(xbee_message.data[6:])
        #self.data_bytes = int.from_bytes(xbee_message.data[2:6], byteorder="big",signed=False)
  
        if xbee_message.data[1:2] == b'\x00':
            get_msg = pickle.loads(self.get_register[:-1])
            #print(get_msg)
            return(get_msg)
    
        
        if xbee_message.data[1:2] == b'\x02': # type: points
            get_points = pickle.loads(self.get_register[:-1])
            pub_msg = self.np_array_to_Odometry(get_points)
            return(pub_msg)


    def xbee_callback_and_decode(self, xbee_message):

        #print(xbee_message.data[1:2])
        if not ((xbee_message.data[1:2] == b'\x00') or (xbee_message.data[1:2] == b'\x01') or (xbee_message.data[1:2] == b'\x02') or (xbee_message.data[1:2] == b'\x03')):
            msg = self.xbee_decode(xbee_message)
            self.pub_joy.publish(msg)


        if xbee_message.data[1:2] == b'\x00':
            get_msg = self.xbee_decode(xbee_message)
            self.pub_robot_status.publish(get_msg)
    
        
        if xbee_message.data[1:2] == b'\x02': # type: points
           # print("decode")
            pub_msg = self.xbee_decode(xbee_message)
            self.pub_odom.publish(pub_msg)

    def np_array_to_Odometry(self,array):
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = array[0]
        msg.pose.pose.position.y = array[1]
        msg.pose.pose.position.z = array[2]
        msg.pose.pose.orientation.x = array[3]
        msg.pose.pose.orientation.y = array[4]
        msg.pose.pose.orientation.z = array[5]
        msg.pose.pose.orientation.w = array[6]
        msg.twist.twist.linear.x = array[7]
        msg.twist.twist.linear.y = array[8]
        msg.twist.twist.linear.z = array[9]
        msg.twist.twist.angular.x = array[10]
        msg.twist.twist.angular.y = array[11]
        msg.twist.twist.angular.z = array[12]
        return msg


if __name__ == "__main__":
	rospy.init_node("xbeeanchorrx")
	xbeeanchorrx = XBeeanchorrx()
	rospy.spin()
