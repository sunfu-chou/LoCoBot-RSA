#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import  Joy
from xbee_communication.srv import xbee
import struct
import roslib
import pickle
from std_msgs.msg import Float32MultiArray
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
from xbee_communication.msg import targetpoint


class XBeeboatrx(object):
    def __init__(self):
        self.PORT = rospy.get_param("~port")
        self.BAUD_RATE = 115200
        self.get_register = bytearray()
        self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
        self.device.open(force_settings=True)
        self.sourceAddr = str(self.device.get_64bit_addr())

        #with open('/home/argduckiepond/duckiepond-nctu/duckiepond-devices/duckiepond-devices-machine.yaml', 'r') as f:
         #   data = yaml.load(f)

       #self.machine,self.compute_unit = self.who_am_I(data)
        self.machine = "boat1"

        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.pub_vr_joy = rospy.Publisher("/boat/vr_joy",Float32MultiArray,queue_size=1)
<<<<<<< Updated upstream
        self.pub_target = rospy.Publisher( "/wamv1/target", targetpoint, queue_size=1)
        self.sub_robot_status = rospy.Subscriber("/dbt22/robotstatus",Float32MultiArray, self.cb_robot_status,queue_size=1)
=======
        self.pub_available_robot = rospy.Publisher("/"+self.machine+"/available_robot",Float32MultiArray,queue_size=1)
        #self.sub_robot_status = rospy.Subscriber("/dbt22/robotstatus",Float32MultiArray, self.cb_robot_status,queue_size=1)
>>>>>>> Stashed changes
        self.pub_twist = rospy.Publisher( "/"+self.machine+"/robot_twist", Twist, queue_size=1)
        self.device.add_data_received_callback(self.xbee_callback_and_decode)
        self.count = 0
        self.robot_status = Float32MultiArray()
        for i in range (10):
            self.robot_status.data.extend([0])

        self.available_robot = Float32MultiArray()
        for i in range (5):
            self.available_robot.data.extend([0])
        print("xbee node initialized (Slave), I am ", self.sourceAddr[8:])

    def who_am_I(self,data):
            ret_byte = subprocess.check_output(['ifconfig'])
            ret_str = ret_byte.decode('utf-8')
            # Cut string from 'equal symbol' to 'degree C symbol', then convert to float
            en = ret_str[ret_str.find('wlan0:'): ret_str.find('Mask')]
            ip = en[en.find('inet')+5: en.find('netmask')-2]
            print(len(ip))
            machine,device = self.get_key(data,ip)
            print("I am "+machine)
            return machine,device

    def get_key(self,dict,value):
        for k, v in dict.items():
            for v,v1 in v.items():
                for v1,v2 in v1.items():
                    #print(v2)
                    #print(value)
                    if v2 == value:
                        return k,v
	
    def cb_robot_status(self,msg):
        self.robot_status.data = msg.data
        #print(self.robot_status.data[1])
	
    def xbee_callback_and_decode(self, xbee_message):
        #print("get xbee msg")
        ADDRESS = str(xbee_message.remote_device.get_64bit_addr()) # msg from who
        #if not self.last_remote == ADDRESS : self.clear()
        #self.last_remote = ADDRESS

        #if not xbee_message.data[0:1] == b'\xAB' : # Header wrong
         #   self.clear()
          #  print('get xbee_message with wrong Header')
           # return

        #print(xbee_message.data[6:])


        if not ((xbee_message.data[1:2] == b'\x00') or (xbee_message.data[1:2] == b'\x01') or (xbee_message.data[1:2] == b'\x02')):
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

       # print("aaaa")
        self.get_register = bytearray()
        self.get_register.extend(xbee_message.data[6:])
       # print("bbbb")
        #self.data_bytes = int.from_bytes(xbee_message.data[2:6], byteorder="big",signed=False)

        #print(self.get_register)
        #get_msg = pickle.loads(xbee_message.data)
        
        if xbee_message.data[1:2] == b'\x00':
            #print(self.get_register[:-1])
            get_joy = pickle.loads(self.get_register[:-1])
            #print(get_joy[0])
            pub_msg = self.np_array_to_vrjoy(get_joy)
            print(get_joy)
            print(self.robot_status.data[1]) 
            if self.robot_status.data[1] == 0:
                print("TVL fail, XBEE working")
                self.pub_vr_joy.publish(pub_msg)
            else:
                print("TVL working")


        if xbee_message.data[1:2] == b'\x01': # type: points
            #print("publish")
            get_points = pickle.loads(self.get_register[:-1])
            #self.clear()
            #print(get_points)
            pub_msg = self.np_array_to_twist(get_points)
            print(pub_msg)
            self.pub_twist.publish(pub_msg)
            #print("publish")
            #self.get_array_checksum = True
            #self.PointsEnd = self.PointsEnd + len(get_points)

        if xbee_message.data[1:2] == b'\x02': # type: points
            #print("publish")
            get_target = pickle.loads(self.get_register[:-1])
            #self.clear()
            #print(get_points)
            pub_msg = self.np_array_to_target(get_target)
            print(pub_msg)
            self.pub_target.publish(pub_msg)
            #print("publish")
            #self.get_array_checksum = True
            #self.PointsEnd = self.PointsEnd + len(get_points)

    def np_array_to_target(self,array):
        array_map = [[191.0, -173.0, 0.0], [161.0, -152.0, 0.0], [134.0, -185.0, 0.0], [122.0, -176.0, 0.0], [131.0, -185.0, 0.0], [137.0, -164.0, 0.0], [206.0, -173.0, 0.0], [137.0, -176.0, 0.0], [182.0, -161.0, 0.0], [179.0, -173.0, 0.0], [59.0, -152.0, 0.0], [179.0, -146.0, 0.0], [191.0, -173.0, 0.0], [65.0, -176.0, 0.0], [215.0, -149.0, 0.0], [107.0, -185.0, 0.0], [188.0, -161.0, 0.0], [146.0, -158.0, 0.0], [140.0, -146.0, 0.0], [209.0, -149.0, 0.0]]
        msg = targetpoint()
        for i in range (20):
            msg.data_x.append(array_map[i][0])
            msg.data_y.append(array_map[i][1])
            msg.data_flag.append(array[i])

        return msg

    def np_array_to_vrjoy(self,array):
        #print("aaaaaaaaa")
        msg = Float32MultiArray()
        msg.data.extend([array[0]])
        msg.data.extend([array[1]])
        msg.data.extend([array[2]])
        msg.data.extend([array[3]])
        #print(msg)
        return msg
    

    def np_array_to_twist(self,array):
        msg = Twist()
        msg.linear.x = array[0]
        msg.linear.y = array[1]
        msg.linear.z = array[2]
        msg.angular.x = array[3]
        msg.angular.y = array[4]
        msg.angular.z = array[5]
        return msg



if __name__ == "__main__":
	rospy.init_node("XBeeboatrx")
	XBeeboatrx = XBeeboatrx()
	rospy.spin()
