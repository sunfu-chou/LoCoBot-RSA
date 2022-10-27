#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import  Joy
from xbee_communication.srv import xbee
from geometry_msgs.msg import Twist
import struct
import roslib
import pickle
import time
from xbee_communication.msg import targetpoint
from std_msgs.msg import Float32MultiArray
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import *
from digi.xbee.models.address import *
from digi.xbee.packets.base import DictKeys
from digi.xbee.packets.common import ATCommPacket, ATCommResponsePacket
from digi.xbee.models.mode import OperatingMode
from datetime import datetime
import subprocess
import yaml

class XBeeAnchorTx(object):
    def __init__(self):
        self.PORT = rospy.get_param("~port")
        self.BAUD_RATE = 115200
        self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)
        self.device.open(force_settings=True)
        self.sourceAddr = str(self.device.get_64bit_addr())

   #     with open('/home/argduckiepond/duckiepond-nctu/duckiepond-devices/duckiepond-devices-machine.yaml', 'r') as f:
    #            data = yaml.load(f)

     #   self.machine,self.compute_unit = self.who_am_I(data)
      #self.xbee_address = self.get_xbee_address(data,self.machine)

        self.machine = "boat1"
        self.machine1 = "wamv1"
        self.xbee_address = "41AF1C55"
        
        self.sub_joy = rospy.Subscriber("/"+ self.machine +"/joy", Joy, self.cb_joy, queue_size=1)
        #self.sub_vr_joy = rospy.Subscriber("/boat/vr_joy_xbee", Float32MultiArray , self.cb_vr_joy, queue_size = 1)
        self.sub_robot_status = rospy.Subscriber("/dbt22/robotstatus", Float32MultiArray, self.cb_robot_status, queue_size = 1)
        self.sub_Robot_cmd_1 = rospy.Subscriber("/"+self.machine1+"/cmd_vel", Twist,self.cb_Robot_cmd,queue_size=1)
        self.sub_target = rospy.Subscriber("/wamv/target", targetpoint,self.cb_target,queue_size=1)
        print("xbee node initialized (Master), I am ", self.sourceAddr[8:], 'remote ',self.xbee_address)


    def who_am_I(self,data):

            ret_byte = subprocess.check_output(['ifconfig'])
            ret_str = ret_byte.decode('utf-8')
            # Cut string from 'equal symbol' to 'degree C symbol', then convert to float
            en = ret_str[ret_str.find('wlan1:'): ret_str.find('192.168.1.255')]
            ip = en[en.find('inet')+5: en.find('netmask')-2]
            print(en)
            machine,device = self.get_key(data,ip)
            print("I am"+ machine)
            return machine,device

    def get_key(self,dict,value):
        for k, v in dict.items():
            for v,v1 in v.items():
                for v1,v2 in v1.items():
                    if v2 == value:
                        return k,v
    
    def get_xbee_address(self,dict,value):
        for k, v in dict.items():
            for v,v1 in v.items():
                for v1,v2 in v1.items():
                    if v2 == value:
                        address = dict[k]["rpi"]['xbee_rx']
                        return address
    
    # def cb_vr_joy(self,msg):
    #     self.data_vr_joy = np.array([msg.data[0],msg.data[1],msg.data[2],msg.data[3]],dtype = np.float32)
    #     print("send vr joy")
    #     print(self.data_vr_joy)
    #     self.xbee_encode_and_send(self.xbee_address, self.data_vr_joy,data_type = b'\x00')
    #     #print("send vr joy")


    def cb_robot_status(self,msg):
        self.robot_status = msg

    def cb_target(self, msg):
        self.data_target  = []
        for i in range (20):
            self.data_target.append(msg.data_flag[i])
        self.xbee_encode_and_send(self.xbee_address, self.data_target, data_type=b'\x02')
        print(self.data_target)



    def cb_Robot_cmd(self, msg):

        self.data_twist = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],dtype=np.float16)
        self.xbee_encode_and_send(self.xbee_address, self.data_twist, data_type=b'\x01')
        print("send twist")


    def cb_joy(self,msg):
        ADDRESS_H = '0013A200'
        ADDRESS = ADDRESS_H + self.xbee_address
        print(len(ADDRESS))
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
        except TransmitException:
            rospy.logerr('digi.xbee.exception.TransmitException: There was a problem with a transmitted packet response')
        except TimeoutException:
            rospy.logerr('digi.xbee.exception.TimeoutException: Response not received in the configured timeout.')
        except :
            rospy.logerr('send data_via_xbee fail')



    def xbee_encode_and_send(self, ADDRESS_L, data_via_xbee, data_type):
        data = data_via_xbee
        ADDRESS_H = '0013A200'
        ADDRESS = ADDRESS_H + ADDRESS_L

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
            if data_type == b'\x03' : self.device.send_data_broadcast(pack) # all robot have to know where others are
            else:
                try:
                    #print(pack)
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

        return True



if __name__ == "__main__":
    rospy.init_node("XBeeanchortx")
    XBeeanchortx = XBeeAnchorTx()
    rospy.spin()
