#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32, Int64
from sensor_msgs.msg import  Joy
from xbee_communication.srv import xbee
import struct
import roslib
import pickle
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from datetime import datetime
import subprocess
import yaml
from nav_msgs.msg import Odometry


class robot_status(object):
    def __init__(self):

        # with open('duckiepond-devices-machine.yaml', 'r') as f:
        #     data = yaml.load(f)

        # self.machine,self.compute_unit = self.who_am_I(data)
        self.machine = "boat5"

        self.pub_status = rospy.Publisher( "/"+self.machine+"/robotstatus", Float32MultiArray, queue_size=1)
        self.sub_imu = rospy.Subscriber("gx5/imu/data", Imu,self.imu_status,queue_size=1)
        self.sub_temp = rospy.Subscriber("/temp", Float32 ,self.cb_temp,queue_size=1)
        self.sub_power = rospy.Subscriber("/curre_8A_node/power", Int64 ,self.cb_power,queue_size=1)
        self.sub_mm_1 = rospy.Subscriber("/"+self.machine"/mm_1", Float32 ,self.cb_mm_1,queue_size=1)
        self.sub_mm_2 = rospy.Subscriber("/"+self.machine"/mm_2", Float32 ,self.cb_mm_2,queue_size=1)
        self.sub_mm_3 = rospy.Subscriber("/"+self.machine"/mm_3", Float32 ,self.cb_mm_3,queue_size=1)
        self.sub_mm_4 = rospy.Subscriber("/"+self.machine"/mm_4", Float32 ,self.cb_mm_4,queue_size=1)
        self.sub_zed = rospy.Subscriber("/"+self.machine"/zed", Float32 ,self.cb_zed,queue_size=1)
        self.sub_vr_joy = rospy.Subscriber("/boat/vr_joy_tvl", Float32MultiArray, self.vr_joy_status,queue_size = 1)
        self.pub_vr_joy = rospy.Publisher("/boat/vr_joy",Float32MultiArray,queue_size = 1)
        self.robot_status = Float32MultiArray()
        self.count = Float32MultiArray()
        self.power_used = 0
        self.total_power = 39000
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_publish)
        self.timer_1 = rospy.Timer(rospy.Duration(1), self.clean)
        for i in range (10):
            self.robot_status.data.extend([0])
            self.count.data.extend([2])

        #print(self.robot_status.data[1])       


    def imu_status(self,msg):
        self.robot_status.data[0] = 1
        self.count.data[0] = 2
    

    def vr_joy_status(self,msg):
        self.robot_status.data[1] = 1
        self.count.data[1] = 5
        self.pub_vr_joy.publish(msg)


    def cb_temp(self,msg):
        self.robot_status.data[2] = int(msg.data)
        self.count.data[2] = 5

    def cb_power(self,msg):
        self.power_used = (self.power_used + msg.data)/1000
        power_use_percentage = 1-(self.power_used/self.total_power)
        self.robot_status.data[3] = power_use_percentage
        self.count.data[3] = 5

    def cb_mm_1(self,msg):
        self.robot_status.data[4] = 1
        self.count.data[4] = 2

    def cb_mm_2(self,msg):
        self.robot_status.data[5] = 1
        self.count.data[5] = 2
    
    def cb_mm_3(self,msg):
        self.robot_status.data[6] = 1
        self.count.data[6] = 2

    def cb_mm_4(self,msg):
        self.robot_status.data[7] = 1
        self.count.data[7] = 2

    def cb_zed(self,msg):
        self.robot_status.data[8] = 1
        self.count.data[8] = 2


    def cb_publish(self, event):
        print(self.count.data)
        print(self.robot_status.data)
        self.pub_status.publish(self.robot_status)

    def clean(self,event):
        for i in range (10):
            self.count.data[i] = max((self.count.data[i]-1),0)
            if self.count.data[i] == 0:
                self.robot_status.data[i] = 0




    def who_am_I(self,data):
            ret_byte = subprocess.check_output(['ifconfig'])
            ret_str = ret_byte.decode('utf-8')
            # Cut string from 'equal symbol' to 'degree C symbol', then convert to float
            en = ret_str[ret_str.find('eno1:'): ret_str.find('192.168.1.255')]
            ip = en[en.find('inet')+5: en.find('netmask')-2]
            machine,device = self.get_key(data,ip)
            return machine,device

    def get_key(self,dict,value):
        for k, v in dict.items():
            for v,v1 in v.items():
                for v1,v2 in v1.items():
                    if v2 == value:
                        return k,v






if __name__ == "__main__":
	rospy.init_node("robot_status")
	robot_status = robot_status()
	rospy.spin()
