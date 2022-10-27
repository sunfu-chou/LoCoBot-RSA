#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
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


class restarttest(object):
    def __init__(self):

        # with open('duckiepond-devices-machine.yaml', 'r') as f:
        #     data = yaml.load(f)

        # self.machine,self.compute_unit = self.who_am_I(data)

        #self.pub_status = rospy.Publisher( "/"+"ipc"+"/robotstatus", Float32MultiArray, queue_size=1)
        print("self recovery service on")
        self.sub_imu = rospy.Subscriber("/"+"ipc"+"/robotstatus", Float32MultiArray ,self.imu_restart,queue_size=1)
        # self.robot_status = Float32MultiArray()
        # self.count = Float32MultiArray()
        # self.timer = rospy.Timer(rospy.Duration(1), self.cb_publish)
        # self.timer_1 = rospy.Timer(rospy.Duration(1), self.clean)
        # for i in range (10):
        #     self.robot_status.data.extend([0])
        #     self.count.data.extend([2])

        #print(self.robot_status.data[1])       
        #["roslaunch","microstrain_mips","microstrain.launch"]

    def imu_restart(self,msg):
        imu_state = msg.data[0]
        #print(imu_state)
        if imu_state == 0:
            print("run")
            ret = subprocess.run(["roslaunch microstrain_mips microstrain.launch"],shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8",timeout=100)
            print(ret)
    
    # def cb_publish(self, event):
    #     print(self.count.data)
    #     print(self.robot_status.data)
    #     self.pub_status.publish(self.robot_status)

    # def clean(self,event):
    #     for i in range (10):
    #         self.count.data[i] = max((self.count.data[i]-1),0)
    #         if self.count.data[i] == 0:
    #             self.robot_status.data[i] = 0




    # def who_am_I(self,data):
    #         ret_byte = subprocess.check_output(['ifconfig'])
    #         ret_str = ret_byte.decode('utf-8')
    #         # Cut string from 'equal symbol' to 'degree C symbol', then convert to float
    #         en = ret_str[ret_str.find('eno1:'): ret_str.find('192.168.1.255')]
    #         ip = en[en.find('inet')+5: en.find('netmask')-2]
    #         machine,device = self.get_key(data,ip)
    #         return machine,device

    # def get_key(self,dict,value):
    #     for k, v in dict.items():
    #         for v,v1 in v.items():
    #             for v1,v2 in v1.items():
    #                 if v2 == value:
    #                     return k,v






if __name__ == "__main__":
	rospy.init_node("restarttest")
	restarttestttt= restarttest()
	rospy.spin()