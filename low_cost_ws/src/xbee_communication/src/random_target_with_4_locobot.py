#!/usr/bin/env python3
from re import X
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float32, Int64
from sensor_msgs.msg import  Joy
from xbee_communication.msg import targetpoint
import struct
import roslib
import pickle
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from datetime import datetime
import subprocess
import yaml
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import random
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
import math


from arg_robotics_tools  import websocket_rosbridge as socket

import threading
import os

######global variable###########
#odom = {'header': {'seq': 19, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'child_frame_id': '', 'pose': {'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}, 'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, 'twist': {'twist': {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}, 'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}
pose1 = {'x':0, 'y':0}
pose2 = {'x':500, 'y':0}
pose3 = {'x':0, 'y':500}
pose4 = {'x':500, 'y':500}

thread = []
locobot_ip = ['192.168.50.90', '192.168.50.60', '192.168.50.40', '192.168.50.20']

robot_flag1 = False
robot_flag2 = False
robot_flag3 = True #False
robot_flag4 = True #False

class robot_status(object):
    def __init__(self):
        self.robotmove = [1,1,1,1]
        self.robot_flag = Int64MultiArray()
        self.robot_flag.data  = self.robotmove

        # with open('duckiepond-devices-machine.yaml', 'r') as f:
        #     data = yaml.load(f)

        # self.machine,self.compute_unit = self.who_am_I(data)
        self.machine = "wamv"
        self.boat = "boat"
        self.boat1 = "wamv"
        self.r = 0.5

        self.pub_status = rospy.Publisher( "/"+self.machine+"/target", targetpoint, queue_size=1)
        self.pub_points = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.pub_position_circle = rospy.Publisher("visualization_circle", Marker, queue_size=1)
        self.pub_robot_move = rospy.Publisher("/robotmoveflag", Int64MultiArray, queue_size=1)
        
        
        #self.sub_wamv1 = rospy.Subscriber("/"+self.boat+"1/pose",Odometry,self.cb_wamv1,queue_size=1)        
        thread.append(threading.Thread(target = self.pose1, args=(locobot_ip[0],), daemon=True))
        self.pub_wamv1 = rospy.Publisher("/"+self.boat1+"1/move_base_simple/goal", PoseStamped, queue_size=1)
        self.wamv1_has_target = 0
        self.wamv1_target_point = 0

        #self.sub_wamv2 = rospy.Subscriber("/"+self.boat+"2/pose",Odometry,self.cb_wamv2,queue_size=1)
        thread.append(threading.Thread(target = self.pose2, args=(locobot_ip[1],), daemon=True))
        self.pub_wamv2 = rospy.Publisher("/"+self.boat1+"2/move_base_simple/goal", PoseStamped, queue_size=1)
        self.wamv2_has_target = 0
        self.wamv2_target_point = 0

        # #self.sub_wamv3 = rospy.Subscriber("/"+self.boat+"3/pose",Odometry,self.cb_wamv3,queue_size=1)
        # thread.append(threading.Thread(target = self.pose3, args=(locobot_ip[2],), daemon=True))
        # self.pub_wamv3 = rospy.Publisher("/"+self.boat1+"3/move_base_simple/goal", PoseStamped, queue_size=1)
        # self.wamv3_has_target = 0
        # self.wamv3_target_point = 0

        # #self.sub_wamv4 = rospy.Subscriber("/"+self.boat+"4/pose",Odometry,self.cb_wamv4,queue_size=1)
        # thread.append(threading.Thread(target = self.pose4, args=(locobot_ip[3],), daemon=True))
        # self.pub_wamv4 = rospy.Publisher("/"+self.boat1+"4/move_base_simple/goal", PoseStamped, queue_size=1)
        # self.wamv4_has_target = 0
        # self.wamv4_target_point = 0

        self.Markers = MarkerArray()
        self.Markers.markers = []
        self.point = 4
        self.target = targetpoint()
        array = [[0.0 for a in range (3)] for b in range (self.point)]
        
        #array = [[1.5, 2.1, 0], [1.5, 1.2, 0], ] # [5.5, 2.0, 0], [5.5, 5.0, 0], [1.8, 5.0, 0]
        
        array = [[3.0, 2.1, 0], [3.0, 1.2, 0], [4.0, 2.1, 0], [4.0, 1.2, 0] ,[2.0 , 1.5 ,0]]

        for i in range(self.point):
            self.target.data_x.append(array[i][0])
            self.target.data_y.append(array[i][1])
            self.target.data_flag.append(0)
            # self.marker = Marker()
            # self.marker.header.stamp = rospy.Time.now()
            # self.marker.header.frame_id = 'odom'
            # self.marker.type = self.marker.SPHERE
            # self.marker.action = self.marker.ADD
            # self.marker.pose.orientation.w = 1
            # self.marker.pose.position.x = array[i][0] 
            # self.marker.pose.position.y = array[i][1]
            # self.marker.id = i
            # i = i+1
            # self.marker.scale.x = 1
            # self.marker.scale.y = 1
            # self.marker.scale.z = 1
            # self.marker.color.a = 1.0
            # self.marker.color.r = 0
            # self.marker.color.g = 1
            # self.marker.color.b = 0
            # self.Markers.markers.append(self.marker)
        self.robot_status_data = Float32MultiArray(data = array)

        
        #self.robot_status.layout.dim = 2
        # self.robot_status.layout.dim[0].label = "x"
        # self.robot_status.layout.dim[0].size = 10
        # self.robot_status.layout.dim[1].label = "y"
        # self.robot_status.layout.dim[1].size = 10

        self.timer = rospy.Timer(rospy.Duration(1), self.cb_publish)

        a = self.find_closest_available_point(0,0,self.robot_status_data.data)
        #print('-------find closet point-------')
        #print(a)

        thread.append(threading.Thread(target = self.start, daemon=True))
        for i in thread:
            i.start()
        

        #print(self.robot_status.data)


        #print(self.robot_status.data[1])       


############websocket part#############

    ####pose1####
    def pose1_callback(self, message):
        global pose1
        global robot_flag1
        pose1['x'] = message['pose']['pose']['position']['x']
        pose1['y'] = message['pose']['pose']['position']['y']
        robot_flag1 = True
        print('robot1')
        print(pose1)


    def pose1(self, locobot_ip):
        socket_sensor = socket.ros_socket(locobot_ip, 9090)
        socket_sensor.subscriber('/uwb_pose' , self.pose1_callback, 500)

    ####pose2####
    def pose2_callback(self, message):
        global pose2
        global robot_flag2
        pose2['x'] = message['pose']['pose']['position']['x']
        pose2['y'] = message['pose']['pose']['position']['y']
        robot_flag2 = True
        print('robot2')
        print(pose2)

    def pose2(self, locobot_ip):
        socket_sensor = socket.ros_socket(locobot_ip, 9090)
        socket_sensor.subscriber('/uwb_pose' , self.pose2_callback, 100)
    
    ####pose3####
    def pose3_callback(self, message):
        global pose3
        global robot_flag3
        pose3['x'] = message['pose']['pose']['position']['x']
        pose3['y'] = message['pose']['pose']['position']['y']
        robot_flag3 = True

    def pose3(self, locobot_ip):
        socket_sensor = socket.ros_socket(locobot_ip, 9090)
        socket_sensor.subscriber('/uwb_pose' , self.pose3_callback, 100)

    ####pose4####
    def pose4_callback(self, message):
        global pose4
        global robot_flag4
        pose4['x'] = message['pose']['pose']['position']['x']
        pose4['y'] = message['pose']['pose']['position']['y']
        robot_flag4 = True

    def pose4(self, locobot_ip):
        socket_sensor = socket.ros_socket(locobot_ip, 9090)
        socket_sensor.subscriber('/uwb_pose', self.pose4_callback, 100)

############ location calculation part################

    def cb_wamv1(self):
        global pose1
        if self.wamv1_has_target == 0:
            if (self.find_closest_available_point(float(pose1['x']), float(pose1['y']), self.robot_status_data.data)) < self.point+1:
                self.robotmove[0] = 1
                self.wamv1_target_point = self.find_closest_available_point(float(pose1['x']), float(pose1['y']),self.robot_status_data.data)
            else:
                self.robotmove[0] = 0
            self.robot_status_data.data[self.wamv1_target_point][2] = 1.0
            self.target.data_flag[self.wamv1_target_point] = 1
            self.wamv1_has_target = 1

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status_data.data[self.wamv1_target_point][0]
        pose.pose.position.y = self.robot_status_data.data[self.wamv1_target_point][1]
        self.pub_wamv1.publish(pose) 
        dis = math.sqrt(math.pow(float(pose1['x'])- pose.pose.position.x,2)+math.pow(float(pose1['y'])- pose.pose.position.y,2))
        # print('1 dist:')
        # print(dis)
        # print('1 goal:')
        # print(pose.pose.position)
        if dis < self.r:
            print("wamv1 reach")
            self.robotmove[0] = 0
            self.wamv1_has_target = 0
            self.robot_status_data.data[self.wamv1_target_point][2] = 2.0
            self.target.data_flag[self.wamv1_target_point] = 2

    def cb_wamv2(self):
        global pose2
        if self.wamv2_has_target == 0:
            if (self.find_closest_available_point(float(pose2['x']), float(pose2['y']),self.robot_status_data.data)) < self.point+1:
                self.wamv2_target_point = self.find_closest_available_point(float(pose2['x']), float(pose2['y']),self.robot_status_data.data)
                self.robotmove[1] = 1
            else:
                self.robotmove[1] = 0 #no new goal point stop robot flag

            self.robot_status_data.data[self.wamv2_target_point][2] = 1.0
            self.target.data_flag[self.wamv1_target_point] = 1
            self.wamv2_has_target = 1

        
            

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status_data.data[self.wamv2_target_point][0]
        pose.pose.position.y = self.robot_status_data.data[self.wamv2_target_point][1]
        self.pub_wamv2.publish(pose) 
        dis = math.sqrt(math.pow(float(pose2['x'])- pose.pose.position.x,2)+math.pow(float(pose2['y'])- pose.pose.position.y,2))
        # print('2 dist:')
        # print(dis)
        # print('2 goal:')
        # print(pose.pose.position)
        if dis < self.r:
            print("wamv2 reach")
            self.robotmove[1] = 0
            self.wamv2_has_target = 0
            self.robot_status_data.data[self.wamv2_target_point][2] = 2.0
            self.target.data_flag[self.wamv1_target_point] = 2

    def cb_wamv3(self):
        if self.wamv3_has_target == 0:
            if (self.find_closest_available_point(float(pose3['x']),float(pose3['y']),self.robot_status_data.data)) < self.point+1:
                self.wamv3_target_point = self.find_closest_available_point(float(pose3['x']),float(pose3['y']),self.robot_status_data.data)
                self.robotmove[2] = 1
            else:
                self.robotmove[2] = 0
            
            self.robot_status_data.data[self.wamv3_target_point][2] = 1.0
            self.target.data_flag[self.wamv1_target_point] = 1
            self.wamv3_has_target = 1

        
    
      
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status_data.data[self.wamv3_target_point][0]
        pose.pose.position.y = self.robot_status_data.data[self.wamv3_target_point][1]
        self.pub_wamv3.publish(pose) 
        dis = math.sqrt(math.pow(float(pose3['x'])- pose.pose.position.x,2)+math.pow(float(pose3['y'])- pose.pose.position.y,2))
        if dis< self.r:
            print("wamv3 reach")
            self.robotmove[2] = 0
            self.wamv3_has_target = 0
            self.robot_status_data.data[self.wamv3_target_point][2] = 2.0
            self.target.data_flag[self.wamv1_target_point] = 2


    def cb_wamv4(self):
        if self.wamv4_has_target == 0:
            if (self.find_closest_available_point(float(pose4['x']),float(pose4['y']),self.robot_status_data.data)) < self.point+1:
                self.wamv4_target_point = self.find_closest_available_point(float(pose4['x']),float(pose4['y']),self.robot_status_data.data)
                self.robotmove[3] = 1
            else:
                self.robotmove[3] = 0
            
            self.robot_status_data.data[self.wamv4_target_point][2] = 1.0
            self.target.data_flag[self.wamv1_target_point] = 1
            self.wamv4_has_target = 1
        

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status_data.data[self.wamv4_target_point][0]
        pose.pose.position.y = self.robot_status_data.data[self.wamv4_target_point][1]
        self.pub_wamv4.publish(pose) 
        dis = math.sqrt(math.pow(float(pose4['x'])- pose.pose.position.x,2)+math.pow(float(pose4['x'])- pose.pose.position.y,2))
        if dis< self.r:
            print("wamv4 reach")
            self.robotmove[3] = 0
            self.wamv4_has_target = 0
            self.robot_status_data.data[self.wamv4_target_point][2] = 2.0
            self.target.data_flag[self.wamv1_target_point] = 2

    def start(self):
        global robot_flag1, robot_flag2, robot_flag3, robot_flag4
        while True:
            if robot_flag1 and robot_flag2 and robot_flag3 and robot_flag4:
                self.cb_wamv1()
                self.cb_wamv2()
                # self.cb_wamv3()
                # self.cb_wamv4()
                time.sleep(0.2)
                self.robot_flag.data = self.robotmove
                self.pub_robot_move.publish(self.robot_flag)
            else:
                print('wait for odom')
                time.sleep(1)

    def find_closest_available_point(self,x,y,array):
        closest_distance = 1000
        target_point = 0
        count  = 0
        for i in range (self.point):
            if array[i][2] == 0:
                distance = math.sqrt(math.pow(x- array[i][0],2)+math.pow(y- array[i][1],2))
                #print([x,y])
                #print(i)
                #print(distance)
                if distance < closest_distance:
                    closest_distance = distance
                    target_point = i
            else:
                count = count+1
        if count == self.point:
            target_point = self.point+1        
        
        return target_point
        

    def cb_publish(self, event):
        print('---------------publish----------------')
        #print(self.robot_status.data)
        self.pub_status.publish(self.target)
        #self.pub_points.publish(self.Markers)
        # marker = Marker()
        # marker.header.stamp = rospy.Time.now()
        # marker.header.frame_id = 'odom'
        # marker.type = marker.LINE_STRIP
        # marker.action = marker.ADD
        # circumference = []
        # p = Point()
        # p.x = 30
        # p.y = -190
        # p.z = 0.1
        # p1 = Point()
        # p1.x = 230
        # p1.y = -140
        # p1.z = 0.1
        # p2 = Point()
        # p2.x = 30
        # p2.y = -140
        # p2.z = 0.1
        # p3 = Point()
        # p3.x = 230
        # p3.y = -190
        # p3.z = 0.1
        # circumference.append(p)
        # circumference.append(p3)
        # circumference.append(p1)
        # circumference.append(p2)
        # circumference.append(p)

        # marker.pose.orientation.w = 1
        # marker.points = circumference
        # marker.scale.x = 0.05
        # marker.color.a = 1.0
        # marker.color.r = 0
        # marker.color.g = 1
        # marker.color.b = 0
        #self.pub_position_circle.publish(marker)

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