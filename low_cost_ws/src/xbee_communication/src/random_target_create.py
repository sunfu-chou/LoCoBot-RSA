#!/usr/bin/env python3
from re import X
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
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



class robot_status(object):
    def __init__(self):

        # with open('duckiepond-devices-machine.yaml', 'r') as f:
        #     data = yaml.load(f)

        # self.machine,self.compute_unit = self.who_am_I(data)
        self.machine = "wamv"
        self.boat = "boat"
        self.boat1 = "wamv"
        self.r = 8

        self.pub_status = rospy.Publisher( "/"+self.machine+"/target", targetpoint, queue_size=1)
        self.pub_points = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.pub_position_circle = rospy.Publisher("visualization_circle", Marker, queue_size=1)
        self.sub_wamv1 = rospy.Subscriber("/"+self.boat+"1/pose",Odometry,self.cb_wamv1,queue_size=1)
        self.pub_wamv1 = rospy.Publisher("/"+self.boat1+"1/move_base_simple/goal", PoseStamped, queue_size=1)
        self.wamv1_has_target = 0
        self.wamv1_target_point = 0
        self.sub_wamv2 = rospy.Subscriber("/"+self.boat+"2/pose",Odometry,self.cb_wamv2,queue_size=1)
        self.pub_wamv2 = rospy.Publisher("/"+self.boat1+"2/move_base_simple/goal", PoseStamped, queue_size=1)
        self.wamv2_has_target = 0
        self.wamv2_target_point = 0
        self.sub_wamv3 = rospy.Subscriber("/"+self.boat+"3/pose",Odometry,self.cb_wamv3,queue_size=1)
        self.pub_wamv3 = rospy.Publisher("/"+self.boat1+"3/move_base_simple/goal", PoseStamped, queue_size=1)
        self.wamv3_has_target = 0
        self.wamv3_target_point = 0
        self.sub_wamv4 = rospy.Subscriber("/"+self.boat+"4/pose",Odometry,self.cb_wamv4,queue_size=1)
        self.pub_wamv4 = rospy.Publisher("/"+self.boat1+"4/move_base_simple/goal", PoseStamped, queue_size=1)
        self.wamv4_has_target = 0
        self.wamv4_target_point = 0
        self.Markers = MarkerArray()
        self.Markers.markers = []
        self.point = 20
        self.target = targetpoint()
        self.array = [[0.0 for a in range (3)] for b in range (self.point)]
        for i in range (self.point):
                self.random_create(i)
                self.target.data_x.append(self.array[i][0])
                self.target.data_y.append(self.array[i][1])
                self.target.data_flag.append(0)
                self.marker = Marker()
                self.marker.header.stamp = rospy.Time.now()
                self.marker.header.frame_id = 'odom'
                self.marker.type = self.marker.SPHERE
                self.marker.action = self.marker.ADD
                self.marker.pose.orientation.w = 1
                self.marker.pose.position.x = self.array[i][0] 
                self.marker.pose.position.y = self.array[i][1]
                self.marker.id = i
                i = i+1
                self.marker.scale.x = 1
                self.marker.scale.y = 1
                self.marker.scale.z = 1
                self.marker.color.a = 1.0
                self.marker.color.r = 0
                self.marker.color.g = 1
                self.marker.color.b = 0
                self.Markers.markers.append(self.marker)

        self.robot_status = Float32MultiArray(data = self.array)
        

        #self.robot_status.layout.dim = 2
        # self.robot_status.layout.dim[0].label = "x"
        # self.robot_status.layout.dim[0].size = 10
        # self.robot_status.layout.dim[1].label = "y"
        # self.robot_status.layout.dim[1].size = 10

        self.timer = rospy.Timer(rospy.Duration(1), self.cb_publish)

        a = self.find_closest_available_point(0,0,self.robot_status.data)
        print(a)

        #print(self.robot_status.data)


        #print(self.robot_status.data[1]) 
        #       
    def random_create(self,i):
        self.array[i][0] = float(random.randrange(35,225,1))
        self.array[i][1] = float(random.randrange(-185,-145,1))
        for j in range (20):
            if j != i :
                if math.sqrt(math.pow(self.array[i][0]- self.array[j][0],2)+math.pow(self.array[i][1]- self.array[j][1],2)) < 15 :
                    self.random_create(i)
            


    def cb_wamv1(self,msg):
        if self.wamv1_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv1_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv1_target_point][2] = 1.0
            self.target.data_flag[self.wamv1_target_point] = 1.0
            self.wamv1_has_target = 1

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status.data[self.wamv1_target_point][0]
        pose.pose.position.y = self.robot_status.data[self.wamv1_target_point][1]
        self.pub_wamv1.publish(pose) 
        dis = math.sqrt(math.pow(msg.pose.pose.position.x- pose.pose.position.x,2)+math.pow(msg.pose.pose.position.y- pose.pose.position.y,2))
        if dis< self.r:
            print("wamv1 reach")
            self.wamv1_has_target = 0
            self.robot_status.data[self.wamv1_target_point][2] = 2.0
            self.target.data_flag[self.wamv1_target_point] = 2.0

    def cb_wamv2(self,msg):
        if self.wamv2_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv2_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv2_target_point][2] = 1.0
            self.target.data_flag[self.wamv2_target_point] = 1.0
            self.wamv2_has_target = 1

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status.data[self.wamv2_target_point][0]
        pose.pose.position.y = self.robot_status.data[self.wamv2_target_point][1]
        self.pub_wamv2.publish(pose) 
        dis = math.sqrt(math.pow(msg.pose.pose.position.x- pose.pose.position.x,2)+math.pow(msg.pose.pose.position.y- pose.pose.position.y,2))
        if dis< self.r:
            print("wamv2 reach")
            self.wamv2_has_target = 0
            self.robot_status.data[self.wamv2_target_point][2] = 2.0
            self.target.data_flag[self.wamv2_target_point] = 2.0

    def cb_wamv3(self,msg):
        if self.wamv3_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv3_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv3_target_point][2] = 1.0
            self.target.data_flag[self.wamv3_target_point] = 1.0
            self.wamv3_has_target = 1

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status.data[self.wamv3_target_point][0]
        pose.pose.position.y = self.robot_status.data[self.wamv3_target_point][1]
        self.pub_wamv3.publish(pose) 
        dis = math.sqrt(math.pow(msg.pose.pose.position.x- pose.pose.position.x,2)+math.pow(msg.pose.pose.position.y- pose.pose.position.y,2))
        if dis< self.r:
            print("wamv3 reach")
            self.wamv3_has_target = 0
            self.robot_status.data[self.wamv3_target_point][2] = 2.0
            self.target.data_flag[self.wamv3_target_point] = 2.0


    def cb_wamv4(self,msg):
        if self.wamv4_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv4_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv4_target_point][2] = 1.0
            self.target.data_flag[self.wamv4_target_point] = 1.0
            self.wamv4_has_target = 1

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.robot_status.data[self.wamv4_target_point][0]
        pose.pose.position.y = self.robot_status.data[self.wamv4_target_point][1]
        self.pub_wamv4.publish(pose) 
        dis = math.sqrt(math.pow(msg.pose.pose.position.x- pose.pose.position.x,2)+math.pow(msg.pose.pose.position.y- pose.pose.position.y,2))
        if dis< self.r:
            print("wamv4 reach")
            self.wamv4_has_target = 0
            self.robot_status.data[self.wamv4_target_point][2] = 2.0
            self.target.data_flag[self.wamv4_target_point] = 2.0


    def find_closest_available_point(self,x,y,array):
        closest_distance = 1000
        target_point = 0
        count  = 0
        for i in range (self.point):
            if array[i][2] == 0:
                distance = math.sqrt(math.pow(x- array[i][0],2)+math.pow(y- array[i][1],2))
                if distance < closest_distance:
                    closest_distance = distance
                    target_point = i
            else:
                count = count+1
        if count == self.point:
            target_point = self.point+1        
        
        return target_point
        

    def cb_publish(self, event):
        print(self.robot_status.data)
        print(self.target)
        self.pub_status.publish(self.target)
        self.pub_points.publish(self.Markers)
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'odom'
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        circumference = []
        p = Point()
        p.x = 30
        p.y = -190
        p.z = 0.1
        p1 = Point()
        p1.x = 230
        p1.y = -140
        p1.z = 0.1
        p2 = Point()
        p2.x = 30
        p2.y = -140
        p2.z = 0.1
        p3 = Point()
        p3.x = 230
        p3.y = -190
        p3.z = 0.1
        circumference.append(p)
        circumference.append(p3)
        circumference.append(p1)
        circumference.append(p2)
        circumference.append(p)

        marker.pose.orientation.w = 1
        marker.points = circumference
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        self.pub_position_circle.publish(marker)






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
