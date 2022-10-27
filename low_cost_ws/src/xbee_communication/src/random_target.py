#!/usr/bin/env python3
from re import X
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32, Int64,String
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
        self.r = 10

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

        self.sub_exp_state = rospy.Subscriber("/boat1/status",String,self.cb_status,queue_size=1)
        self.wamv4_has_target = 0
        self.wamv4_target_point = 0
        self.Markers = MarkerArray()
        self.Markers.markers = []
        self.point = 20
        self.count_flag = 0
        self.array_num = 0
        self.target = targetpoint()
        array = [[0.0 for a in range (3)] for b in range (self.point)]
        for i in range(self.point):
            self.target.data_x.append(0)
            self.target.data_y.append(0)
            self.target.data_flag.append(0)            
        self.array1 = [[191.0, -173.0, 0.0], [161.0, -152.0, 0.0], [134.0, -185.0, 0.0], [122.0, -176.0, 0.0], [131.0, -185.0, 0.0], [137.0, -164.0, 0.0], [206.0, -173.0, 0.0], [137.0, -176.0, 0.0], [182.0, -161.0, 0.0], [179.0, -173.0, 0.0], [59.0, -152.0, 0.0], [179.0, -146.0, 0.0], [191.0, -173.0, 0.0], [65.0, -176.0, 0.0], [215.0, -149.0, 0.0], [107.0, -185.0, 0.0], [188.0, -161.0, 0.0], [146.0, -158.0, 0.0], [140.0, -146.0, 0.0], [209.0, -149.0, 0.0]]
        self.array2 = [[98.0, -158.0, 0.0], [77.0, -149.0, 0.0], [62.0, -185.0, 0.0], [131.0, -158.0, 0.0], [215.0, -161.0, 0.0], [218.0, -161.0, 0.0], [59.0, -167.0, 0.0], [47.0, -176.0, 0.0], [101.0, -173.0, 0.0], [170.0, -170.0, 0.0], [182.0, -164.0, 0.0], [65.0, -185.0, 0.0], [59.0, -182.0, 0.0], [53.0, -167.0, 0.0], [137.0, -179.0, 0.0], [158.0, -185.0, 0.0], [197.0, -179.0, 0.0], [221.0, -161.0, 0.0], [128.0, -146.0, 0.0], [35.0, -167.0, 0.0]]
        self.array3 = [[38.0, -176.0, 0.0], [65.0, -161.0, 0.0], [95.0, -164.0, 0.0], [200.0, -164.0, 0.0], [110.0, -155.0, 0.0], [161.0, -158.0, 0.0], [140.0, -155.0, 0.0], [155.0, -161.0, 0.0], [74.0, -149.0, 0.0], [176.0, -167.0, 0.0], [89.0, -161.0, 0.0], [74.0, -173.0, 0.0], [149.0, -167.0, 0.0], [197.0, -182.0, 0.0], [161.0, -158.0, 0.0], [95.0, -176.0, 0.0], [56.0, -179.0, 0.0], [149.0, -167.0, 0.0], [179.0, -161.0, 0.0], [140.0, -161.0, 0.0]]
        self.array4 = [[86.0, -173.0, 0.0], [200.0, -158.0, 0.0], [107.0, -149.0, 0.0], [50.0, -167.0, 0.0], [110.0, -146.0, 0.0], [92.0, -158.0, 0.0], [197.0, -146.0, 0.0], [98.0, -155.0, 0.0], [83.0, -155.0, 0.0], [113.0, -173.0, 0.0], [173.0, -179.0, 0.0], [35.0, -149.0, 0.0], [167.0, -176.0, 0.0], [146.0, -158.0, 0.0], [221.0, -164.0, 0.0], [191.0, -182.0, 0.0], [101.0, -158.0, 0.0], [47.0, -152.0, 0.0], [179.0, -164.0, 0.0], [221.0, -146.0, 0.0]]
        self.array5 = [[41.0, -149.0, 0.0], [35.0, -167.0, 0.0], [56.0, -158.0, 0.0], [158.0, -164.0, 0.0], [209.0, -146.0, 0.0], [110.0, -179.0, 0.0], [137.0, -161.0, 0.0], [170.0, -170.0, 0.0], [194.0, -158.0, 0.0], [71.0, -176.0, 0.0], [74.0, -152.0, 0.0], [44.0, -161.0, 0.0], [35.0, -164.0, 0.0], [218.0, -152.0, 0.0], [164.0, -146.0, 0.0], [185.0, -161.0, 0.0], [212.0, -152.0, 0.0], [203.0, -176.0, 0.0], [206.0, -146.0, 0.0], [203.0, -164.0, 0.0]]
        self.array6 = [[143.0, -164.0, 0.0], [206.0, -149.0, 0.0], [155.0, -164.0, 0.0], [176.0, -164.0, 0.0], [146.0, -167.0, 0.0], [104.0, -179.0, 0.0], [182.0, -185.0, 0.0], [164.0, -158.0, 0.0], [89.0, -152.0, 0.0], [140.0, -158.0, 0.0], [173.0, -152.0, 0.0], [224.0, -152.0, 0.0], [155.0, -167.0, 0.0], [47.0, -146.0, 0.0], [122.0, -152.0, 0.0], [35.0, -164.0, 0.0], [185.0, -173.0, 0.0], [107.0, -152.0, 0.0], [203.0, -146.0, 0.0], [134.0, -164.0, 0.0]]
        self.array7 = [[188.0, -158.0, 0.0], [68.0, -167.0, 0.0], [170.0, -167.0, 0.0], [86.0, -161.0, 0.0], [224.0, -179.0, 0.0], [104.0, -167.0, 0.0], [83.0, -158.0, 0.0], [197.0, -185.0, 0.0], [143.0, -149.0, 0.0], [167.0, -164.0, 0.0], [212.0, -158.0, 0.0], [101.0, -155.0, 0.0], [185.0, -173.0, 0.0], [35.0, -158.0, 0.0], [125.0, -164.0, 0.0], [92.0, -176.0, 0.0], [92.0, -182.0, 0.0], [80.0, -179.0, 0.0], [41.0, -182.0, 0.0], [191.0, -161.0, 0.0]]
        self.array8 = [[218.0, -170.0, 0.0], [173.0, -155.0, 0.0], [212.0, -152.0, 0.0], [41.0, -182.0, 0.0], [92.0, -185.0, 0.0], [170.0, -164.0, 0.0], [215.0, -173.0, 0.0], [131.0, -155.0, 0.0], [95.0, -161.0, 0.0], [59.0, -155.0, 0.0], [56.0, -155.0, 0.0], [62.0, -167.0, 0.0], [74.0, -149.0, 0.0], [116.0, -149.0, 0.0], [83.0, -152.0, 0.0], [74.0, -164.0, 0.0], [53.0, -152.0, 0.0], [206.0, -158.0, 0.0], [65.0, -149.0, 0.0], [161.0, -182.0, 0.0]]
        self.array9 = [[47.0, -149.0, 0.0], [95.0, -164.0, 0.0], [47.0, -149.0, 0.0], [35.0, -146.0, 0.0], [173.0, -170.0, 0.0], [167.0, -167.0, 0.0], [125.0, -176.0, 0.0], [59.0, -155.0, 0.0], [146.0, -167.0, 0.0], [41.0, -182.0, 0.0], [194.0, -155.0, 0.0], [179.0, -161.0, 0.0], [107.0, -176.0, 0.0], [74.0, -173.0, 0.0], [161.0, -170.0, 0.0], [149.0, -167.0, 0.0], [122.0, -176.0, 0.0], [176.0, -164.0, 0.0], [113.0, -182.0, 0.0], [74.0, -182.0, 0.0]]
        self.array10 = [[218.0, -158.0, 0.0], [155.0, -182.0, 0.0], [107.0, -158.0, 0.0], [41.0, -167.0, 0.0], [203.0, -164.0, 0.0], [173.0, -176.0, 0.0], [170.0, -179.0, 0.0], [224.0, -185.0, 0.0], [197.0, -149.0, 0.0], [35.0, -146.0, 0.0], [188.0, -182.0, 0.0], [59.0, -152.0, 0.0], [38.0, -176.0, 0.0], [203.0, -179.0, 0.0], [71.0, -176.0, 0.0], [155.0, -185.0, 0.0], [62.0, -185.0, 0.0], [89.0, -173.0, 0.0], [188.0, -152.0, 0.0], [140.0, -149.0, 0.0]]
        self.fillinarray(self.array1)
        
        self.collision = 0
        self.finish = 0
        self.next_array = 0
        #self.robot_status.layout.dim = 2
        # self.robot_status.layout.dim[0].label = "x"
        # self.robot_status.layout.dim[0].size = 10
        # self.robot_status.layout.dim[1].label = "y"
        # self.robot_status.layout.dim[1].size = 10

        self.timer = rospy.Timer(rospy.Duration(5), self.cb_publish)

        #print(self.robot_status.data)


        #print(self.robot_status.data[1])       
    def cb_status(self,msg):
        if msg.data == "collision":
            self.collision = 1
        elif msg.data == "finish":
            #print("finish")
            self.finish = 1
        elif self.next_array == 1:
            self.collision = 0
            self.finish = 0
            self.next_array = 0

    def fillinarray(self,array):
        for i in range (self.point):
                self.target.data_x[i] = array[i][0]
                self.target.data_y[i] = array[i][1]
                self.target.data_flag[i] = array[i][2]
                self.marker = Marker()
                self.marker.header.stamp = rospy.Time.now()
                self.marker.header.frame_id = 'odom'
                self.marker.type = self.marker.SPHERE
                self.marker.action = self.marker.ADD
                self.marker.pose.orientation.w = 1
                self.marker.pose.position.x = array[i][0] 
                self.marker.pose.position.y = array[i][1]
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

        self.robot_status = Float32MultiArray(data = array)



    def change_color(self,x,y,i):
        self.marker = Marker()
        self.marker.header.stamp = rospy.Time.now()
        self.marker.header.frame_id = 'odom'
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.orientation.w = 1
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.id = i
        i = i+1
        self.marker.scale.x = 2
        self.marker.scale.y = 2
        self.marker.scale.z = 2
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0
        self.marker.color.b = 0
        self.Markers.markers.append(self.marker)

    def change_color_finish(self,x,y,i):
        self.marker = Marker()
        self.marker.header.stamp = rospy.Time.now()
        self.marker.header.frame_id = 'odom'
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.orientation.w = 1
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.id = i
        i = i+1
        self.marker.scale.x = 2
        self.marker.scale.y = 2
        self.marker.scale.z = 2
        self.marker.color.a = 1.0
        self.marker.color.r = 0
        self.marker.color.g = 0
        self.marker.color.b = 1
        self.Markers.markers.append(self.marker)


    def cb_wamv1(self,msg):
        if self.wamv1_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv1_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv1_target_point][2] = 1.0
            self.target.data_flag[self.wamv1_target_point] = 1.0
            self.wamv1_has_target = 1

        self.change_color(self.robot_status.data[self.wamv1_target_point][0],self.robot_status.data[self.wamv1_target_point][1],self.wamv1_target_point)
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
            self.change_color_finish(self.robot_status.data[self.wamv1_target_point][0],self.robot_status.data[self.wamv1_target_point][1],self.wamv1_target_point)

    def cb_wamv2(self,msg):
        if self.wamv2_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv2_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv2_target_point][2] = 1.0
            self.target.data_flag[self.wamv2_target_point] = 1.0
            self.wamv2_has_target = 1
        self.change_color(self.robot_status.data[self.wamv2_target_point][0],self.robot_status.data[self.wamv2_target_point][1],self.wamv2_target_point)
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
            self.change_color_finish(self.robot_status.data[self.wamv2_target_point][0],self.robot_status.data[self.wamv2_target_point][1],self.wamv2_target_point)

    def cb_wamv3(self,msg):
        if self.wamv3_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv3_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv3_target_point][2] = 1.0
            self.target.data_flag[self.wamv3_target_point] = 1.0
            self.wamv3_has_target = 1
        self.change_color(self.robot_status.data[self.wamv3_target_point][0],self.robot_status.data[self.wamv3_target_point][1],self.wamv3_target_point)
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
            self.change_color_finish(self.robot_status.data[self.wamv3_target_point][0],self.robot_status.data[self.wamv3_target_point][1],self.wamv3_target_point)


    def cb_wamv4(self,msg):
        if self.wamv4_has_target == 0:
            if (self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)) < self.point+1:
                self.wamv4_target_point = self.find_closest_available_point(msg.pose.pose.position.x,msg.pose.pose.position.y,self.robot_status.data)
            self.robot_status.data[self.wamv4_target_point][2] = 1.0
            self.target.data_flag[self.wamv4_target_point] = 1.0
            self.wamv4_has_target = 1
        self.change_color(self.robot_status.data[self.wamv4_target_point][0],self.robot_status.data[self.wamv4_target_point][1],self.wamv4_target_point)
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
            self.change_color_finish(self.robot_status.data[self.wamv4_target_point][0],self.robot_status.data[self.wamv4_target_point][1],self.wamv4_target_point)


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
        #print(self.robot_status.data)
        #print(self.finish)
        if self.finish == 1 or self.collision == 1:
            self.next_array = 1
            self.wamv1_has_target = 0
            self.wamv2_has_target = 0
            self.wamv3_has_target = 0
            self.wamv4_has_target = 0
            self.array_num = self.array_num+1
            if self.array_num == 1:
                self.fillinarray(self.array2)
            elif self.array_num == 2:
                self.fillinarray(self.array3)
            elif self.array_num == 3:
                self.fillinarray(self.array4)
            elif self.array_num == 4:
                self.fillinarray(self.array5)
            elif self.array_num == 5:
                self.fillinarray(self.array6)
            elif self.array_num == 6:
                self.fillinarray(self.array7)
            elif self.array_num == 7:
                self.fillinarray(self.array8)
            elif self.array_num == 8:
                self.fillinarray(self.array9)
            elif self.array_num == 9:
                self.fillinarray(self.array10)

        self.count_flag = 0
        print(self.array_num)
        print(self.robot_status.data)
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
