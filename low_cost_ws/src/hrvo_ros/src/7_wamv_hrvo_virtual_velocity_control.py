#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
#from RVO import RVO_update, reach, compute_V_des, reach
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import math
import tf
import numpy as np
import os
import pickle as pkl
from RVO import RVO_update, reach, compute_V_des, reach


class BoatHRVO(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)
        self.frame = "odom"
        self.frame1 = "odom"
        self.auto = 1

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_model = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)

        # setup publisher
        self.pub_v1 = rospy.Publisher("/wamv/cmd_vel", Twist, queue_size=1)
        self.sub_p3d1 = rospy.Subscriber(
            "/wamv/localization_gps_imu/odometry", Odometry, self.cb_boat1_odom, queue_size=1)
        

        self.pub_v2 = rospy.Publisher("/wamv1/cmd_vel", Twist, queue_size=1)
        self.sub_p3d2 = rospy.Subscriber(
            "/wamv1/localization_gps_imu/odometry", Odometry, self.cb_boat2_odom, queue_size=1)

        self.pub_v3 = rospy.Publisher("/wamv2/cmd_vel", Twist, queue_size=1)
        self.sub_p3d3 = rospy.Subscriber(
            "/wamv2/localization_gps_imu/odometry", Odometry, self.cb_boat3_odom, queue_size=1)
        
        self.pub_v4 = rospy.Publisher("/wamv3/cmd_vel", Twist, queue_size=1)
        self.sub_p3d4 = rospy.Subscriber(
            "/wamv3/localization_gps_imu/odometry", Odometry, self.cb_boat4_odom, queue_size=1)
        
        self.pub_v5 = rospy.Publisher("/wamv4/cmd_vel", Twist, queue_size=1)
        self.sub_p3d5 = rospy.Subscriber(
            "/wamv4/localization_gps_imu/odometry", Odometry, self.cb_boat5_odom, queue_size=1)
        
        self.pub_v6 = rospy.Publisher("/wamv5/cmd_vel", Twist, queue_size=1)
        self.sub_p3d6 = rospy.Subscriber(
            "/wamv5/localization_gps_imu/odometry", Odometry, self.cb_boat6_odom, queue_size=1)
        
        self.pub_v7 = rospy.Publisher("/wamv6/cmd_vel", Twist, queue_size=1)
        self.sub_p3d7 = rospy.Subscriber(
            "/wamv6/localization_gps_imu/odometry", Odometry, self.cb_boat7_odom, queue_size=1)

        self.sub_goal = rospy.Subscriber("/wamv/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_goal1 = rospy.Subscriber("/wamv1/move_base_simple/goal", PoseStamped, self.cb_goal1, queue_size=1)
        self.sub_goal2 = rospy.Subscriber("/wamv2/move_base_simple/goal", PoseStamped, self.cb_goal2, queue_size=1)
        self.sub_goal3 = rospy.Subscriber("/wamv3/move_base_simple/goal", PoseStamped, self.cb_goal3, queue_size=1)
        self.sub_goal4 = rospy.Subscriber("/wamv4/move_base_simple/goal", PoseStamped, self.cb_goal4, queue_size=1)
        self.sub_goal5 = rospy.Subscriber("/wamv5/move_base_simple/goal", PoseStamped, self.cb_goal5, queue_size=1)
        self.sub_goal6 = rospy.Subscriber("/wamv6/move_base_simple/goal", PoseStamped, self.cb_goal6, queue_size=1)


        self.sub_joy = rospy.Subscriber("/wamv/joy", Joy, self.cb_joy, queue_size=1)
        # initiallize boat status
        self.boat_odom = [Odometry() for i in range(7)]
        self.cmd_drive = [Twist() for i in range(7)]
        self.yaw = [0 for i in range(7)]
        self.vx = [0 for i in range(7)]
        self.vy = [0 for i in range(7)]
        # initiallize HRVO environment
        self.ws_model = dict()
        # robot radius
        self.ws_model['robot_radius'] = 3.5
        self.ws_model['circular_obstacles'] = []
        self.right_start = (6.5,-170)
        self.right_end = (9.3,-107)
        self.left_start = (5.7,-192)
        self.left_end = (5,-249)
        self.radius = 3
        self.right_point = abs(self.right_start[1]-self.right_end[1])/self.radius
        self.right_x = abs(self.right_start[0]-self.right_end[0])/self.right_point 
        for point in range(self.right_point+1):
            #print(point)
            self.ws_model['circular_obstacles'].append((self.right_start[0]+(self.right_x*point),self.right_start[1]+(self.radius*point),self.radius))

        self.left_point = abs(self.left_start[1]-self.left_end[1])/self.radius
        self.left_x = abs(self.left_start[0]-self.left_end[0])/self.left_point 
        for point in range(self.left_point+1):
            #print(point)
            self.ws_model['circular_obstacles'].append((self.left_start[0]-(self.left_x*point),self.left_start[1]-(self.radius*point),self.radius))

        #print(self.ws_model['circular_obstacles'])
        #print(self.right_start[1])#-170
        #hole = [x, y, rad]
        # rectangular boundary, format [x,y,width/2,heigth/2]


        self.ws_model['boundary'] = []


        self.position = []
        self.goal = [[0, 0] for i in range(7)]
        # print(self.position)
        # print(self.goal)
        self.velocity = [[0, 0] for i in range(7)]
        self.velocity_detect = [[0, 0] for i in range(7)]
        self.v_max = [0.5 for i in range(7)]
        self.vx_error_inte = [0 for i in range(7)]
        self.vy_error_inte = [0 for i in range(7)]
        self.v_error_inte = [0 for i in range(7)]
        self.vx_prior = [0 for i in range(7)]
        self.vy_prior = [0 for i in range(7)]
        self.v_prior = [0 for i in range(7)]
        self.vx_error_diff = [0 for i in range(7)]
        self.vy_error_diff = [0 for i in range(7)]
        self.v_error_diff = [0 for i in range(7)]
        self.angle_prior = [0 for i in range(7)]
        self.angle_diff = [0 for i in range(7)]

        # timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_hrvo)

    def cb_hrvo(self, event):
        if self.goal[0] == [0,0] and self.goal[1]==[0,0]:
            return
        if self.auto == 0:
            return
        self.update_all()
        v_des = compute_V_des(self.position, self.goal, self.v_max)
        self.velocity = RVO_update(
            self.position, v_des, self.velocity_detect, self.ws_model)

        for i in range(7):
            dis, angle = self.process_ang_dis(
                i,self.velocity[i][0], self.velocity[i][1], self.vx[i],self.vy[i],self.yaw[i])
            # dis, angle = self.process_ang_dis(
            #     1, 1, self.vx[i],self.vy[i],self.yaw[i])
            ##p3d 0.35 0.8
            cmd = Twist()
            cmd.linear.x = dis * 1.0
            cmd.angular.z = angle * 1.0
            self.cmd_drive[i] = cmd
            #print(cmd)

        #print(self.yaw)
        #print(v_des)
        #print(self.position)
        # print("vx")
        # print(self.velocity[0][0])
        # print("vy")
        #print(self.cmd_drive[6])
        #print(dis)
        #print(angle)
        #print(self.cmd_drive)
        #print(self.position)
        self.pub_v1.publish(self.cmd_drive[0])
        self.pub_v2.publish(self.cmd_drive[1])
        self.pub_v3.publish(self.cmd_drive[2])
        self.pub_v4.publish(self.cmd_drive[3])
        self.pub_v5.publish(self.cmd_drive[4])
        self.pub_v6.publish(self.cmd_drive[5])
        self.pub_v7.publish(self.cmd_drive[6])

    def check_state(self):
        min_dis = 1e9
        done = True
        for i in range(7):
            p = np.array(self.position[i])
            g = np.array(self.goal[i])
            d2g = np.linalg.norm(p-g)
            done = True if d2g < 2 else False
            for k in range(i+1, 7):
                p1 = np.array(self.position[i])
                p2 = np.array(self.position[k])
                dis = np.linalg.norm(p1-p2)
                min_dis = dis if dis < min_dis else min_dis
        return min_dis, done

    def process_ang_dis(self,i, vx, vy,vx_now,vy_now, yaw):
        #vx_error_angle = vx - vx_now
        #vy_error_angle = vy - vy_now
        v = math.sqrt(vx**2+vy**2)
        v_now = math.sqrt(vx_now**2+vy_now**2)
        vx_error = vx - vx_now
        vy_error = vx - vx_now
        v_error_int = v-v_now
        #v_error = math.sqrt(vx_error**2+vy_error **2)
        v_error = v-v_now
        
        self.vx_error_inte[i] = self.vx_error_inte[i] + vx_error
        self.vy_error_inte[i] = self.vy_error_inte[i] + vy_error
        self.v_error_inte[i] = self.v_error_inte[i] + v_error_int
        self.vx_error_diff[i] = vx_error - self.vx_prior[i]
        self.vy_error_diff[i] = vy_error - self.vy_prior[i]
        self.v_error_diff[i] = v_error - self.v_prior[i]
        self.vx_prior[i] = vx_error
        self.vy_prior[i] = vy_error
        self.v_prior[i] = v_error
        print(v)
        print(v_now)
        # print(vx)
        # print(vx_now)
        print(i)
        print(self.v_error_inte[i])


        
        dest_yaw = math.atan2(vy, vx)

        angle = dest_yaw - yaw

        self.angle_diff[i] = angle - self.angle_prior[i]
        self.angle_prior[i] = angle
        angle = 1.2 * angle + 1 * self.angle_diff[i]
        # print(dest_yaw)
        # print(yaw)
        print(angle)
        print("===========")

        #print(angle)
        if angle > np.pi:
            angle = angle-2*np.pi

        if angle < -np.pi:
            angle = angle+2*np.pi

        # angle = angle/np.pi
        # print(angle)

        #dis = math.sqrt((0.7*vx_error+0.01*self.vx_error_inte[i]-0.005*self.vx_error_diff[i])**2+(0.7*vy_error+0.01*self.vy_error_inte[i]-0.005*self.vy_error_diff[i])**2) 
        dis = 0.8*v_error +0.006*self.v_error_inte[i]+ 0.0002*self.v_error_diff[i]

        # if self.v_error_inte[i] < -15:
        #     #dis = 0.0
        #     # if abs(angle)>1:
        #     #     dis = -0.2
        #     #     print("keepppppp")
        #     self.v_error_inte[i] = 0
        #     print("reset")

        # print "pos      %2.2f, %2.2f" % (self.position[0][0], self.position[0][1])
        # print "goal     %2.2f, %2.2f" % (self.goal[0][0], self.goal[0][1])
        # print "dest_yaw %2.2f" % dest_yaw
        # print "yaw      %2.2f" % yaw
        # print "angle    %2.2f" % angle
        # print "dis      %2.2f\n" % dis

        dis = max(min(dis, 1), -1)
        angle = max(min(angle, 1), -1)

        return dis, angle

    def update_all(self):
        self.position = []
        for i in range(7):
            # update position
            pos = [self.boat_odom[i].pose.pose.position.x,
                   self.boat_odom[i].pose.pose.position.y]
            self.position.append(pos)

            # update orientation
            quaternion = (self.boat_odom[i].pose.pose.orientation.x,
                          self.boat_odom[i].pose.pose.orientation.y,
                          self.boat_odom[i].pose.pose.orientation.z,
                          self.boat_odom[i].pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw[i] = euler[2]
            self.vx[i] = self.boat_odom[i].twist.twist.linear.x
            self.vy[i] = self.boat_odom[i].twist.twist.linear.y

            # update velocity
            self.velocity_detect[i] = [self.boat_odom[i].twist.twist.linear.x,
                                       self.boat_odom[i].twist.twist.linear.y]
    def cb_goal(self, msg):
        if msg.header.frame_id != self.frame:
            self.goal = None
            return
        self.goal[0] = [msg.pose.position.x, msg.pose.position.y]

    def cb_goal1(self, msg):
        if msg.header.frame_id != self.frame1:
            self.goal = None
            return
        self.goal[1] = [msg.pose.position.x, msg.pose.position.y]

    def cb_goal2(self, msg):
        if msg.header.frame_id != self.frame1:
            self.goal = None
            return
        self.goal[2] = [msg.pose.position.x, msg.pose.position.y]

    def cb_goal3(self, msg):
        if msg.header.frame_id != self.frame1:
            self.goal = None
            return
        self.goal[3] = [msg.pose.position.x, msg.pose.position.y]

    def cb_goal4(self, msg):
        if msg.header.frame_id != self.frame1:
            self.goal = None
            return
        self.goal[4] = [msg.pose.position.x, msg.pose.position.y]

    def cb_goal5(self, msg):
        if msg.header.frame_id != self.frame1:
            self.goal = None
            return
        self.goal[5] = [msg.pose.position.x, msg.pose.position.y]

    def cb_goal6(self, msg):
        if msg.header.frame_id != self.frame1:
            self.goal = None
            return
        self.goal[6] = [msg.pose.position.x, msg.pose.position.y]

    
    def cb_joy(self, msg):
        start_button = 7
        back_button = 6

        if (msg.buttons[start_button] == 1) and not self.auto:
            self.auto = 1
            rospy.loginfo('go auto')
        elif msg.buttons[back_button] == 1 and self.auto:
            self.auto = 0
            rospy.loginfo('go manual')
    
    def cb_boat1_odom(self, msg):
        self.boat_odom[0] = msg

    def cb_boat2_odom(self, msg):
        self.boat_odom[1] = msg

    def cb_boat3_odom(self, msg):
        self.boat_odom[2] = msg

    def cb_boat4_odom(self, msg):
        self.boat_odom[3] = msg

    def cb_boat5_odom(self, msg):
        self.boat_odom[4] = msg

    def cb_boat6_odom(self, msg):
        self.boat_odom[5] = msg

    def cb_boat7_odom(self, msg):
        self.boat_odom[6] = msg
    


if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    rospy.spin()