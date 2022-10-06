#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from RVO import RVO_update, reach, compute_V_des, reach
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import math
import tf
import numpy as np
import os
import pickle as pkl


class BoatHRVO(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)

        # #initiallize PID
        # self.dis_pid = [PID_control("distance_control%d" % i) for i in range(4)]
        # self.angu_pid = [PID_control("angular_control%d" % i) for i in range(4)]
        # self.dis_server = Server(dis_PIDConfig,self.cb_dis_pid,"distance_control")
        # self.ang_server = Server(ang_PIDConfig,self.cb_ang_pid,"angular_control")
        # for i in range(4):
        #     self.dis_pid[i].setSampleTime(0.1)
        #     self.angu_pid[i].setSampleTime(0.1)
        #     self.dis_pid[i].SetPoint = 0
        #     self.angu_pid[i].SetPoint = 0

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_model = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)

        # setup publisher
        self.pub_v1 = rospy.Publisher("/boat1/cmd_vel", Twist, queue_size=1)
        self.sub_p3d1 = rospy.Subscriber(
            "/boat1/p3d_odom", Odometry, self.cb_boat1_odom, queue_size=1)

        self.pub_v2 = rospy.Publisher("/boat2/cmd_vel", Twist, queue_size=1)
        self.sub_p3d1 = rospy.Subscriber(
            "/boat2/p3d_odom", Odometry, self.cb_boat2_odom, queue_size=1)

        self.pub_v3 = rospy.Publisher("/boat3/cmd_vel", Twist, queue_size=1)
        self.sub_p3d1 = rospy.Subscriber(
            "/boat3/p3d_odom", Odometry, self.cb_boat3_odom, queue_size=1)

        self.pub_v4 = rospy.Publisher("/boat4/cmd_vel", Twist, queue_size=1)
        self.sub_p3d1 = rospy.Subscriber(
            "/boat4/p3d_odom", Odometry, self.cb_boat4_odom, queue_size=1)

        # initiallize boat status
        self.boat_odom = [Odometry() for i in range(4)]
        self.cmd_drive = [Twist() for i in range(4)]
        self.yaw = [0 for i in range(4)]
        # initiallize HRVO environment
        self.ws_model = dict()
        # robot radius
        self.ws_model['robot_radius'] = 1.5
        self.ws_model['circular_obstacles'] = []
        # rectangular boundary, format [x,y,width/2,heigth/2]
        self.ws_model['boundary'] = []

        self.position = []
        self.goal = []
        self.random_init()
        # print(self.position)
        # print(self.goal)
        self.velocity = [[0, 0] for i in range(4)]
        self.velocity_detect = [[0, 0] for i in range(4)]
        self.v_max = [1 for i in range(4)]

        # timer
        self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_hrvo)

    def random_init(self):
        self.goal = []
        for i in range(4):
            state, g = self.get_initial_state("boat%d" % (i+1), i)
            self.reset_model(state)
            self.goal.append(g)

    def get_initial_state(self, name, id):
        # start position
        state_msg = ModelState()
        state_msg.model_name = name

        r = 8
        degree = np.random.uniform(-30, 30) + id*90
        yaw = math.radians(degree)

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw-np.pi)
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]

        state_msg.pose.position.x = r * math.cos(yaw)
        state_msg.pose.position.y = r * math.sin(yaw)
        state_msg.pose.position.z = 0

        goal = [r * -math.cos(yaw), r * -math.sin(yaw)]

        return state_msg, goal

    def cb_hrvo(self, event):

        epoch = 0
        iteration = 60
        record = np.zeros([iteration])

        for i in range(iteration):
            rospy.sleep(0.5)
            while True:
                self.update_all()
                v_des = compute_V_des(self.position, self.goal, self.v_max)
                self.velocity = RVO_update(
                    self.position, v_des, self.velocity_detect, self.ws_model)

                for i in range(4):
                    dis, angle = self.process_ang_dis(
                        self.velocity[i][0], self.velocity[i][1], self.yaw[i])

                    cmd = Twist()
                    cmd.linear.x = dis * 0.4
                    cmd.angular.z = angle * 0.4
                    self.cmd_drive[i] = cmd

                self.pub_v1.publish(self.cmd_drive[0])
                self.pub_v2.publish(self.cmd_drive[1])
                self.pub_v3.publish(self.cmd_drive[2])
                self.pub_v4.publish(self.cmd_drive[3])

                min_dis, done = self.check_state()
                # print min_dis
                if min_dis < 1.7:
                    self.random_init()
                    record[epoch] = 0
                    break
                if done:
                    self.random_init()
                    record[epoch] = 1
                    break

            print epoch, record[epoch]

            epoch += 1

        folder = os.getcwd()
        record_name = 'hrvo_multi.pkl'
        fileObject = open(folder+"/"+record_name, 'wb')

        pkl.dump(record, fileObject)
        fileObject.close()
        exit()

    def check_state(self):
        min_dis = 1e9
        done = True
        for i in range(4):
            p = np.array(self.position[i])
            g = np.array(self.goal[i])
            d2g = np.linalg.norm(p-g)
            done = True if d2g < 2 else False
            for k in range(i+1, 4):
                p1 = np.array(self.position[i])
                p2 = np.array(self.position[k])
                dis = np.linalg.norm(p1-p2)
                min_dis = dis if dis < min_dis else min_dis
        return min_dis, done

    def process_ang_dis(self, vx, vy, yaw):
        dest_yaw = math.atan2(vy, vx)

        angle = dest_yaw - yaw
        if angle > np.pi:
            angle = angle-2*np.pi

        if angle < -np.pi:
            angle = angle+2*np.pi

        angle = angle/np.pi

        dis = math.sqrt(vx**2+vy**2)

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
        for i in range(4):
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

            # update velocity
            self.velocity_detect[i] = [self.boat_odom[i].twist.twist.linear.x,
                                       self.boat_odom[i].twist.twist.linear.y]

    def cb_boat1_odom(self, msg):
        self.boat_odom[0] = msg

    def cb_boat2_odom(self, msg):
        self.boat_odom[1] = msg

    def cb_boat3_odom(self, msg):
        self.boat_odom[2] = msg

    def cb_boat4_odom(self, msg):
        self.boat_odom[3] = msg

    def cb_dis_pid(self, config, level):
        print(
            "distance: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        for i in range(4):
            self.dis_pid[i].setKp(Kp)
            self.dis_pid[i].setKi(Ki)
            self.dis_pid[i].setKd(Kd)
        return config

    def cb_ang_pid(self, config, level):
        print(
            "angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        for i in range(4):
            self.angu_pid[i].setKp(Kp)
            self.angu_pid[i].setKi(Ki)
            self.angu_pid[i].setKd(Kd)
        return config


if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    rospy.spin()
