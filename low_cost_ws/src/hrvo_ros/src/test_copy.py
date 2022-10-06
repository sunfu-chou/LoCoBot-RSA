#! /usr/bin/env python3

from turtle import distance
import rospy
from nav_msgs.msg import Odometry
from RVO import RVO_update, reach, compute_V_des, reach
from PID import PID_control
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, PoseStamped
import tf

# tsp
from python_tsp.heuristics import solve_tsp_simulated_annealing

# math
import math
import csv
import pickle as pkl
import numpy as np
import os
import random
import time

# read waypoint
waypoints = []
path = '/home/joe/LoCoBot-RSA/low_cost_ws/src/hrvo_ros/src/waypoints.txt'
with open(path,'r') as f:
    for line in f.readlines():
        p = line.split(' ')
        waypoints.append([int(p[0]),int(p[1])])

# hrvo function
def dist(p1, p2):
    """
        calculate the distance between two waypoints.
        Args:
            p1 : point's (x,y) position.
            p2 : point's (x,y) position.
        Returns:
            distance between two points.
    """
    return math.sqrt(((p1-p2)**2).sum())

def distanceGenerate(point_set):
    """
        generate a distance matrix based on the waypoints.
        Args:
            point_set : a set that contains all waypoint.
        Returns:
            a square matrix, which shows the distance between pairs of waypoint.
    """
    return np.asarray([[dist(np.array(p1), np.array(p2)) for p2 in point_set] for p1 in point_set])

def sortWaypoint(permutation, point_set):
    """
        accoriding to permutation calculated by tsp solver, return new set of waypoint which is sorted.
        Args:
            permutation : the order of waypoints calculated by tsp solver.
            point_set : a set that contains all waypoint.
        Returns:
            new set of waypoint which is sorted.
    """
    return [x for _, x in sorted(zip(permutation, point_set))]

class BoatHRVO(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)

        self.ws_model = dict()
        # robot radius
        self.ws_model['robot_radius'] = 3.6
        self.ws_model['circular_obstacles'] = []
        # rectangular boundary, format [x,y,width/2,heigth/2]
        self.ws_model['boundary'] = []

        # subscriber
        self.odom_sub = rospy.Subscriber("/test/odom", Odometry, self.cb_odom)

        #publisher
        self.pub_v = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

        # initiallize robot status
        self.boat_odom = Odometry()
        self.yaw = 0
        self.goal = [0,0]
        self.velocity_detect = [0,0]
        self.position = []

        self.begin = False
        self.waypoint_index = 0
        self.result_waypoints = []

        self.cmd_drive = Twist()
        self.velocity = [0, 0]
        self.v_max = [1]
        self.rate = rospy.Duration(0.1)

        while not rospy.is_shutdown():
            self.cb_hrvo()
            rospy.sleep(self.rate)

    def cb_odom(self, msg):

        self.boat_odom = msg
        # add start point into waypoints and solve TSP
        if not self.begin:
            start_point = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            waypoints.insert(0,start_point)
            distance_matrix = distanceGenerate(waypoints)
            permutation_sa, distance_sa = solve_tsp_simulated_annealing(distance_matrix)
            self.result_waypoints = sortWaypoint(permutation_sa, waypoints)
            self.begin = True

    def cb_hrvo(self):
        if self.waypoint_index != -1:
            if self.begin:
                self.update_all()
                v_des = compute_V_des(self.position, self.goal, self.v_max)
                self.velocity = RVO_update(self.position, v_des, self.velocity_detect, self.ws_model)
                dis, angle = self.process_ang_dis(self.velocity[0], self.velocity[1], self.yaw)
        
                cmd = Twist()
                cmd.linear.x = dis * 0.3
                cmd.angular.z = angle * 0.95
                self.cmd_drive = cmd
                self.pub_v.publish(self.cmd_drive)
            else:
                rospy.loginfo(" Waiting for odom")
        else:
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = 0
            self.cmd_drive = cmd
            self.pub_v.publish(self.cmd_drive)
            rospy.loginfo(" All points reached!!")


    # calculate distance between robot and current goal
    def goal_dist(self):
        dist = math.sqrt((self.goal[0] - self.position[0])**2 + (self.goal[1] - self.position[1])**2)
        return dist
        
    def update_all(self):
       # update position
        self.position = [self.boat_odom.pose.pose.position.x,
                         self.boat_odom.pose.pose.position.y]

        # update orientation
        quaternion = (self.boat_odom.pose.pose.orientation.x,
                      self.boat_odom.pose.pose.orientation.y,
                      self.boat_odom.pose.pose.orientation.z,
                      self.boat_odom.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

        # update velocity
        self.velocity_detect = [self.boat_odom.twist.twist.linear.x,
                                self.boat_odom.twist.twist.linear.y]

        if(self.goal_dist() < 0.5):
            # goal reached
            self.waypoint_index+=1

            # all points reached
            if(self.waypoint_index == (len(waypoints)+1)):
                self.waypoint_index = -1
                return

            self.goal = self.result_waypoints[self.waypoint_index]
            rospy.loginfo("Moving to No.{} point".format(self.waypoint_index))

    def process_ang_dis(self, vx, vy, yaw):
        dest_yaw = math.atan2(vy, vx)

        angle = dest_yaw - yaw
        if angle > np.pi:
            angle = angle-2*np.pi

        if angle < -np.pi:
            angle = angle+2*np.pi

        angle = angle/np.pi
        dis = math.sqrt(vx**2+vy**2)

        dis = max(min(dis, 1), -1)
        angle = max(min(angle, 1), -1)
        return dis, angle

if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    rospy.spin()
