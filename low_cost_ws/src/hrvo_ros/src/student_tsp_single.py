#! /usr/bin/env python3

from turtle import distance
import rospy
from nav_msgs.msg import Odometry
from RVO import RVO_update, reach, compute_V_des, reach
from PID import PID_control
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, PoseStamped
import tf
import sys
import os
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
path = os.path.dirname(__file__).__str__() + "/waypoints.txt"
try:
    with open(path,'r') as f:
        for line in f.readlines():
            p = line.split(' ')
            waypoints.append([float(p[0]),float(p[1])])
    print("Successfully open/read file: " + path + "\n")
except OSError:
    print("Could not open/read file: " + path)
    sys.exit()

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
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.cb_odom)

        #publisher
        self.pub_v = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

        # initiallize robot status
        self.boat_odom = Odometry()
        self.yaw = 0
        self.goal = []
        self.velocity_detect = [[0,0]]
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
            self.goal.append(self.result_waypoints[0])
            self.begin = True

    def cb_hrvo(self):
        if self.waypoint_index != -1:
            if self.begin:
                self.update_all()
                v_des = compute_V_des(self.position, self.goal, self.v_max)
                self.velocity = RVO_update(self.position, v_des, self.velocity_detect, self.ws_model)
                dis, angle = self.process_ang_dis(self.velocity[0][0], self.velocity[0][1], self.yaw)
                ###################################TODO############################################
                # update linear velocity : dis & angular velocity : angle 
                # multiply linear velocity by factor < 0.4
                # multiply angular velocity by factor 2



                ###################################################################################
                self.pub_v.publish(self.cmd_drive)
                
            else:
              rospy.loginfo(" Waiting for odom")
        else:
            ##################################TODO############################################
            # update linear velocity : 0 & angular velocity : 0
            



            #################################################################################
            self.pub_v.publish(self.cmd_drive)
            rospy.loginfo(" All points reached!!")


    # calculate distance between robot and current goal
    def goal_dist(self):
        dist = math.sqrt((self.goal[0][0] - self.position[0][0])**2 + (self.goal[0][1] - self.position[0][1])**2)
        return dist
        
    def update_all(self):

        #################################TODO#######################################
       # update position
       # hint : in self.boat_odom
        self.position = []
        pos = [ , ]
        self.position.append(pos)
        
        # update orientation
        # hint : in self.boat_odom 
        quaternion = ( , , , )

        # hint : transform quaternion to euler by using tf function
        euler = 
        self.yaw = euler[2]
        

        # update velocity
        # hint : in self.boat_odom
        self.velocity_detect[0] = [ , ]
        ###############################################################################
        if(self.goal_dist() < 0.3):
            # goal reached
            self.waypoint_index+=1

            # all points reached
            if(self.waypoint_index == (len(waypoints))):
                self.waypoint_index = -1
                return
            else:
                self.goal = []
                self.goal.append(self.result_waypoints[self.waypoint_index])
                rospy.loginfo("Moving to No.{} point".format(self.waypoint_index) + " at {}".format(self.goal[0]))

    def process_ang_dis(self, vx, vy, yaw):
        ######################### TODO ###################################
        # compute destination angle
        # hint : math.atan2( , )
        

        # compute angle difference of destination angle and current angle

        
        # make sure -np.pi< angle < np.pi
        

        ####################################################################
        angle = angle/np.pi
        dis = math.sqrt(vx**2+vy**2)
        dis = max(min(dis, 1), -1)
        angle = max(min(angle, 1), -1)
        return dis, angle

if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    rospy.spin()
