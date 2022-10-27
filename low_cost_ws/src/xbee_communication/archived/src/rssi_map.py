#!/usr/bin/env python2

import rospy
import tf
import csv
import numpy as np
import pandas as pd
from xbee_communication.msg import NeighbourNode, NeighbourArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

node_dict = {}
data = []

rospy.init_node('export_rssi_csv', anonymous=True)
rospy.loginfo("Start rssi csv exporter")
listener = tf.TransformListener()
pose = PoseStamped()

def on_shutdown():
    with open("out01.csv", "wb") as f:
        writer = csv.writer(f)
        writer.writerows(data)
    
    print(node_dict)

def main():
    xbee_data_sub = rospy.Subscriber("/husky1/rssi_neighbour", NeighbourArray, nb_callback)
    robot_pose_sub = rospy.Subscriber("/husky1/husky_velocity_controller/odom", PoseWithCovarianceStamped, ps_callback)

    rospy.on_shutdown(on_shutdown)
    while not rospy.is_shutdown():
        rospy.spin()

def ps_callback(posecov):
    global listener
    global pose

    global index
    pose.pose = posecov.pose.pose
    pose.header = posecov.header
    rospy.loginfo('cb')
    listener.waitForTransform("odom", "slam_map", rospy.Time.now(), rospy.Duration(5))
    try:
        pose = listener.transformPose('slam_map', pose)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('e')
        pass


def nb_callback(neigh_arr):

    #rospy.loginfo('cb')
    global node_dict
    global data
    global pose

    data_entry = []
    data_entry.append(pose.pose.position.x)
    data_entry.append(pose.pose.position.y)

    for i in range(20):
        data_entry.append(0)

    query = neigh_arr.query_id
    rospy.loginfo('cb %s', str(query))
    for i in range(neigh_arr.length):
        node = NeighbourNode()
        node = neigh_arr.vec[i]
        index = 2

        if node.node_id == '':
            continue
        
        elif node.node_id[0] == ' ':
            node.node_id = node.node_id[1:]

        if node.node_id in node_dict:
            index = node_dict[node.node_id] + 2
        
        else:
            index = len(node_dict) + 2
            node_dict[node.node_id] = index - 2

        if node.query_id == query:
            data_entry[index] = node.rssi

    data.append(data_entry)

if __name__ == '__main__':
    main()
