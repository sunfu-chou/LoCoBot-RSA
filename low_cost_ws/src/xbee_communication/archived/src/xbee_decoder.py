#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray
from subt_msgs.msg import SubTInfo, ArtifactPose, AnchorBallDetection, GloraPack, ArtifactPoseArray
import sys

def xbee_callback(msg_pre):

    msg = msg_pre.data
    case = msg[0]
    # case 0 => Testing case
    # case 1 => Apriltags Message Receive

    if case == "0":
        rospy.loginfo("This is a test message.")
        rospy.loginfo((msg[1:]))
    elif case == "1":
        rospy.loginfo("This is an Apriltags message.")

        at_array = AprilTagDetectionArray()

        secs = int(msg[1:10])
        nsecs = int(msg[10:19])
        at_array.header.stamp.secs = secs
        at_array.header.stamp.nsecs = nsecs
        time_offset = 18
        # Offset of each parameter
        id_offset = 0
        id_len = 3
        # px
        px_left_offset = id_offset+id_len
        px_left_len = 2
        px_right_offset = px_left_offset+px_left_len
        px_right_len = 3
        # py
        py_left_offset = px_right_offset+px_right_len
        py_left_len = 2
        py_right_offset = py_left_offset+py_left_len
        py_right_len = 3
        # pz
        pz_left_offset = py_right_offset+py_right_len
        pz_left_len = 2
        pz_right_offset = pz_left_offset+pz_left_len
        pz_right_len = 3
        # ox
        ox_left_offset = pz_right_offset+pz_right_len
        ox_left_len = 1
        ox_right_offset = ox_left_offset+ox_left_len
        ox_right_len = 12
        # oy
        oy_left_offset = ox_right_offset+ox_right_len
        oy_left_len = 1
        oy_right_offset = oy_left_offset+oy_left_len
        oy_right_len = 12
        # oz
        oz_left_offset = oy_right_offset+oy_right_len
        oz_left_len = 1
        oz_right_offset = oz_left_offset+oz_left_len
        oz_right_len = 12
        # ow
        ow_left_offset = oz_right_offset+oz_right_len
        ow_left_len = 1
        ow_right_offset = ow_left_offset+ow_left_len
        ow_right_len = 12
        # Watchtower id
        wat_id_offset = ow_right_offset+ow_right_len

        AT_OFFSET = ow_right_offset + ow_right_len

        position_offset = 50 # Assume the range of pose -49~49
        orientation_offset = 1 # Quaternion has range -1~1
        print "Len: ", len(msg[1+time_offset:])
        print "Offset: ", AT_OFFSET
        for i in range(len(msg[1+time_offset:])/AT_OFFSET):
            detection = AprilTagDetection()

            base_offset = i+1+time_offset
            detection.id.append(int(msg[base_offset:base_offset+id_len]))
            detection.pose.pose.pose.position.x = int(msg[base_offset+px_left_offset:base_offset+px_left_offset+px_left_len])+float(msg[base_offset+px_right_offset:base_offset+px_right_offset+px_right_len])/1000 -position_offset
            detection.pose.pose.pose.position.y = int(msg[base_offset+py_left_offset:base_offset+py_left_offset+py_left_len])+float(msg[base_offset+py_right_offset:base_offset+py_right_offset+py_right_len])/1000 -position_offset
            detection.pose.pose.pose.position.z = int(msg[base_offset+pz_left_offset:base_offset+pz_left_offset+pz_left_len])+float(msg[base_offset+pz_right_offset:base_offset+pz_right_offset+pz_right_len])/1000 -position_offset
            detection.pose.pose.pose.orientation.x = int(msg[base_offset+ox_left_offset:base_offset+ox_left_offset+ox_left_len])+float(msg[base_offset+ox_right_offset:base_offset+ox_right_offset+ox_right_len])/1000000000000 -orientation_offset
            detection.pose.pose.pose.orientation.y = int(msg[base_offset+oy_left_offset:base_offset+oy_left_offset+oy_left_len])+float(msg[base_offset+oy_right_offset:base_offset+oy_right_offset+oy_right_len])/1000000000000 -orientation_offset
            detection.pose.pose.pose.orientation.z = int(msg[base_offset+oz_left_offset:base_offset+oz_left_offset+oz_left_len])+float(msg[base_offset+oz_right_offset:base_offset+oz_right_offset+oz_right_len])/1000000000000 -orientation_offset
            detection.pose.pose.pose.orientation.w = int(msg[base_offset+ow_left_offset:base_offset+ow_left_offset+ow_left_len])+float(msg[base_offset+ow_right_offset:base_offset+ow_right_offset+ow_right_len])/1000000000000 -orientation_offset
            detection.pose.header.frame_id = msg[base_offset+wat_id_offset:]

            print "The Watchtower is: ",detection.pose.header.frame_id

            at_array.detections.append(detection)

        pub_apriltags.publish(at_array)
    elif case == "9":
        rospy.loginfo("This is an SubTInfo message.")

        strings = msg.split('#')
        print len(strings)
        info_msg = SubTInfo()

        #deconding header
        info_msg.header.seq = int(strings[1])
        info_msg.header.stamp.secs = int(strings[2])
        info_msg.header.stamp.nsecs = int(strings[3])
        info_msg.header.frame_id = strings[4]
        #deconding robot_name
        info_msg.robot_name = strings[5]
        #deconing robot_pose
        info_msg.robot_pose.position.x = float(strings[6])
        info_msg.robot_pose.position.y = float(strings[7])
        info_msg.robot_pose.position.z = float(strings[8])

        info_msg.robot_pose.orientation.x = float(strings[9])
        info_msg.robot_pose.orientation.y = float(strings[10])
        info_msg.robot_pose.orientation.z = float(strings[11])
        info_msg.robot_pose.orientation.w = float(strings[12])

        #deconding artifacts
        info_msg.artifacts.header.seq = int(strings[13])
        info_msg.artifacts.header.stamp.secs = int(strings[14])
        info_msg.artifacts.header.stamp.nsecs = int(strings[15])
        info_msg.artifacts.header.frame_id = strings[16]

        info_msg.artifacts.camera = strings[17]
        info_msg.artifacts.count = int(strings[18])
        info_msg.artifacts.pose_array = []
        for i in range(info_msg.artifacts.count):
            arti = ArtifactPose()
            arti.Class = strings[19+11*i]
            arti.status = strings[20+11*i]
            arti.appear_count = int(strings[21+11*i])
            arti.probability = int(strings[22+11*i])

            arti.pose.position.x = float(strings[23+11*i])
            arti.pose.position.y = float(strings[24+11*i])
            arti.pose.position.z = float(strings[25+11*i])

            arti.pose.orientation.x = float(strings[26+11*i])
            arti.pose.orientation.y = float(strings[27+11*i])
            arti.pose.orientation.z = float(strings[28+11*i])
            arti.pose.orientation.w = float(strings[29+11*i])

            info_msg.artifacts.pose_array.append(arti)


        pub_subt_info.publish(info_msg)
        pub_pose_list.publish(info_msg.artifacts)



if __name__ == '__main__':

    # prepare ROS node
    rospy.init_node('xbee_decoder',anonymous=False)

    # prepare ROS subscriber
    sub_string = rospy.Subscriber("xbee_receive", String, xbee_callback, queue_size=1)

    # prepare ROS publisher
    # might increase if the usage increase
#    pub_apriltags = rospy.Publisher("apriltags_from_xbee", AprilTagDetectionArray, queue_size=1)
    pub_subt_info = rospy.Publisher("xBee_subt_info", SubTInfo, queue_size=1)
    pub_pose_list = rospy.Publisher("artifact_pose_list", ArtifactPoseArray, queue_size=1)

    rospy.spin()
