#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray
from subt_msgs.msg import SubTInfo, ArtifactPose, AnchorBallDetection, GloraPack
import sys

def case_0_test(msg):

    sent_msg = "0"
    sent_msg += msg.data

    pub_string.publish(sent_msg)

def case_1_apriltags(msg):

    sent_msg = "1"

    # Don't process if no detections
    if len(msg.detections) == 0:
        return

    rospy.loginfo("Encoding Apriltags message.")

    def num_to_str(num, offset, left_len, right_len):
        n = num + offset
        n_str = str(n).split('.')
        final_str = ""
        final_left = n_str[0][-left_len:]
        while(True):
            if len(final_left)<left_len:
                final_left = "0"+final_left
                continue
            break
        if right_len > 0: # If we need the number on the right of decimal point
            final_right = n_str[1][:right_len]
            while(True):
                if len(final_right)<right_len:
                    final_right = final_right+"0"
                    continue
                break
            final_str = final_left + final_right
        else:
            final_str = final_left
        return final_str

    sent_msg = sent_msg + num_to_str(msg.header.stamp.secs, 0, 9, 0)
    sent_msg = sent_msg + num_to_str(msg.header.stamp.nsecs, 0, 9, 0)

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

    AT_OFFSET = ow_right_offset + ow_right_len

    position_offset = 50 # Assume the range of pose -49~49
    orientation_offset = 1 # Quaternion has range -1~1

    for detection in msg.detections:

        if detection.id[0] < 10:
            sent_msg = sent_msg + "00" + str(detection.id[0])
        elif detection.id[0] < 100:
            sent_msg = sent_msg + "0" + str(detection.id[0])
        else:
            sent_msg = sent_msg + str(detection.id[0])

        sent_msg = sent_msg + num_to_str(detection.pose.pose.pose.position.x, position_offset, px_left_len, px_right_len)
        # print "px:", sent_msg
        sent_msg = sent_msg + num_to_str(detection.pose.pose.pose.position.y, position_offset, py_left_len, py_right_len)
        # print "py:", sent_msg
        sent_msg = sent_msg + num_to_str(detection.pose.pose.pose.position.z, position_offset, pz_left_len, pz_right_len)
        # print "pz:", sent_msg
        sent_msg = sent_msg + num_to_str(detection.pose.pose.pose.orientation.x, orientation_offset, ox_left_len, ox_right_len)
        # print "ox:", sent_msg
        sent_msg = sent_msg + num_to_str(detection.pose.pose.pose.orientation.y, orientation_offset, oy_left_len, oy_right_len)
        # print "oy:", sent_msg
        sent_msg = sent_msg + num_to_str(detection.pose.pose.pose.orientation.z, orientation_offset, oz_left_len, oz_right_len)
        # print "oz:", sent_msg
        sent_msg = sent_msg + num_to_str(detection.pose.pose.pose.orientation.w, orientation_offset, ow_left_len, ow_right_len)
        # print "wat_id:", sent_msg
        import socket
        sent_msg = sent_msg + socket.gethostname()

    pub_string.publish(sent_msg)


def case_2_subtinfo(msg):
    rospy.loginfo("Encoding subt info message.")
    sent_msg = "9#"
    #cascading header 1~4
    sent_msg = sent_msg + str(msg.header.seq) + "#" #uint32 1
    sent_msg = sent_msg + str(msg.header.stamp.secs) + "#" #uint32 2
    sent_msg = sent_msg + str(msg.header.stamp.nsecs) + "#" #uint32 3
    sent_msg = sent_msg + msg.header.frame_id + "#" #string 4
    #cascading robot_name 5
    sent_msg = sent_msg + msg.robot_name + "#" #string 5
    #cascading robot_pose 6~12
    sent_msg = sent_msg + str(msg.robot_pose.position.x) + "#" #float64 6
    sent_msg = sent_msg + str(msg.robot_pose.position.y) + "#" #float64 7
    sent_msg = sent_msg + str(msg.robot_pose.position.z) + "#" #float64 8

    sent_msg = sent_msg + str(msg.robot_pose.orientation.x) + "#" #float64 9
    sent_msg = sent_msg + str(msg.robot_pose.orientation.y) + "#" #float64 10
    sent_msg = sent_msg + str(msg.robot_pose.orientation.z) + "#" #float64 11
    sent_msg = sent_msg + str(msg.robot_pose.orientation.w) + "#" #float64 12

    #cascading artifacts
    sent_msg = sent_msg + str(msg.artifacts.header.seq) + "#" #13~16 uint32 13
    sent_msg = sent_msg + str(msg.artifacts.header.stamp.secs) + "#" #uint32 14
    sent_msg = sent_msg + str(msg.artifacts.header.stamp.nsecs) + "#" #uint32 15
    sent_msg = sent_msg + msg.artifacts.header.frame_id + "#" #string 16

    sent_msg = sent_msg + msg.artifacts.camera + "#" #string 17
    sent_msg = sent_msg + str(msg.artifacts.count) + "#" #int32 18
    for i in range(msg.artifacts.count):
        artifact_pose = msg.artifacts.pose_array[i]
        sent_msg = sent_msg + artifact_pose.Class + "#" #string 19+11k
        sent_msg = sent_msg + artifact_pose.status + "#" #string 20+11k
        sent_msg = sent_msg + str(artifact_pose.appear_count) + "#" #int32 21+11k
        sent_msg = sent_msg + str(artifact_pose.probability) + "#" #float64 22+11k

        sent_msg = sent_msg + str(artifact_pose.pose.position.x) + "#"
        sent_msg = sent_msg + str(artifact_pose.pose.position.y) + "#"
        sent_msg = sent_msg + str(artifact_pose.pose.position.z) + "#"

        sent_msg = sent_msg + str(artifact_pose.pose.orientation.x) + "#"
        sent_msg = sent_msg + str(artifact_pose.pose.orientation.y) + "#"
        sent_msg = sent_msg + str(artifact_pose.pose.orientation.z) + "#"
        sent_msg = sent_msg + str(artifact_pose.pose.orientation.w) + "#"

    pub_string.publish(sent_msg)


if __name__ == '__main__':

    # prepare ROS node
    rospy.init_node('xbee_encoder',anonymous=False)

    # prepare ROS publisher
    pub_string = rospy.Publisher("xbee_send", String, queue_size=1)

    # prepare ROS publisher
    # might increase if the usage increase
    #sub_test = rospy.Subscriber("just_for_test", String, case_0_test, queue_size=1)
    # import socket
    # ap_topic = socket.gethostname() + "/tag_detections"
#    sub_apriltags = rospy.Subscriber("tag_detections", AprilTagDetectionArray, case_1_apriltags, queue_size=1)

    sub_subt_info = rospy.Subscriber("/subt_info", SubTInfo, case_2_subtinfo, queue_size=1)
    rospy.spin()
