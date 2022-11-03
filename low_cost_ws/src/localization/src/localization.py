#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray

from uwb import UWB
import time

uwb = UWB()

pose = PoseWithCovarianceStamped()
distances = Float64MultiArray()

last_pose = [0.0, 0.0, 0.0]


def uwb_error_handler():
    rospy.logwarn("Pozyx UWB error, reseatring")
    s = 'F,b0,,1'
    import serial
    ser = serial.Serial(uwb.port)
    ser.write(s.encode())
    return
    # uwb._pozyx_handler.resetSystem()


def timer_callback(e):
    now = rospy.Time.now()

    uwb.localize_2_5D()
    uwb.range_all()

    distances.data = [devicerange.distance for devicerange in uwb.ranges_all]

    distances_pub.publish(distances)

    if uwb.pose[0] == 0 and uwb.pose[1] == 0 and uwb.pose[2] == 0:
        uwb_error_handler()
        return

    if uwb.pose[0] == last_pose[0] and uwb.pose[1] == last_pose[1] and uwb.pose[2] == last_pose[2]:
        uwb_error_handler()
        return

    pose.header.stamp = now
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = uwb.pose[0] / 1000.0
    pose.pose.pose.position.y = uwb.pose[1] / 1000.0
    pose.pose.pose.position.z = uwb.pose[2] / 1000.0
    pose.pose.pose.orientation.w = 1.0
    pose.pose.covariance = [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pose_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("uwb_localization", anonymous=False)
    if uwb.connect():
        uwb.reset()
        rospy.loginfo("Pozyx UWB connected")
    if not uwb.connect():
        uwb.reset()
        rospy.loginfo('Connect error')
        exit()
    
    uwb.load_env_config(rospy.get_param("~config_file_path"))

    uwb.height = 950
    uwb.write_env_config()
    pose_pub = rospy.Publisher("uwb_pose", PoseWithCovarianceStamped, queue_size=10)
    distances_pub = rospy.Publisher("uwb_distances", Float64MultiArray, queue_size=10)
    localization_timer = rospy.Timer(rospy.Duration(0.05), timer_callback)
    rospy.spin()
