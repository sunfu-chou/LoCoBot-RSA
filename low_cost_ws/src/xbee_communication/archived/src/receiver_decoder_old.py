#!/usr/bin/env python
import serial
import rospy
import struct
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Header


class Receiver(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        #setup serial
        self.baud_rate = rospy.get_param("baud_rate",115200)
        self.port_name = rospy.get_param("port_name","/dev/lora_arduino")
        self.ser = serial.Serial(self.port_name,self.baud_rate)

        # Publications
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)

        while True:
            cmd_data = self.ser.readline()
            #print(len(cmd_data))
            try: 
                data = struct.unpack("8fss",cmd_data)
                print(data)
                odom = Odometry()
                odom.header.stamp = rospy.Time.from_sec(data[0])
                odom.pose.pose.position.x = data[1]
                odom.pose.pose.position.y = data[2]
                odom.pose.pose.position.z = data[3]
                odom.pose.pose.orientation.x = data[4]
                odom.pose.pose.orientation.y = data[5]
                odom.pose.pose.orientation.z = data[6]
                odom.pose.pose.orientation.w = data[7]
                self.pub_odom.publish(odom)
            except:
                print("data currupt")

    def on_shutdown(self):
        rospy.loginfo("shutting down [%s]" %(self.node_name))

if __name__ == "__main__":
    rospy.init_node("lora_receiver",anonymous=False)
    transmission = Receiver()
    rospy.on_shutdown(transmission.on_shutdown)
    rospy.spin()