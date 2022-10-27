#!/usr/bin/env python
import serial
import rospy
import struct
import math
import time
import copy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Header


class EstopMonitor():
    def __init__(self):
        self.DEFAULT_PORT = rospy.get_param("~port","/dev/ftdi_DN066LP0")
        rospy.loginfo("port %s"%self.DEFAULT_PORT)
        # 0x 7E 00 0B 88 01 49 53 00 01 00 02 00 00 00 D5
        self.HIGH_STATE = bytearray([126, 0, 11, 136, 1, 73, 83, 0, 1, 0, 2, 0, 0, 2, 213])
        # 0x 7E 00 0B 88 01 49 53 00 01 00 02 00 00 00 D7
        self.LOW_STATE = bytearray([126, 0, 11, 136, 1, 73, 83, 0, 1, 0, 2, 0, 0, 0, 215])

        self.ESTOP_STATE = False
        self.pub_estop = rospy.Publisher("e_stop_xbee", Bool, queue_size=1)
        self.serial_link = serial.Serial(self.DEFAULT_PORT, 9600)

        rospy.Timer(rospy.Duration(1), self.cb_time)

    def cb_time(self, event):
        # note = 0x 7E 00 04 08 01 49 53 5A
        note = bytearray([126, 0, 4, 8, 1, 73, 83, 90])
        print "Write to xbee: ", list(note)
        self.serial_link.write(note)
        time.sleep(0.1)

        status = self.serial_link.read(self.serial_link.inWaiting())
        print "Serial status: ", list(status)

        if status == self.LOW_STATE:
            print "Normal State"
            self.pub_estop.publish(False)

        elif status == self.HIGH_STATE:
            print "Emergency Stop !!!"
            self.ESTOP_STATE = True
            self.pub_estop.publish(True)
            
        else:
            print "Other State"


if __name__ == "__main__":
    rospy.init_node("estop_monitor")
    rospy.loginfo("[%s] Initializing " % (rospy.get_name()))
    monitor = EstopMonitor()
    rospy.spin()
