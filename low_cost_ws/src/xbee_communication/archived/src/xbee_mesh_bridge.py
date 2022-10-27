#!/usr/bin/env python3

#
# Subscribe: /xbee_send(String)
# Publish: /xbee_receive(String)
#
#

from digi.xbee.devices import XBeeDevice
import rospy
from std_msgs.msg import String, Int64
from std_srvs.srv import SetBool
from subt_msgs.srv import start, pause, stop
import random
import time
count = 0

MAX_MEMORY_SEC = 100.0
packetDict = { }

class XBee_Operation(object):
    def __init__(self):
        self.PORT = rospy.get_param("~port")
        self.VEH = rospy.get_param("~veh")
        self.BAUD_RATE = 9600
        self.device = XBeeDevice(self.PORT, self.BAUD_RATE)
        self.sub_msg = rospy.Subscriber("xbee_send", String, self.ros_msg_cb, queue_size=10)
        self.pub_msg = rospy.Publisher("xbee_receive", String, queue_size=1)
        self.start = rospy.ServiceProxy('RL/start', start)
        self.joy = rospy.ServiceProxy('RL/stop', stop)
        
        if self.VEH == 'husky1':
            self.pub_node_count = rospy.Publisher("node_count_h2", Int64, queue_size=1)
            self.timer = rospy.Timer(rospy.Duration(1),self.cb_timer)
        
        if self.VEH == 'husky2':
            self.pub_node_count = rospy.Publisher("node_count_h1", Int64, queue_size=1)
            self.timer = rospy.Timer(rospy.Duration(1),self.cb_timer)
        
        if self.VEH == 'glora':
            self.baseop_auto = rospy.Service('baseop_auto_h1', SetBool, self.cb_baseopauto_h1)
            self.baseop_manu = rospy.Service('baseop_auto_h2', SetBool, self.cb_baseopauto_h2)

        self.device.open()
        self.device.add_data_received_callback(self.xbee_msg_cb)
        self.sourceAddr = str(self.device.get_64bit_addr())[-4:]
        rospy.loginfo("xbee initialized")

    def cb_baseopauto_h1(self, req):
        if req.data == True:
            rospy.loginfo("Sending autonomous command to h1")
            self.msg_cb("71a")
        elif req.data == False:
            rospy.loginfo("Sending manual command to h1")
            self.msg_cb("71m")

        return [True, "Sent"]

    def cb_baseopauto_h2(self, req):
        if req.data == True:
            rospy.loginfo("Sending autonomous command to h2")
            self.msg_cb("72a")
        elif req.data == False:
            rospy.loginfo("Sending manual command to h2")
            self.msg_cb("72m")

        return [True, "Sent"]


    # TODO: Encoding message
    def ros_msg_cb(self, msg):
        #rospy.loginfo("ros_msg cb")
        #rospy.loginfo(msg.data)

        message = msg.data
        packetID = str(random.randint(1000, 9999))
        #print(packetID + "," + self.sourceAddr)
        self.device.send_data_broadcast(packetID + self.sourceAddr + message)

    #Handler for primitive string message
    def msg_cb(self, msg):
        #rospy.loginfo("msg cb")
        #rospy.loginfo(msg)

        packetID = str(random.randint(1000, 9999))
        #print(packetID + "," + self.sourceAddr)
        self.device.send_data_broadcast(packetID + self.sourceAddr + msg)

    def cb_timer(self, event):
        global count
        rospy.loginfo("Publishing node count")
        self.pub_node_count.publish(count)


    # TODO: Decoding message
    def xbee_msg_cb(self, xbee_msg):
        global count
        #rospy.loginfo("xbee_msg cb")
        data = xbee_msg.data.decode("utf8")
        packetID = data[:4]
        sourceAddr = data[4:8]
        message = data[8:]

        if sourceAddr == self.sourceAddr: #packetbyself
            return
        if packetID in packetDict:
            return
        else:
            packetDict[packetID] = [time.time(), message]
            currTime = time.time()
            key_copy = tuple(packetDict.keys())
            for pack in key_copy:
                if currTime - packetDict[pack][0] > MAX_MEMORY_SEC:
                    packetDict.pop(pack)

        #anchorball count msg
        if message[0] == "8":
            if self.VEH == "husky1" and message[1] == "2":
                if int(message[2])>count:
                    count = int(message[2])
            if self.VEH == "husky2" and message[1] == "1":
                if int(message[2])>count:
                    count = int(message[2])

        #autonomous command report
        elif message[0] == "6":
            if self.VEH == "glora":
                rospy.loginfo("Husky %s go %s", message[1], message[2])

        #autonomous command switch
        elif message[0] == "7":
            if message[1] == "1" and self.VEH == "husky1":
                if (message[2] == "a"):
                    #service call start autonomous
                    try:
                        self.start()
                        rospy.loginfo("go autonomous")
                        self.msg_cb("61a") #send acknowledge message
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call failed: %s", e)
                elif (message[2] == "m"):
                    #service call stop autonomous
                    try:
                        self.joy()
                        rospy.loginfo("go manual")
                        self.msg_cb("61m") #send acknowledge message
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call failed: %s", e)

            elif message[1] == "2" and self.VEH == "husky2":
                if message[2] == "a":
                    #service call start autonomous
                    try:
                        self.start()
                        rospy.loginfo("go autonomous")
                        self.msg_cb("62a") #send acknowledge message
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call failed: %s", e)
                elif message[2] == "m":
                    #service call stop autonomous
                    try:
                        self.joy()
                        rospy.loginfo("go manual")
                        self.msg_cb("62m") #send acknowledge message
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call failed: %s", e)

        else:
            # rospy.loginfo(message)
            self.pub_msg.publish(message)

    def on_shutdown(self):
        if self.device is not None and self.device.is_open():
            self.device.close()

if __name__ == "__main__":
    rospy.init_node("xbee_operation")

    xbee_node = XBee_Operation()
    try:
        rospy.spin()
    finally:
        xbee_node.on_shutdown()
